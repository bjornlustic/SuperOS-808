/*
 * flash_service.c -- SPM flash-write service for SuperOS-808.
 *
 * SPM (flash self-write) can ONLY execute from the boot section, so the app
 * cannot write flash directly. This tiny service lives in the otherwise-empty
 * top of the boot section, above the bootloader body, and the running app calls
 * into it.
 *
 * Install: built as its own image and packed into service-install.syx, which
 * writes only the service pages via the existing bootloader SysEx updater. The
 * running bootloader (pages 0x1F0..0x1F7) is never touched, so there is no
 * brick risk and a failed install is retryable.
 *
 * THREE entries, pinned by the linker at fixed addresses so the app can call
 * them by absolute word address forever. avr:51 function pointers are WORD
 * addresses, so byte 0x1FE00 is word 0xFF00.
 *
 *   0x1FE00  program_page(page, buf)        arena pages only, no key
 *   0x1FE04  stage_page(page, buf, key)     staging window only, key
 *   0x1FE08  commit_boot(key)               staging -> bootloader body, key
 *
 * The split is the safety model. The app can never name a boot-section page to
 * write: the only routine that touches 0x1F1..0x1F8 is commit_boot, and its
 * source is hard-wired to the staging window. Page 0x1F0, the bootloader's
 * permanent reset stub, is not a destination at all. And nothing here will ever
 * program any of the pages this service itself occupies.
 *
 * commit_boot is also callable BY that stub, at reset, to finish a commit that
 * a power cut interrupted. See the note on its definition.
 *
 * So each half updates the other and neither updates itself:
 *   the app (+ this service) rewrites the bootloader,
 *   the bootloader's SysEx updater rewrites this service.
 * Whichever code is running the erase is never the code being erased.
 *
 * ABI (avr-gcc default): first arg r24:r25, second r22:r23, third r20:r21,
 * status back in r24.
 */

#include <avr/io.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#define PAGE_SIZE 256

/* Emulated-EEPROM arena = 0x17E00..0x1DFFF (RWW, below the 0x1E000 NRWW edge).
 * The unprivileged entry refuses anything outside it, so a buggy app can never
 * erase the application code, the boot section, or the arena's neighbours.
 *
 * The lower bound tracks flash_eeprom.h and has been raised twice as the arena
 * shrank. It matters more than it looks: everything from 0xE000 up to the arena
 * is now .natcode, the statically translated emulator program, which is
 * ordinary flash that the running emulator executes from. A window that still
 * started at 0xE000 would let a stale or buggy app hand this routine a page
 * number that erases the code currently executing. */
#define ARENA_FIRST_PAGE 0x17Eu /* 0x17E00 / 256 -> arena base (flash_eeprom.h) */
#define ARENA_LAST_PAGE  0x1DFu /* 0x1DF00 / 256 -> last page fully below 0x1E000 */

/* Bootloader staging window: 2 KB of application-region flash immediately below
 * the arena. A replacement bootloader is received into here over USB or DIN and
 * checked at leisure; only then is it committed. The largest build (combined)
 * ends around page 0xB6, so this never collides with app code, and it stops one
 * page short of the arena so it never collides with stored patterns either. */
#define STAGE_FIRST_PAGE 0xD8u  /* 0xD800 */
#define STAGE_LAST_PAGE  0xDFu  /* 0xDFFF, one below ARENA_FIRST_PAGE */

/* The resident bootloader's BODY: the destination of a commit and of nothing
 * else. Page 0x1F0 is deliberately excluded. That page is the bootloader's
 * permanent reset stub and the routine that finishes an interrupted commit, so
 * it has to survive exactly the accident it exists to repair. Nothing here can
 * write it; changing it is an ISP job. */
#define BOOT_FIRST_PAGE  0x1F1u
#define BOOT_LAST_PAGE   0x1F8u
#define BOOT_PAGES       (BOOT_LAST_PAGE - BOOT_FIRST_PAGE + 1)

/* Commit arm flag, one page below the staging window. Written before the first
 * boot page is erased and erased once the last one is written, so its presence
 * at reset means a commit was cut short. The bootloader's stub checks it and
 * calls back in here, which is what makes an interrupted commit survivable.
 * Kept out of the staging window so a re-upload cannot disturb it. */
#define CTRL_PAGE        0xD7u
#define CTRL_ADDR        0xD700UL
#define CTRL_MAGIC       0xA53C

/* Callers of the privileged entries must present this. It is not security (the
 * app is trusted code); it stops a wild ICALL or a corrupted function pointer
 * from landing on the one routine that can unmake the bootloader. */
#define SVC_KEY 0x9E37u

#define SVC_OK        0
#define SVC_REJECTED  1  /* page outside the range this entry serves */
#define SVC_BAD_KEY   2

/* Trampolines. Pinned at 0x1FE00 by the linker; the three JMPs follow each
 * other at 4 bytes apiece, so the entry addresses are 0x1FE00 / 0x1FE04 /
 * 0x1FE08. Naked and in order: nothing may be inserted between them. The
 * argument registers pass straight through. */
__attribute__((section(".service_entry"), naked, used))
void flash_service_entry(void) {
  __asm__ __volatile__(
    "jmp flash_service_impl        \n\t"   /* 0x1FE00 */
    "jmp flash_service_stage_impl  \n\t"   /* 0x1FE04 */
    "jmp flash_service_commit_impl \n\t"); /* 0x1FE08 */
}

/* Erase + write one page from RAM. No range check of its own: every caller
 * below has already decided the page is one it is allowed to touch. */
static uint8_t program_page(uint16_t page, const uint8_t *buf) {
  /* page * 256 via shift so no __mul libgcc call is needed under -nostdlib. */
  uint32_t addr = (uint32_t)page << 8;

  uint8_t sreg = SREG;
  cli(); /* interrupt vectors live in RWW and are unreadable during the write */

  eeprom_busy_wait();

  boot_page_erase(addr);
  boot_spm_busy_wait();

  for (uint16_t i = 0; i < PAGE_SIZE; i += 2) {
    uint16_t w = (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8);
    boot_page_fill(addr + i, w);
  }

  boot_page_write(addr);
  boot_spm_busy_wait();
  boot_rww_enable();

  RAMPZ = 0; /* boot_page_* (extended) left RAMPZ set; restore for plain LPM */
  SREG = sreg;
  return SVC_OK;
}

/* 0x1FE00 -- the original entry. Arena pages only, and deliberately unchanged:
 * every already-installed app calls this address with this behaviour. */
__attribute__((used))
uint8_t flash_service_impl(uint16_t page, const uint8_t *buf) {
  if (page < ARENA_FIRST_PAGE || page > ARENA_LAST_PAGE)
    return SVC_REJECTED;
  return program_page(page, buf);
}

/* 0x1FE04 -- receive one page of a replacement bootloader into the staging
 * window. Writes nothing outside that window, so an interrupted or nonsense
 * upload costs only the staging area. */
__attribute__((used))
uint8_t flash_service_stage_impl(uint16_t page, const uint8_t *buf, uint16_t key) {
  if (key != SVC_KEY)
    return SVC_BAD_KEY;
  if (page < STAGE_FIRST_PAGE || page > STAGE_LAST_PAGE)
    return SVC_REJECTED;
  return program_page(page, buf);
}

/* 0x1FE08 -- commit the staged image: copy the eight staging pages onto the
 * eight bootloader body pages. Source and destination are both fixed constants,
 * so there is no argument that can redirect this at another page.
 *
 * The caller is expected to have verified the staged bytes first (it can read
 * them with ordinary far program-memory reads); this routine only moves them.
 *
 * Reading the staging window between writes is safe: staging is RWW and only a
 * write TO RWW disturbs RWW reads, while these writes all go to NRWW, during
 * which the CPU is halted outright.
 *
 * POWER-FAIL BEHAVIOUR, which is the reason for the arm flag. The flag page is
 * written before the first boot page is erased and erased after the last one is
 * written, so at any instant when the boot body could be inconsistent the flag
 * is set. Reset vectors into the bootloader's permanent page, which is never a
 * destination here; its stub reads the flag and calls straight back into this
 * routine, so an interrupted commit simply finishes on the next power-up.
 *
 * Every interruption point is covered, and re-entry is idempotent:
 *   during the arm write   -> flag lands or it does not. Set: we copy verified
 *                             bytes over a body that was never touched. Clear:
 *                             the body was never touched and nothing is owed.
 *   during the page copy   -> flag is set, recovery re-copies from staging.
 *   during the disarm      -> flag reads set or clear. Set: recovery re-copies
 *                             the same bytes and disarms again. Clear: done.
 * The one thing that would break this is losing the staging window, which is
 * why nothing writes there during a commit, and why an ISP recovery (which
 * chip-erases, clearing the flag) cannot be undone by a stale arm. */
__attribute__((used))
uint8_t flash_service_commit_impl(uint16_t key) {
  if (key != SVC_KEY)
    return SVC_BAD_KEY;

  uint8_t sreg = SREG;
  cli();

  eeprom_busy_wait();

  /* Arm. Fill the whole page explicitly rather than trusting the temporary
     page buffer to be erased on entry: only word 0 carries meaning, but the
     other 127 must be known. */
  boot_page_erase(CTRL_ADDR);
  boot_spm_busy_wait();
  for (uint16_t i = 0; i < PAGE_SIZE; i += 2)
    boot_page_fill(CTRL_ADDR + i, i ? 0xFFFF : CTRL_MAGIC);
  boot_page_write(CTRL_ADDR);
  boot_spm_busy_wait();

  for (uint8_t p = 0; p < BOOT_PAGES; ++p) {
    uint32_t src = (uint32_t)(STAGE_FIRST_PAGE + p) << 8;
    uint32_t dst = (uint32_t)(BOOT_FIRST_PAGE + p) << 8;

    boot_page_erase(dst);
    boot_spm_busy_wait();

    for (uint16_t i = 0; i < PAGE_SIZE; i += 2)
      boot_page_fill(dst + i, pgm_read_word_far(src + i));

    boot_page_write(dst);
    boot_spm_busy_wait();
  }

  /* Disarm: an erased page reads 0xFFFF, which is not the magic. */
  boot_page_erase(CTRL_ADDR);
  boot_spm_busy_wait();

  boot_rww_enable();
  RAMPZ = 0;
  SREG = sreg;
  return SVC_OK;
}
