// flash_store.h -- app-side access to the boot-section SPM flash service.
//
// Phase 0 primitives for flash-as-EEPROM. The routine that actually writes
// flash lives in the boot section (src/flash_service, byte 0x1FE00); this is
// just the caller plus arena bounds.
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>

// Byte address of the SPM service entry trampoline (pinned by the linker).
static constexpr uint32_t FLASH_SERVICE_ENTRY = 0x1FE00UL;

// Two more entries follow it, 4 bytes apart (see flash_service.c): staging a
// replacement bootloader, and committing the staged image onto the resident
// bootloader's pages. Boards carrying the older single-entry service have 0xFFFF
// there, which flash_boot_service_present() detects.
static constexpr uint32_t FLASH_SERVICE_STAGE  = 0x1FE04UL;
static constexpr uint32_t FLASH_SERVICE_COMMIT = 0x1FE08UL;

// Key the two privileged entries demand. Not security (the app is trusted
// code): it stops a wild ICALL from landing on the one routine that can unmake
// the bootloader. Must match SVC_KEY in flash_service.c.
static constexpr uint16_t FLASH_SERVICE_KEY = 0x9E37;

// avr:51 function pointers are WORD addresses: byte 0x1FE00 -> word 0xFF00.
typedef uint8_t (*FlashServiceFn)(uint16_t page, const uint8_t *buf);
typedef uint8_t (*FlashStageFn)(uint16_t page, const uint8_t *buf, uint16_t key);
typedef uint8_t (*FlashCommitFn)(uint16_t key);
static const FlashServiceFn kFlashService = (FlashServiceFn)(FLASH_SERVICE_ENTRY >> 1);
static const FlashStageFn   kFlashStage   = (FlashStageFn)(FLASH_SERVICE_STAGE >> 1);
static const FlashCommitFn  kFlashCommit  = (FlashCommitFn)(FLASH_SERVICE_COMMIT >> 1);

static constexpr uint32_t FLASH_PAGE_SIZE  = 256;
static constexpr uint32_t FLASH_ARENA_BASE = 0x17E00UL;  // first arena byte (24.5 KB arena)
static constexpr uint32_t FLASH_ARENA_END  = 0x1E000UL;  // one past the last arena byte
static constexpr uint16_t FLASH_ARENA_FIRST_PAGE = FLASH_ARENA_BASE / FLASH_PAGE_SIZE;        // 0x17E
static constexpr uint16_t FLASH_ARENA_LAST_PAGE  = (FLASH_ARENA_END / FLASH_PAGE_SIZE) - 1;   // 0x1DF

// Status codes returned by flash_write_page (0 = ok).
static constexpr uint8_t FLASH_OK            = 0;
static constexpr uint8_t FLASH_ERR_REJECTED  = 1;     // service refused the page (out of arena)
static constexpr uint8_t FLASH_ERR_NO_SERVICE = 0xFE; // SPM service not installed

// True if the SPM service trampoline is installed at the entry address. The
// entry is a JMP whose first opcode word is 0x940C (high address bits are zero).
inline bool flash_service_present() {
  return pgm_read_word_far(FLASH_SERVICE_ENTRY) == 0x940C;
}

// Erase + write one 256-byte page from a RAM buffer. Returns FLASH_OK on
// success. Halts the CPU for a few ms during the write, so call only when the
// clock is stopped (same discipline as the current EEPROM writes). Never calls
// into the boot section unless the service is actually present.
inline uint8_t flash_write_page(uint16_t page, const uint8_t *buf) {
  if (!flash_service_present()) return FLASH_ERR_NO_SERVICE;
  return kFlashService(page, buf);
}

// Read one byte from the arena (far address, >64 KB).
inline uint8_t flash_read(uint32_t addr) {
  return pgm_read_byte_far(addr);
}

// True if the installed service is the three-entry build, i.e. one that can
// replace the bootloader. Same test as above applied to both extra
// trampolines: each is a JMP, whose first opcode word is 0x940C.
inline bool flash_boot_service_present() {
  return pgm_read_word_far(FLASH_SERVICE_STAGE)  == 0x940C &&
         pgm_read_word_far(FLASH_SERVICE_COMMIT) == 0x940C;
}

// Write one page of a replacement bootloader into the staging window. The
// service refuses any page outside it, so this cannot reach live code.
inline uint8_t flash_stage_page(uint16_t page, const uint8_t *buf) {
  if (!flash_boot_service_present()) return FLASH_ERR_NO_SERVICE;
  return kFlashStage(page, buf, FLASH_SERVICE_KEY);
}

// Copy the staged image onto the resident bootloader. Verify first: this does
// not come back if it goes wrong. See boot_update.h.
inline uint8_t flash_commit_boot() {
  if (!flash_boot_service_present()) return FLASH_ERR_NO_SERVICE;
  return kFlashCommit(FLASH_SERVICE_KEY);
}
