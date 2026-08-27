/*
 * SuperOS — minimal MIDI SysEx bootloader (DIN serial + USB-MIDI)
 * Target: AT90USB1286 @ 16 MHz, resident at 0x1F000
 *
 * ONE BINARY FOR THREE MACHINES. The TR-808, TR-606 and TB-303 firmwares all
 * run on an AT90USB1286 at 16 MHz with BOOTRST programmed and BOOTSZ=01, so the
 * boot section lands at the same address with the same size on all three, and
 * all three already speak the identical update protocol below. The only thing
 * that ever differed was the panel: which button holds the bootloader, and
 * which pins show progress. Both chords are probed here (see main), and the
 * only port driven is PORTF, which is the select/LED matrix on all three. PORTD
 * is deliberately never driven: it is the 303's indicator LEDs but the 606's
 * RAM bus.
 *
 * Same protocol as the other SuperOS bootloaders, so the same host tooling and
 * the same .syx update files work unchanged:
 *   F0 7D 01 <page_hi7> <page_lo7> <len_hi7> <len_lo7> <ck_hi4> <ck_lo4>
 *      <7-bit packed page data> F7      program one 256-byte flash page
 *   F0 7D 02 F7                         jump to the application
 *
 * The update bytes are accepted from either source: USART1 (DIN MIDI IN) or
 * the polled USB-MIDI device in usb_boot.c, so the same .syx flashes over
 * USB-C or the DIN jack.
 *
 * Entry is the machine's boot chord held at power-on (STEP 1 on the 808,
 * WRITE/NEXT on the 303 and 606). The PA status group is deliberately not
 * used: it carries TEMPO CLOCK / START-STOP / TAP / FILL IN and the bit order is
 * not confirmed from the schematic, so a clock line sitting high at boot would
 * trap the machine in the bootloader. The step-switch matrix is unambiguous, so
 * we select column PH0 and look at socket PB0 (= step 1).
 *
 * Additionally the app can jump here without a reset after parking BOOT_MAGIC
 * in GPIOR0 (SysEx 0x4A), which is what makes updates possible over USB-C
 * alone — no panel access needed.
 *
 * Progress and status blink step LEDs 1-4.
 */

#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#include "usb_boot.h"

#define APP_ADDRESS 0x0000
#define PAGE_SIZE SPM_PAGESIZE
#define SYSEX_MAX (PAGE_SIZE * 2)

/* App-requested bootloader entry: survives a jmp (no reset), and GPIOR0
 * resets to 0 on any real reset, so it cannot false-positive at power-on.
 * Keep in sync with the app's 0x4A handler (midi.cpp / main.cpp). */
#define BOOT_MAGIC 0xB7

/* ------------------------------------------------------------------------
 * Page 0x1F0: the permanent page.
 *
 * Everything in this page is reached by a reset and must therefore keep
 * working when the REST of the bootloader is half-written garbage. Nothing
 * here is replaceable over USB or DIN: an over-the-wire update rewrites the
 * body (0x1F1..0x1F8) and never touches this page, so a commit interrupted by
 * a power cut still lands on an intact entry point that can finish the job.
 * Changing anything in this page therefore costs an ISP flash. Keep it small
 * and keep it dull.
 * ------------------------------------------------------------------------ */

/* The flash service's commit entry (flash_service.c) and the key it wants.
 * Kept as bare constants rather than a shared header because this page must
 * not depend on anything that lives in the replaceable body. */
#define SVC_COMMIT_ENTRY 0x1FE08UL
#define SVC_KEY          0x9E37u
#define SVC_JMP_OPCODE   0x940C  /* first word of the trampoline's JMP */

/* Arm flag for a bootloader commit, one flash page below the staging window.
 * The flash service writes it before it starts copying and erases it when the
 * copy is complete, so finding it set at reset means a commit was cut short. */
#define BOOTUPD_CTRL_ADDR 0xD700UL
#define BOOTUPD_CTRL_MAGIC 0xA53C

/* Resume an interrupted bootloader commit. Pinned into the permanent page by
 * its own linker section, so it is present and correct no matter what state
 * the body pages are in.
 *
 * The copy itself lives in the flash service (page 0x1FE), which an
 * over-the-wire update never touches either, so this only has to decide
 * whether to call it. Calling it again when the previous attempt got further
 * than we can tell is harmless: it re-copies the same verified bytes and
 * erases the flag again, so recovery is idempotent however many times it is
 * interrupted.
 *
 * Doing nothing is always safe here. If the flag is clear the boot section is
 * whole, and if the service is missing there is nothing that could help. */
__attribute__((section(".boot_recover"), used, noinline))
void boot_recover(void) {
  /* The indirect call below is a plain ICALL through a 16-bit word address.
     This part has exactly 64K words of flash, so Z alone reaches all of it and
     there is no EIND to set up (which -nostartfiles could not have done). */
  if (pgm_read_word_far(BOOTUPD_CTRL_ADDR) != BOOTUPD_CTRL_MAGIC)
    return;                                   /* no commit was in progress */
  if (pgm_read_word_far(SVC_COMMIT_ENTRY) != SVC_JMP_OPCODE)
    return;                                   /* no flash service installed */

  ((uint8_t (*)(uint16_t))(SVC_COMMIT_ENTRY >> 1))(SVC_KEY);
  RAMPZ = 0;
}

/* Hardware enters the boot section at 0x1F000 (BOOTRST, BOOTSZ=01). This
 * stub is pinned there by the linker (.boot_entry); it establishes the C
 * environment the app or reset left behind, finishes any interrupted
 * bootloader commit, and enters the body. Built with -nostartfiles, so no crt
 * does this for us.
 *
 * It jumps to the CONSTANT 0x1F100, never to main directly. Everything in this
 * page must be independent of where the body's code happens to land, because
 * the body is replaced over the wire and this page is not: a `jmp main` here
 * would be re-encoded every time main moved, and an update would then pair a
 * new body with a stub still jumping to the old address. 0x1F100 is the body's
 * first byte and holds nothing but a jump to main. */
__attribute__((naked, used, section(".boot_entry")))
void boot_entry(void) {
  asm volatile(
    "eor  r1, r1            \n\t"
    "ldi  r28, %0           \n\t"
    "ldi  r29, %1           \n\t"
    "out  __SP_L__, r28     \n\t"
    "out  __SP_H__, r29     \n\t"
    "call boot_recover      \n\t"
    "jmp  0x1F100           \n\t"
    :: "M" (RAMEND & 0xFF), "M" (RAMEND >> 8));
}

/* ------------------------------------------------------------------------
 * Page 0x1F1 onward: the replaceable body.
 * ------------------------------------------------------------------------ */

/* The body's fixed doorway, pinned at 0x1F100. The permanent stub above jumps
 * here blind, so this must be the body's first byte and must stay a single
 * jump. It travels WITH the body, so main is free to move. */
int main(void);
__attribute__((naked, used, section(".body_entry")))
void body_entry(void) {
  asm volatile("jmp main");
}

static uint8_t page_buffer[PAGE_SIZE];
static uint8_t sysex_buf[SYSEX_MAX];

static volatile uint16_t sysex_index = 0;
static volatile uint8_t in_sysex = 0;

static void uart_init(void) {
  /* Pull up RXD1 (PD2, the MIDI IN line) before enabling the receiver. The app
     does the same in setup() (pinMode(MIDI_IN_PIN, INPUT_PULLUP)) and the
     bootloader used not to, which is why DIN updates never worked on this
     board: with nothing holding the line at the idle-high level, the receiver
     sees a permanent break instead of MIDI and no page ever arrives. The app
     could still be talked to over DIN, so the fault looked like "the bootloader
     is deaf" rather than a missing pull-up. */
  DDRD  &= (uint8_t)~(1 << 2);
  PORTD |= (1 << 2);

  /* 31250 baud, derived from F_CPU rather than hardcoded: at 16 MHz this is
     the same 31 as before, but a build targeting a divided core clock (see
     env:app-4mhz) would otherwise sit at 4x the intended baud rate and receive
     nothing. U2X is off, so UBRR = F_CPU/(16*baud) - 1. */
  const uint16_t ubrr = (uint16_t)((F_CPU / (16UL * 31250UL)) - 1UL);
  UBRR1H = (uint8_t)(ubrr >> 8);
  UBRR1L = (uint8_t)(ubrr & 0xFF);

  UCSR1A = 0;
  UCSR1B = (1 << RXEN1);                  /* RX only */
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); /* 8N1 */
}

/* returns checksum of decoded bytes */
static uint8_t decode_7bit(uint8_t *in, uint16_t len, uint8_t *out) {
  uint16_t i = 0, o = 0;
  uint8_t check = 0;

  while (i < len) {
    uint8_t msb = in[i++];

    for (uint8_t b = 0; b < 7 && i < len; b++) {
      uint8_t v = in[i++];
      out[o] = v | (((msb >> b) & 1) << 7);
      check ^= out[o++];
      if (o >= PAGE_SIZE)
        return check;
    }
  }
  return check;
}

static void flash_write_page(uint16_t page) {
  uint32_t addr = (uint32_t)page * PAGE_SIZE;

  uint8_t sreg = SREG;
  cli();

  eeprom_busy_wait();

  boot_page_erase(addr);
  boot_spm_busy_wait();

  for (uint16_t i = 0; i < PAGE_SIZE; i += 2) {
    uint16_t w = (uint16_t)(page_buffer[i]) | ((uint16_t)(page_buffer[i + 1]) << 8);
    boot_page_fill(addr + i, w);
  }

  boot_page_write(addr);
  boot_spm_busy_wait();
  boot_rww_enable();

  SREG = sreg;
}

static void jump_to_app(void) {
  cli();

  /* If application not blank (flash read: a plain *(uint16_t*)0 would read
     the r0/r1 register file, not flash). Returns to the caller when blank. */
  if (pgm_read_word(APP_ADDRESS) != 0xFFFF) {
    usb_boot_detach(); /* no-op unless USB was brought up */
    ((void (*)(void))APP_ADDRESS)();
  }

  /* Otherwise fall back into the updater rather than hanging. */
}

#ifdef SUPEROS_USB_DIAG
/* Publish raw USB state into EEPROM 0x100.. so it can be read back over ISP.
   -nostdlib means no avr-libc eeprom_*, so the write is done by hand. Writes
   happen only when a byte actually changes, so a settled state costs nothing
   and the cell endurance is never a concern during a bring-up session. */
static void ee_write(uint16_t addr, uint8_t val) {
  while (EECR & (1 << EEPE))
    ;
  EEAR = addr;
  EEDR = val;
  uint8_t s = SREG;
  cli();
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
  SREG = s;
}

/* UART/update-path counters, published alongside the USB registers. All are
   saturating so a byte can represent "lots" without wrapping to a small
   number and reading as "a few". */
static uint8_t d_bytes, d_f0, d_f7, d_cmd1, d_pages, d_cksum, d_ucsra;
static void bump(uint8_t *p) { if (*p != 0xFE) (*p)++; }

#define DIAG_BASE 0x100
static uint8_t diag_last[16];
static uint8_t diag_primed;   /* cleared in main(): .bss is not zeroed here */
static void diag_publish(void) {
  uint8_t v[16];
  v[0] = 0x5A;               /* magic: the diagnostic loop ran at all */
  v[1] = USBSTA;             /* bit0 VBUS, bit1 ID, bit2 VBUSTI */
  v[2] = USBCON;             /* bit7 USBE, bit5 FRZCLK, bit4 OTGPADE */
  v[3] = UDCON;              /* bit0 DETACH */
  v[4] = PLLCSR;             /* bit0 PLOCK, bit1 PLLE */
  v[5] = usb_boot_flags;     /* bit0 saw bus reset, bit1 configured */
  v[6] = UDINT;              /* bit3 EORSTI, bit2 SOFI */
  v[7] = UDADDR;             /* bit7 ADDEN + assigned address */
  v[8]  = d_bytes;           /* UART bytes seen at all */
  v[9]  = d_f0;              /* SysEx starts */
  v[10] = d_f7;              /* SysEx ends */
  v[11] = d_cmd1;            /* 0x01 program-page commands parsed */
  v[12] = d_pages;           /* pages actually written */
  v[13] = d_cksum;           /* last checksum residual (0 = matched) */
  v[14] = d_ucsra;           /* UCSR1A: bit4 FE1 framing, bit3 DOR1 overrun */
  v[15] = 0xA5;              /* end marker */
  /* First pass writes all of them unconditionally. Without that, an unwritten
     cell reads back as the 0xFF erase value and is indistinguishable from a
     real reading, which is exactly how the first run reported a phantom
     bus-reset and a phantom USB address. */
  for (uint8_t i = 0; i < 16; i++) {
    if (!diag_primed || v[i] != diag_last[i]) {
      ee_write(DIAG_BASE + i, v[i]);
      diag_last[i] = v[i];
    }
  }
  diag_primed = 1;
}
#endif

static uint8_t count = 0;
static void process_sysex(uint8_t *data, uint16_t len) {
  if (len < 2)
    return;

  if (data[0] != 0x7D)
    return;

  uint8_t cmd = data[1];

  if (cmd == 0x01 && len > 9) {
    uint16_t page = ((uint16_t)data[2] << 7) | data[3];
    uint16_t packed_len = ((uint16_t)data[4] << 7) | data[5];
    uint8_t cksum = (data[6] << 4) | data[7];

    if (packed_len + 8 > len)
      return;

    /* progress: cycle step LEDs 1-4 (PORTF: select 0 active, one PG row high) */
    PORTF = 0x0E | (uint8_t)(0x10 << (count++ & 3));

    /* if they match, cksum becomes zero */
    cksum ^= decode_7bit(&data[8], packed_len, page_buffer);
#ifdef SUPEROS_USB_DIAG
    bump(&d_cmd1);
    d_cksum = cksum;
    if (cksum) diag_publish();   /* record before the lock-up below */
#endif
    if (cksum) {
      /* bad checksum: blink step LEDs 1-4 together forever */
      while (1) {
        PORTF ^= 0xF0;
        _delay_ms(200);
      }
    }
    flash_write_page(page);
#ifdef SUPEROS_USB_DIAG
    bump(&d_pages);
#endif
  } else if (cmd == 0x02) {
    jump_to_app();
  }
}

/* One byte of MIDI from either transport (USART1 or USB); non-static so
   usb_boot.c can feed it. */
void midi_byte(uint8_t b) {
#ifdef SUPEROS_USB_DIAG
  bump(&d_bytes);
  if (b == 0xF0) bump(&d_f0);
  if (b == 0xF7) bump(&d_f7);
#endif
  if (b == 0xF0) {
    in_sysex = 1;
    sysex_index = 0;
    return;
  }

  if (!in_sysex)
    return;

  if (b == 0xF7) {
    process_sysex(sysex_buf, sysex_index);
    in_sysex = 0;
    return;
  }

  if (b < 0x80 && sysex_index < SYSEX_MAX)
    sysex_buf[sysex_index++] = b;
}

int main(void) {
  cli();

  /* After a watchdog reset, WDRF forces WDE on and wdt_disable() alone cannot
     clear it: the chip reset-loops every 15 ms until power-off. Clear MCUSR
     first. Stash the reset cause in GPIOR1 first so the app can tell a
     watchdog recovery from a normal power-on. */
  GPIOR1 = MCUSR;
  MCUSR = 0;
  wdt_disable();

  /* Run at the undivided crystal rate whatever the entry path left behind.
     CLKPR survives both ways in: an app that divided the core clock jumps here
     with its divider still loaded (SysEx 0x4A from the 4 MHz build leaves
     CLKPS = /4), and a chip whose CKDIV8 fuse is still programmed resets into
     /8. This file's UBRR and every _delay_ms are computed from F_CPU = the
     crystal, so either case would put DIN MIDI at the wrong bit rate and look
     exactly like a deaf bootloader. USB does not care either way: clkUSB comes
     off the PLL, which taps XTAL1 ahead of this prescaler.
     Timed sequence: CLKPCE alone, then the value within four cycles. */
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  /* -nostartfiles: .bss is never zeroed, so parser state is set explicitly
     (this also covers entry by jump from the app, where RAM is app leftovers) */
  sysex_index = 0;
  in_sysex = 0;
  count = 0;
#ifdef SUPEROS_USB_DIAG
  diag_primed = 0;
#endif

  DDRF = 0xFF;   /* PH selects + PG LED rows are outputs */
  DDRB = 0x00;   /* PA status (PB0-3) + PB step rows (PB4-7) are inputs */
  uart_init();

  /* app-requested entry (SysEx 0x4A jumped here with the magic parked) */
  uint8_t stay = 0;
  if (GPIOR0 == BOOT_MAGIC) {
    GPIOR0 = 0;
    stay = 1;
  }

  /* Boot chord. One binary serves all three machines, so both known chords are
     probed and either one holds the bootloader. Skipped entirely when the app
     asked for us, which is the common path and wants no added latency.

     Probe A, the 808: engage scan column PH0 (selects are active LOW at the CPU
     pin; the booster inverts it into a HIGH on the panel rail) and read STEP 1
     back on socket PB0 = AVR PB4. The 15 K panel pull-downs make an unpressed
     key read low, so this is a clean test with no pull-up of our own.

     Probe B, the 303 and 606: no column engaged, which gates the status group
     through, then read WRITE/NEXT on bit 1.

     The two do not collide. With no column engaged an 808 reports its status
     group on bits 0-3, and bit 1 there is TAP, measured on hardware as a ~15 ms
     one-shot: high in 2 samples out of 3907 across a 90 s capture in which it
     was deliberately held for three seconds twice. It cannot sit high at boot,
     so probe B is silent on an 808. (That also rules TAP out as the 808's own
     chord, which is why STEP 1 remains its button. Bit 3 of the same group is
     the free-running tempo clock, high half the time, and would trap the
     machine in the bootloader forever if it were ever read here.)

     UNVERIFIED, and it needs a bench check on each: probe A engages column 0 on
     a 303 or 606 and reads bit 4. Their pull-downs should make that read low
     with nothing held, but it has not been measured on those machines. If one
     of them boots straight into the bootloader with nothing held, this is why. */
  if (!stay) {
    PORTF = 0x0F;
    PORTF = 0x0E;
    _delay_ms(40);
    if (PINB & (1 << 4))
      stay = 1;                  /* 808: STEP 1 */

    if (!stay) {
      PORTF = 0x0F;
      _delay_ms(40);
      if (PINB & (1 << 1))
        stay = 1;                /* 303 / 606: WRITE/NEXT */
    }
  }

  if (!stay) {
    PORTF = 0x0F;    /* park the matrix before handing over */
    jump_to_app();   /* returns only if the app section is blank */
  }

  /* attach early so USB enumeration overlaps the LED show (control
     requests just wait in the FIFO until the loop below services them) */
  usb_boot_init();

  /* indicate bootloader mode on the 808's own step LEDs: blink steps 1-4
     twice, then hold step 1 solid while waiting for SysEx */
  for (uint8_t i = 0; i < 4; ++i) {
    PORTF = (i & 1) ? 0x0E : 0xFE;
    _delay_ms(150);
  }
  PORTF = 0x1E;  /* step 1 solid = bootloader resident */

  while (1) {
    {
      const uint8_t sr = UCSR1A;
#ifdef SUPEROS_USB_DIAG
      d_ucsra |= (uint8_t)(sr & ((1 << FE1) | (1 << DOR1) | (1 << UPE1)));
#endif
      if (sr & (1 << RXC1))
        midi_byte(UDR1);
    }
    usb_boot_task();

#ifdef SUPEROS_USB_DIAG
    /* USB bring-up telemetry on step LEDs 1-4. Needs no host tooling and no
       MIDI return path, which is the point: it reports from the far side of
       whatever is failing.
         step 1  blinking  = bootloader alive, USB loop running
         step 2  on        = VBUS sensed at the AVR pin
         step 3  on        = host drove a bus reset (it SAW us electrically)
         step 4  on        = SET_CONFIGURATION done (enumeration complete)
       step 1 alone means the host never detected an attach: CC pulldowns,
       VBUS, or the D+ pull-up / D+/D- pair. step 3 without step 4 means the
       electrical link is good and enumeration itself is failing. */
    {
      static uint16_t tick = 0;
      uint8_t leds = 0;
      if (++tick & 0x1000)                      leds |= 0x1;
      if (USBSTA & (1 << VBUS))                 leds |= 0x2;
      if (usb_boot_flags & USB_BOOT_SAW_RESET)  leds |= 0x4;
      if (usb_boot_flags & USB_BOOT_CONFIGURED) leds |= 0x8;
      PORTF = (uint8_t)(0x0E | (uint8_t)(leds << 4));
      if (!tick) diag_publish();   /* every 65536 passes: mirror state to EEPROM */
    }
#endif
  }
}
