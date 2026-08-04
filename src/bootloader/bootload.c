/*
 * SuperOS-808 — minimal serial-MIDI SysEx bootloader
 * Target: AT90USB1286 @ 16 MHz, resident at 0x1F000
 *
 * Same protocol as the other SuperOS bootloaders, so the same host tooling and
 * the same .syx update files work unchanged:
 *   F0 7D 01 <page_hi7> <page_lo7> <len_hi7> <len_lo7> <ck_hi4> <ck_lo4>
 *      <7-bit packed page data> F7      program one 256-byte flash page
 *   F0 7D 02 F7                         jump to the application
 *
 * Entry is STEP 1 held at power-on. The PA status group is deliberately not
 * used: it carries TEMPO CLOCK / START-STOP / TAP / FILL IN and the bit order is
 * not confirmed from the schematic, so a clock line sitting high at boot would
 * trap the machine in the bootloader. The step-switch matrix is unambiguous, so
 * we select column PH0 and look at socket PB0 (= step 1).
 *
 * Progress and status blink step LEDs 1-4.
 */

#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#define APP_ADDRESS 0x0000
#define PAGE_SIZE SPM_PAGESIZE
#define SYSEX_MAX (PAGE_SIZE * 2)

static uint8_t page_buffer[PAGE_SIZE];
static uint8_t sysex_buf[SYSEX_MAX];

static volatile uint16_t sysex_index = 0;
static volatile uint8_t in_sysex = 0;

static void uart_init(void) {
  /* 31250 baud @ 16 MHz -> UBRR = 31 */
  UBRR1H = 0;
  UBRR1L = 31;

  UCSR1A = 0;
  UCSR1B = (1 << RXEN1);                  /* RX only */
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); /* 8N1 */
}

static uint8_t uart_rx(void) {
  while (!(UCSR1A & (1 << RXC1)))
    ;
  return UDR1;
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

  /* If application not blank */
  if (*(uint16_t *)APP_ADDRESS != 0xFFFF) {
    ((void (*)(void))APP_ADDRESS)();
  }

  /* Otherwise fall back into the updater rather than hanging. */
}

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
    if (cksum) {
      /* bad checksum: blink step LEDs 1-4 together forever */
      while (1) {
        PORTF ^= 0xF0;
        _delay_ms(200);
      }
    }
    flash_write_page(page);
  } else if (cmd == 0x02) {
    jump_to_app();
  }
}

static void midi_task(void) {
  uint8_t b = uart_rx();

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
  MCUSR = 0;     /* clear WDRF so a watchdog reboot can't pin WDE on */
  wdt_disable();
  DDRF = 0xFF;   /* PH selects + PG LED rows are outputs */
  DDRB = 0x00;   /* PA status (PB0-3) + PB step rows (PB4-7) are inputs */
  uart_init();

  /* Engage scan column PH0 (active LOW at the CPU pin; Q23 inverts it into a
     HIGH on the panel rail) and let the harness settle. Step 1 then reads back
     on socket PB0 = AVR PB4. The 15 K panel pull-downs make an unpressed key
     read low, so this is a clean test with no pull-up of our own. */
  PORTF = 0x0F;
  PORTF = 0x0E;
  _delay_ms(40);
  if (PINB & (1 << 4)) { /* hold STEP 1 at power-on to stay in the bootloader */
    /* indicate bootloader mode on the 808's own step LEDs: blink steps 1-4
       twice, then hold step 1 solid while waiting for SysEx */
    for (uint8_t i = 0; i < 4; ++i) {
      PORTF = (i & 1) ? 0x0E : 0xFE;
      _delay_ms(150);
    }
    PORTF = 0x1E;  /* step 1 solid = bootloader resident */

    while (1) {
      midi_task();
    }
  }

  PORTF = 0x0F;    /* park the matrix before handing over */
  jump_to_app();
}
