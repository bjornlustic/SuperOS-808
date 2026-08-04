// SuperOS-808 — low-level matrix driver
//
// The TR-808's scan matrix: PH columns buffered by PNP boosters (Q23-Q26,
// active-LOW at the CPU pin), PA/PB rows returned through 15 K pull-downs
// (R76-R82), long harness to the switch board. The read schedule below is
// deliberately clock-independent.
//
// There is no scan-freeze or clamp machinery here. These builds run with USB
// torn down (see main.cpp), so nothing is injecting into the matrix, and
// leaving it out keeps the read path single-branch — which matters on a device
// that cannot be bench-probed.
#pragma once
#include <Arduino.h>
#include <string.h>
#include "pins.h"

namespace hw {


inline void Init() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(SELECT_PINS[i], OUTPUT);
    pinMode(LED_PINS[i], OUTPUT);
    pinMode(SW_PINS[i], INPUT);
    pinMode(STATUS_PINS[i], INPUT);
  }
  for (uint8_t i = 0; i < sizeof(QUIESCENT_LOW_PINS); ++i) {
    pinMode(QUIESCENT_LOW_PINS[i], OUTPUT);
    digitalWrite(QUIESCENT_LOW_PINS[i], LOW);
  }
  // The µPD444C data bus stays hi-Z: the real RAMs are still fitted and drive it.
  for (uint8_t i = 0; i < 4; ++i) pinMode(RAM_BUS_PINS[i], INPUT);
  // PI0 idles LOW. It is the RAM /WE line, so it is pulsed only by ModeProbe().
  pinMode(PI0_PIN, OUTPUT);
  digitalWrite(PI0_PIN, LOW);
  PORTF = 0x0F;            // all selects inactive, all LEDs off
}

// Light exactly one step-LED matrix cell. sel/led are 0..3. Pass led >= 4 for none.
inline void LightCell(uint8_t sel, uint8_t led) {
  uint8_t f = 0x0F;            // selects inactive, LED rows low
  if (led < 4) {
    f &= ~(1 << sel);          // pull this scan column LOW (booster drives it HIGH)
    f |= (1 << (4 + led));     // drive this LED row HIGH
  }
  PORTF = f;
}

inline void AllOff() { PORTF = 0x0F; }

// Matrix read schedule, CLOCK-INDEPENDENT.
//
// Established on hardware: at 4 MHz the per-pin digitalRead calls each took
// ~12 us, so reads landed at +3/+15/+27/+39 us (PB) and +51/+63/+75/+87 us (PA)
// after a select change, and the matrix behaved. At 16 MHz the same source reads
// 4x earlier, before the high-impedance lines settle through the 15 K
// pull-downs: pressed keys ghost into every row and the PA status lines read
// phantom levels, which shows up as the sequencer auto-running at power-on.
// Encoding the working schedule as explicit delays makes the scan identical at
// any F_CPU. Do not shrink these without testing on hardware.
static constexpr uint8_t SCAN_SETTLE_US   = 3;   // select change -> first read
static constexpr uint8_t SCAN_READ_GAP_US = 12;  // spacing between pin reads

// Read the whole switch matrix.
//   cell[s] (s = select 0..3): bit0-3 = PB0-3, bit4-7 = PA0-3 while select s is active
//   *status                  : bit0-3 = PA0-3 with all selects inactive (STATUS gate)
inline void ScanMatrix(uint8_t cell[4], uint8_t *status) {
  PORTF = 0x0F;                       // all selects high -> status group valid
  delayMicroseconds(SCAN_SETTLE_US);
  {
    uint8_t st = 0;
    for (uint8_t j = 0; j < 4; ++j) {
      st |= (uint8_t)(digitalRead(STATUS_PINS[j]) << j);
      delayMicroseconds(SCAN_READ_GAP_US);
    }
    *status = st;
  }

  for (uint8_t s = 0; s < 4; ++s) {
    PORTF = (uint8_t)(0x0F & ~(1 << s));   // engage select s
    delayMicroseconds(SCAN_SETTLE_US);
    uint8_t v = 0;
    for (uint8_t j = 0; j < 4; ++j) {
      v |= (uint8_t)(digitalRead(SW_PINS[j]) << j);
      delayMicroseconds(SCAN_READ_GAP_US);
    }
    for (uint8_t j = 0; j < 4; ++j) {
      v |= (uint8_t)(digitalRead(STATUS_PINS[j]) << (4 + j));
      delayMicroseconds(SCAN_READ_GAP_US);
    }
    cell[s] = v;
  }
  PORTF = 0x0F;
}

// One combined scan + display pass (~1 ms): reads the matrix and status like
// ScanMatrix while also multiplexing a 16-bit step-LED frame (bit n = LED n) one
// column at a time. Each column gets `col_us` of LED dwell, so the frame runs at
// ~25% duty. This is the main loop's heartbeat: calling it continuously keeps the
// LEDs lit and the switches sampled at ~1 kHz.
//
// The short default dwell keeps the loop fast, so the gated tempo clock on the
// PA status line is sampled often enough that a COMMON-TRIG disturbance from a
// stack of voices cannot swallow a tick. Cost is dimmer LEDs.
inline void ScanAndDisplay(uint16_t frame, uint8_t cell[4], uint8_t *status,
                           uint16_t col_us = 25) {
  PORTF = 0x0F;
  delayMicroseconds(SCAN_SETTLE_US);
  {
    uint8_t st = 0;
    for (uint8_t j = 0; j < 4; ++j) {
      st |= (uint8_t)(digitalRead(STATUS_PINS[j]) << j);
      delayMicroseconds(SCAN_READ_GAP_US);
    }
    *status = st;
  }

  for (uint8_t s = 0; s < 4; ++s) {
    const uint8_t leds = (uint8_t)((frame >> (4 * s)) & 0x0F);
    // Anti-ghost column entry — the stock D650C's own drive discipline,
    // traced out of the emulator core and confirmed on hardware. Going
    // straight from one active column to the next in a single PORTF write makes
    // every lit LED glow faintly at the same row of both adjacent columns
    // (steps n+/-4). Passing through an all-selects-parked state and writing the
    // rows BEFORE engaging the select eliminates it. The switch-read schedule
    // is untouched: reads still start SCAN_SETTLE_US after the select engages.
    //
    // The engaged-column span runs with interrupts held off so no ISR can land
    // mid-column and stretch its dwell (which reads as randomly brighter
    // columns = ghosting). ~160 us per column is well inside every ISR's latency
    // budget here: a UART byte time is 320 us and Timer0 ticks at 1.024 ms.
    const uint8_t col_sreg = SREG;
    cli();
    PORTF = (uint8_t)((PORTF & 0xF0) | 0x0F);            // park selects, rows unchanged
    delayMicroseconds(10);
    PORTF = (uint8_t)((leds << 4) | 0x0F);               // new rows while parked
    delayMicroseconds(10);
    PORTF = (uint8_t)((0x0F & ~(1 << s)) | (leds << 4)); // engage select
    delayMicroseconds(SCAN_SETTLE_US);
    uint8_t v = 0;
    for (uint8_t j = 0; j < 4; ++j) {
      v |= (uint8_t)(digitalRead(SW_PINS[j]) << j);
      delayMicroseconds(SCAN_READ_GAP_US);
    }
    for (uint8_t j = 0; j < 4; ++j) {
      v |= (uint8_t)(digitalRead(STATUS_PINS[j]) << (4 + j));
      delayMicroseconds(SCAN_READ_GAP_US);
    }
    cell[s] = v;
    delayMicroseconds(col_us);                           // LED dwell
    PORTF = 0x0F;        // park selects AND clear rows in ONE write before ISRs
    SREG = col_sreg;     // resume: a late ISR then lands on a fully dark matrix
  }
  PORTF = 0x0F;
}

// SW1b mode probe. Drive PI0 (socket pin 30) HIGH and read what arrives on the
// PC bus: with the board mod in place (R85 jumpered, IC7 pins 10-11 jumpered)
// the path is pin 30 -> IC7 /WE -> IC7 I/O4 -> pin 2, gated by SW1b. So the
// returned nibble is a second, independent view of the MODE knob.
//
// Returns bit0 = PC0 ... bit3 = PC3. Returns 0 on an unmodded board, where PI0
// is not even wired to the AVR.
//
// Safe with the RAMs fitted: /WE alone does nothing while /CE (PI1) is idle —
// a µPD444C only writes when it is selected. The pulse is kept short anyway.
inline uint8_t ModeProbe() {
  digitalWrite(PI0_PIN, HIGH);
  delayMicroseconds(20);              // 3.3k/15k divider + bus capacitance
  uint8_t v = 0;
  for (uint8_t i = 0; i < 4; ++i) v |= (uint8_t)(digitalRead(RAM_BUS_PINS[i]) << i);
  digitalWrite(PI0_PIN, LOW);
  return v;
}

} // namespace hw
