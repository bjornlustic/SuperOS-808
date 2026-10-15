// SuperOS-808 — low-level matrix driver
//
// The TR-808's scan matrix: PH columns buffered by PNP boosters (Q23-Q26,
// active-LOW at the CPU pin), PA/PB rows returned through 15 K pull-downs
// (R76-R82), long harness to the switch board. The read schedule below is
// deliberately clock-independent.
//
// There is no scan-freeze or clamp machinery here. Leaving it out keeps the read
// path single-branch, which matters on a device that cannot be bench-probed. The
// 15 K pull-downs above hold the sense lines at a defined level, and the read
// schedule below is timed in microseconds so it is identical at any F_CPU.
//
// NOTE (usb-c branch): the earlier claim that "these builds run with USB torn
// down, so nothing is injecting into the matrix" is no longer true. Both the
// combined path (combined.cpp: usb_vbus_gate before superos_loop) and the app
// path (main.cpp loop) keep the USB engine live during the scan whenever a cable
// is present. Whether that injection reaches these 15 K-pulled lines the way it
// reaches the 606's FLOATING status nets (findings 15-19) is a bench question,
// unmeasured on the 808: see claim-008 item 3.
#pragma once
#include <Arduino.h>
#include <string.h>
#include "pins.h"

// Called between scan columns, ~4x per pass. Defined in main.cpp: the scan
// cannot see the engine, and the engine must not wait a whole pass to release
// the shared indicator lines (claim-128).
void scan_tick();

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
  // PI0 idles LOW. It is the RAM /WE line, so it is pulsed only by WriteEnable().
  pinMode(PI0_PIN, OUTPUT);
  digitalWrite(PI0_PIN, LOW);
  PORTF = 0x0F;            // all selects inactive, all LEDs off
  // Socket pin 18 (BD data, PF2_PIN = AVR PC6) passes U5, a TS5A3159A path
  // selector on boards that also serve the TB-303. AVR PA4 LOW (R15 holds it
  // there while hi-Z) routes PC6 through the 303 cutoff-CV low-pass, which
  // smears the BD edge so the bass drum never fires. HIGH is the raw path. PA4 is
  // otherwise unused by this firmware. Every boot path runs Init, so both
  // firmwares get it.
  PORTA |= _BV(4);
  DDRA  |= _BV(4);
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
// 4x earlier and the matrix does not: pressed keys ghost into every row and the
// PA status lines read phantom levels, which shows up as the sequencer
// auto-running at power-on. Encoding the working schedule as explicit delays
// makes the scan identical at any F_CPU. Do not shrink these without testing on
// hardware.
//
// WHAT IS ACTUALLY SLOW, because this comment used to say the wrong thing.
// It said the reads landed "before the lines settle through the 15 K
// pull-downs", which points at the rising edge. Simulated in
// tools/spice/808_pa_sense.cir (see docs/gates/G13_808_SENSE_PATH.md), a driven
// line reaches V_IH in ~13 ns: the rise is limited by the column driver, not by
// the pull-down. The 15 K governs the FALL. Once a column deselects, its
// encoder diode reverse-biases and the resistor is the sense net's only path to
// ground, so the net takes ~2.2 us (100 pF harness) to fall below V_IL.
//
// That is why the symptom is a key ghosting into rows it is NOT in: what the
// early read sees is the PREVIOUS column's charge still on the line. A
// rise-limited fault could not produce that. SCAN_SETTLE_US below clears the
// simulated worst case (2.98 us at the slow end of the harness-capacitance
// range the hardware measurement bounds) with the 12 us inter-read gaps behind
// it, so the schedule is right; only the explanation was wrong.
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
    // claim-128: the scan is most of the loop pass, so this is the only place
    // fine enough to end a trigger's data window ON TIME. Between columns, with
    // interrupts already restored, and it touches no PORTF bit -- the lines it
    // may write are PORTD/PORTE -- so the matrix is undisturbed.
    scan_tick();
  }
  PORTF = 0x0F;
}

// SW1b, the MODE switch's memory-protect deck, read through an optional board
// mod: R85 bridged, PC0's 15k pull-down (R89) removed, and a RAM's pin 10
// (/WE) linked to its pin 11 (I/O, PC0). SW1b joins /WE to PI0 in the write
// positions (COMPOSE, 1ST/2ND PART, PATTERN CLEAR) and leaves it on R91 (1M to
// BU) in PLAY and MANUAL PLAY. So with PI0 HIGH the node reads 1 in every
// position (driven, or pulled up), and with PI0 LOW it reads 0 only where SW1b
// joins it: 1 = write enabled, 0 = protected. A stock board keeps R89, so PC0
// reads 0 with PI0 HIGH: -1, no mod, nothing to tell PLAY from COMPOSE.
// (Measured 2026-09-28: PI0 HIGH read 1 in PLAY, COMPOSE and MANUAL PLAY.)
// Safe with the RAMs fitted: /CE (PI1) is idle, so /WE alone writes nothing.
inline int8_t WriteEnable() {
  digitalWrite(PI0_PIN, HIGH);
  delayMicroseconds(20);
  const bool hi = digitalRead(RAM_BUS_PINS[0]);
  digitalWrite(PI0_PIN, LOW);
  delayMicroseconds(20);
  const bool lo = digitalRead(RAM_BUS_PINS[0]);
  if (!hi) return -1;
  return lo ? 0 : 1;
}

} // namespace hw
