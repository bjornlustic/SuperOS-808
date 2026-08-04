// SuperOS-808 — TR-808 firmware for the AT90USB1286 CPU replacement
//                (any AT90USB1286 board in the µPD650C socket; Teensy++ 2.0 class)
//
// pins.h — the µPD650C CPU pinout as wired to the AT90USB1286 that replaces it,
//          annotated with TR-808 port semantics.
//
// The socket-pin <-> AVR-pin wiring is a fixed property of the replacement
// board: a 42-pin µPD650C, port-to-pin assignment per the service manual p.3
// ("µPD650C-085 functional description"). See docs/HARDWARE_MAP.md.
//
//   Port  Dir  TR-808 use                                     µPD650 pins
//   ----  ---  ---------------------------------------------  -----------
//   PH0-3 OUT  scan-select to switches + STATUS buffer gate    26-29
//   PG0-3 OUT  drive signals to the 16 STEP LEDs               22-25
//   PA0-3 IN   switch-scan inputs + status                     33-36
//                (status, all PH high: tempo clk, start/stop, tap, fill in)
//   PB0-3 IN   inputs from the 16 STEP switches                37-40
//   PC0-3 I/O  RAM (4x µPD444C) data bus, via R85-R88           2-5
//   PD0-3 OUT  RAM addr A0-A3 / instrument data CH,OH,CY,CB     8-11
//   PE0-3 OUT  RAM addr A8,A9 + chip sel / inst data CP,RS,HT,MT 12-15
//   PF0-3 OUT  RAM addr A4-A7 / instrument data LT,SD,BD,AC     16-19
//   PI0   OUT  RAM WE                                            30
//   PI1   OUT  RAM CE (decoded with PE2/PE3 by IC5)              31
//   PI2   OUT  COMMON TRIG (instrument trigger pulse)            32
//
#pragma once
#include <Arduino.h>

// Teensy++ 2.0 Arduino pin numbers for each µPD650 pin (replacement-board wiring).
enum TppPin : uint8_t {
  MIDI_IN_PIN  = 2,   // Serial1 RX
  MIDI_OUT_PIN = 3,   // Serial1 TX

  // PC0-3 : RAM data bus
  PC0_PIN = 4, PC1_PIN = 5, PC2_PIN = 6, PC3_PIN = 7,

  // PI : RAM CE + common-trigger out
  PI1_PIN = 8,   // RAM CE
  PI2_PIN = 9,   // COMMON TRIG

  // PD0-3 : RAM addr A0-A3, and the CH/OH/CY/CB instrument-data bits
  PD0_PIN = 10,  // CH instrument data   (µPD650 pin  8)
  PD1_PIN = 11,  // OH instrument data   (µPD650 pin  9)
  PD2_PIN = 12,  // CY instrument data   (µPD650 pin 10)
  PD3_PIN = 13,  // CB instrument data   (µPD650 pin 11)

  // PF0-3 : RAM addr A4-A7, and the LT/SD/BD/AC instrument-data bits
  PF0_PIN = 14,  // LT instrument data   (µPD650 pin 16)
  PF1_PIN = 15,  // SD instrument data   (µPD650 pin 17)
  PF2_PIN = 16,  // BD instrument data   (µPD650 pin 18)
  PF3_PIN = 17,  // AC (ACCENT) data     (µPD650 pin 19)

  // PE0-3 : RAM addr A8/A9 + chip select, and the CP/RS/HT/MT data bits
  PE2_PIN = 0,   // HT instrument data   (µPD650 pin 14)
  PE3_PIN = 1,   // MT instrument data   (µPD650 pin 15)
  PE0_PIN = 18,  // CP instrument data   (µPD650 pin 12)
  PE1_PIN = 19,  // RS instrument data   (µPD650 pin 13)

  // PB0-3 : step-switch inputs   (AVR PB4-7)
  PB0_PIN = 24, PB1_PIN = 25, PB2_PIN = 26, PB3_PIN = 27,

  // PA0-3 : switch-scan + status inputs   (AVR PB0-3)
  PA0_PIN = 20, PA1_PIN = 21, PA2_PIN = 22, PA3_PIN = 23,

  // PH0-3 : scan-select outputs  (AVR PORTF low nibble)
  PH0_PIN = 38, PH1_PIN = 39, PH2_PIN = 40, PH3_PIN = 41,

  // PG0-3 : step-LED drive outputs (AVR PORTF high nibble)
  PG0_PIN = 42, PG1_PIN = 43, PG2_PIN = 44, PG3_PIN = 45,
};

// ACCENT is the fourth PF instrument-data bit: µPD650 PF3 = socket pin 19.
// Per the service manual p.4 it feeds the Q31-Q34 accent circuit, which sets the
// AMPLITUDE of the COMMON TRIG pulse (~4 V plain, 4-14 V accented via VR3). So
// accent is one shared level per trigger window, not a per-voice attribute.
static constexpr uint8_t AC_PIN = PF3_PIN;

// PH (select) + PG (LED) are the low + high nibbles of AVR PORTF, so the whole
// LED/scan matrix can be driven with a single byte write to PORTF:
//   low nibble  = PH0-3  (drive a bit LOW to activate that scan column; Q23-Q26
//                         invert it into a HIGH on the panel rail)
//   high nibble = PG0-3  (drive a bit HIGH to light that LED row)
// PORTF = 0x0F -> all selects inactive, all LEDs off.

static const uint8_t SELECT_PINS[4] = { PH0_PIN, PH1_PIN, PH2_PIN, PH3_PIN }; // scan columns
static const uint8_t LED_PINS[4]    = { PG0_PIN, PG1_PIN, PG2_PIN, PG3_PIN }; // step-LED rows
static const uint8_t SW_PINS[4]     = { PB0_PIN, PB1_PIN, PB2_PIN, PB3_PIN }; // step-switch inputs
static const uint8_t STATUS_PINS[4] = { PA0_PIN, PA1_PIN, PA2_PIN, PA3_PIN }; // status/scan inputs

// PC0-3 (socket pins 2-5) are the µPD444C DATA BUS and must NEVER be driven by
// this firmware. The four RAM chips are still fitted and drive that bus back
// during a read, so driving it is a bus fight: the 808 has 3.3k series buffer
// resistors (R85-R88) precisely because the CPU and the RAM would otherwise
// conflict there (service manual p.4). Our RAM is emulated in SRAM, so there is
// nothing to gain by mirroring it onto the pins and real damage to be done.
// They stay INPUTS. Each line has a 15k pull-down to ground (R83/R84/R90/R89),
// so an idle bus reads low without any pull-up of ours.
static const uint8_t RAM_BUS_PINS[4] = { PC0_PIN, PC1_PIN, PC2_PIN, PC3_PIN };

// Socket pin 30 = PI0 = the µPD444C /WE line, gated by SW1b (the MODE switch's
// second deck — the memory-protect deck: "MANUAL PLAY and PLAY are the only two
// Mode positions that will protect your programmed memories", owner's manual
// p.3). Driving it and reading what comes back on the PC bus is therefore an
// INDEPENDENT read of the MODE knob, which is the one thing the SW1a diode
// encoder cannot give us (COMPOSE and PLAY collide there — see
// docs/HARDWARE_MAP.md §4.1).
//
// REQUIRES A MOD WIRE ON MOST BOARDS. CPU replacements generally leave socket
// pin 30 unconnected, so this pin does nothing until J4 pin 30 is wired to a
// spare AVR GPIO. Typically free: PA4/PA5 (ex-I2C) and PA6/PA7/PE4/PE5
// (ex-JTAG). NOT free, however a board's notes describe them: PE7/PD0/PD1/PC7
// are bonded to socket pins 13/14/15/19, which on the TR-808 are the
// RS/HT/MT/AC instrument-data bits.
//
// Default PA5 = Teensy 33. Override with -DSUPEROS_PI0_PIN=<teensy pin>.
#ifndef SUPEROS_PI0_PIN
#define SUPEROS_PI0_PIN 33
#endif
static constexpr uint8_t PI0_PIN = SUPEROS_PI0_PIN;

// Data / trigger lines, held OUTPUT-LOW at rest so the 808's voice-trigger gates
// stay quiescent (COMMON TRIG idle LOW => no voice fires). The RAM data bus is
// deliberately absent — see RAM_BUS_PINS above.
static const uint8_t QUIESCENT_LOW_PINS[] = {
  PD0_PIN, PD1_PIN, PD2_PIN, PD3_PIN,
  PF0_PIN, PF1_PIN, PF2_PIN, PF3_PIN,
  PE0_PIN, PE1_PIN, PE2_PIN, PE3_PIN,
  PI1_PIN, PI2_PIN,
};
