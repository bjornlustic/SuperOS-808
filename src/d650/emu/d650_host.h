// d650_host.h — the emulated TR-808 "machine": µPD650C-085 core + the four
// external µPD444C pattern RAMs + I/O decode onto TR-808 semantics. Portable C
// (no AVR/Arduino deps) so it runs identically on a desktop host and on the
// AT90USB1286.
//
// The host implements the four ucom4 I/O callbacks:
//   read A/B  -> switch/status matrix (from an input snapshot, see below)
//   read C    -> µPD444C data bus (only while a chip is enabled and WE idle)
//   write C..I-> RAM bus decode, then every write is forwarded raw to the
//                platform via drv.port_write. On hardware those latches ARE the
//                physical pin levels, so the AVR bridge just mirrors them and
//                the 808's own gates do the rest.
//
// EXTERNAL RAM MODEL (service manual p.3/p.4)
//   Four µPD444C (1024 x 4) = 4096 nibbles = 2048 bytes packed.
//     address  PD = A0-A3, PF = A4-A7, PE0 = A8, PE1 = A9   (1024 per chip)
//     chip     PE2/PE3 decoded 2-to-4 by IC5, gated by PI1 (/CE)
//     write    PI0 (/WE)
//   Contents, per the manual's Figure 4 memory map:
//     IC7  ACCENT / BASS DRUM / SNARE DRUM / LOW TOM
//     IC8  COW BELL / CYMBAL / OPEN HI HAT / CLOSED HI HAT
//     IC10 MID TOM / HIGH TOM / RIM SHOT / HAND CLAP
//     IC9  RHYTHM TRACK / RHYTHM PRESCALE / RHYTHM STEP
//   The host does not care which is which — the emulated CPU owns that layout.
#pragma once
#include <stdint.h>
#include "ucom4.h"

#ifdef __cplusplus
extern "C" {
#endif

// Input snapshot, same layout as hw::ScanMatrix produces (controls.h):
// 1 = closed/active cell.
//   [row*4 + col]      PB while PH row selected: the 16 STEP switches  idx  0..15
//   [16 + row*4 + col] PA while PH row selected: MODE/CLEAR/PRE SCALE/
//                      BASIC VARIATION/INSTRUMENT/AUTO FILL IN/I-F     idx 16..31
//   [32 + j]           PA with all PH high: the STATUS group           idx 32..35
#define D650_IN_COUNT  40
#define D650_IN_STATUS 32
// Status-group indices. Which physical PA bit each of these sits on is a
// calibrated property of the panel (see controls.h PanelMap); the AVR bridge
// maps the real bits onto these slots before feeding the snapshot in, so the
// emulated CPU always sees the layout the real machine had.
#define D650_IN_RUN    (D650_IN_STATUS + 0)
#define D650_IN_TAP    (D650_IN_STATUS + 1)
#define D650_IN_FILL   (D650_IN_STATUS + 2)
#define D650_IN_CLOCK  (D650_IN_STATUS + 3)

// External µPD444C store: 4096 nibbles packed 2 per byte (even nibble = low 4
// bits). The packed array is the persistence image — snapshot/restore verbatim.
#define D650_EXT_NIBS  4096
#define D650_EXT_BYTES (D650_EXT_NIBS / 2)

// Strobe polarity at the CPU pins.
//
// /WE IS ACTIVE LOW, measured against a real µPD650C-085 on a desktop host (the
// whole point of keeping this file portable). Driving the 808 CPU with a clock
// and a held step key, PI0 read high in 200000 of 200000 samples in every
// normal mode, and went low ONLY in the one state that genuinely writes memory —
// PATTERN CLEAR with the CLEAR button down. A strobe that idles high and pulses
// low is active low.
//
// The old `1` broke the emulator's memory in a way that looks like a dead panel
// rather than a dead RAM: ext_refresh() then
// treats the store as write-transparent for ~96% of the time, splattering the
// stale PORTC latch across every address the CPU sets up during a READ, and
// io_read_c() returns 0 whenever PI0 is high — which is to say always. Every
// pattern read came back empty, so nothing entered on the panel ever appeared on
// the step LEDs or triggered a voice.
#ifndef D650_WE_ACTIVE
#define D650_WE_ACTIVE 0   // PI0 idles HIGH and pulses LOW to write
#endif
#ifndef D650_CE_ACTIVE
#define D650_CE_ACTIVE 1   // PI1 pulses HIGH around any access
#endif

// Driver hook — set by the platform. The host forwards every port write
// (ucom4 port index + 4-bit latch value) after its own RAM decode; the AVR
// bridge mirrors the latches onto the socket pins. Desktop mocks log/inspect.
//
// read_clock (optional, may be NULL): live TEMPO/DIN clock level, sampled at the
// instant the emulated CPU reads its status group. The polled snapshot is too
// coarse for the 1.9 ms-cadence sampling the real µPD650C did; on the AVR the PH
// latches are mirrored to the real select pins, so at that exact moment the
// physical clock line is readable directly (it is actively driven, so it needs
// none of the key lines' settle time). NULL = fall back to the snapshot.
typedef struct {
  void (*port_write)(void *u, int port, uint8_t data);
  void *u;
  uint8_t (*read_clock)(void *u);
} d650_drivers;

typedef struct {
  ucom4_t  cpu;
  uint8_t  ext[D650_EXT_BYTES]; // 4x µPD444C pattern/track store, packed nibbles
  uint8_t  ext_dirty;           // set when a RAM nibble changes (save trigger)
  uint8_t  in[D650_IN_COUNT];
  // Output-latch mirrors (the core updates its port_out AFTER the write callback
  // runs, so the host keeps its own copies).
  uint8_t  lat_c, lat_d, lat_e, lat_f, lat_g, lat_h, lat_i;
  d650_drivers drv;
} d650_host;

// Initialize + reset. `prg` = the 2 KB TR-808 program image. `drv` may be NULL.
void d650_init(d650_host *h, const uint8_t *prg, const d650_drivers *drv);

// Run one CPU instruction. Returns machine cycles consumed.
uint32_t d650_step(d650_host *h);

// Run at least `cycles` machine cycles (batched; cheaper than stepping).
static inline uint32_t d650_run(d650_host *h, uint32_t cycles) {
  return ucom4_run(&h->cpu, cycles);
}

// Drive the /INT pin — the fixed interrupt clock, 1.9 ms period per the service
// manual p.3 Figure 2. NOT the tempo clock; tempo goes on in[D650_IN_CLOCK].
void d650_clock(d650_host *h, int level);

// Snapshot/restore the packed pattern store (persistence layer target).
static inline int  d650_dirty(const d650_host *h) { return h->ext_dirty; }
static inline void d650_clear_dirty(d650_host *h) { h->ext_dirty = 0; }

#ifdef __cplusplus
}
#endif
