// emu_avr.cpp -- AVR bridge for the µPD650C-085 emulator (combined builds).
//
// Runs the emulated CPU against the real TR-808 hardware. Its port
// latches ARE the socket pins, so the bridge is mostly a mirror:
//   PH -> the four scan selects        PG -> the step-LED rows
//   PD/PE/PF -> RAM address + instrument data      PI2 -> COMMON TRIG
// and the panel is read back through the same matrix scan SuperOS uses, then
// handed to the host as an input snapshot.
//
// Timing. The stock machine feeds /INT every 1.9 ms (service manual p.3 Fig. 2)
// and does all of its panel scanning inside that interrupt. We drive the same
// 1.9 ms /INT and let the interpreter run free between edges, so the machine's
// own cadence — including the ~1.8 ms step-LED dwell — comes out of the emulation
// rather than being reproduced by hand.
//
// The tempo clock is NOT taken from the polled snapshot: the CPU samples it at
// its own moments and a ~4 ms poll would alias. d650_drivers::read_clock reads
// the physical status line at the instant the CPU asks (the line is actively
// driven, so it needs none of the key lines' settle time).
#ifdef SUPEROS_COMBINED

#include <Arduino.h>
#include <avr/wdt.h>
#include <string.h>
#include <util/delay.h>

#include "../pins.h"
#include "../hw.h"
#include "../controls.h"
#include "../combined.h"
#include "../lamp_probe.h"
#include "../settings.h"
#include "emu/d650_host.h"
#ifdef UCOM4_NATIVE
extern "C" uint16_t ucom4_nat_calls;      // bench telemetry (ucom4.c)
extern "C" uint16_t ucom4_nat_cycles;
extern "C" uint16_t ucom4_idle_hits;
extern "C" uint16_t ucom4_isr_count;
extern "C" uint16_t ucom4_io_calls;
extern "C" uint16_t ucom4_int_cycles;
extern "C" uint8_t  ucom4_nat_state(void);
#endif
#include "image_store.h"
#ifdef D650_IMG_EMBEDDED
// The 1981 program image is 2 KB of PURE DATA and it was sitting in the app
// region, under the 0xD700 ceiling, competing with code for the scarcest
// resource on the machine (claim-114). It is read only through
// UCOM4_PRG_RD -> pgm_read_byte, a 16-bit LPM, so it may live anywhere below
// 0x10000; .d650img is linked at 0xE000, the bottom of the emulator window,
// with .natcode moved up to 0xE800 to make room. Both are ABOVE the ceiling and
// below the pattern arena, and makesyx's app_page_ok() refuses any image whose
// pages reach the arena, so a future overrun is caught at build time.
//
// PROGMEM is redefined rather than the header edited because image_embed.h is
// GENERATED and gitignored: an edit there is lost on the next regeneration and
// cannot be reviewed. tools/d650/make_image_embed.py is untouched, so a
// regenerated header keeps working with no coordination.
#ifdef UCOM4_NATIVE
#  undef PROGMEM
#  define PROGMEM __attribute__((section(".d650img")))
#endif
#include "image_embed.h"          // generated locally; gitignored
#ifdef UCOM4_NATIVE
#  undef PROGMEM
#  define PROGMEM __attribute__((__progmem__))
#endif
#endif

extern PanelMap g_panel_map;

static d650_host H;
// The program-image copy shares midi.cpp's buffer arena: only one firmware runs per
// boot, so the 2 KB is spent once instead of twice (see the arena note there).
extern uint8_t g_shared_arena[];
static uint8_t *const s_img = g_shared_arena;
static bool      s_have_img = false;
static ImgRx     s_imgrx;

// ---------------------------------------------------------------------------
// Port mirror
// ---------------------------------------------------------------------------
// Every latch the emulated CPU writes goes straight to the socket pin it drives.
// The 808's own gates do the rest — a voice fires only where its data bit and
// COMMON TRIG overlap, exactly as with the real chip.
static const uint8_t PD_PINS[4] = { PD0_PIN, PD1_PIN, PD2_PIN, PD3_PIN };
static const uint8_t PE_PINS[4] = { PE0_PIN, PE1_PIN, PE2_PIN, PE3_PIN };
static const uint8_t PF_PINS[4] = { PF0_PIN, PF1_PIN, PF2_PIN, PF3_PIN };

static uint8_t s_ph = 0x0F, s_pg = 0x00;

// Direct-port pin writes. Arduino digitalWrite costs ~150 AVR clocks per call
// (PROGMEM pin-table lookups plus the PWM-disconnect check); the emulated
// program strobes PORTI (RAM /CE, i.e. EVERY pattern-memory access) and writes
// whole nibbles constantly, so at 4 MHz digitalWrite was the single biggest
// cost in the whole emulator: measured 137 clk per emulated cycle with the
// translated core, dominated by these hooks. Cache each pin's output register
// and mask once and a write is ~6 clocks. The PCINT ISR never touches these
// ports, so the read-modify-write needs no cli.
struct fastpin { volatile uint8_t *out; uint8_t bm; };
static fastpin s_fp_pd[4], s_fp_pe[4], s_fp_pf[4], s_fp_pi1, s_fp_pi2;
static void fastpin_init(fastpin *f, uint8_t pin) {
  f->out = portOutputRegister(digitalPinToPort(pin));
  f->bm  = digitalPinToBitMask(pin);
}
static inline void fast_wr(const fastpin *f, uint8_t on) {
  if (on) *f->out |= f->bm; else *f->out &= (uint8_t)~f->bm;
}
static void fastpins_init() {
  for (uint8_t i = 0; i < 4; ++i) {
    fastpin_init(&s_fp_pd[i], PD_PINS[i]);
    fastpin_init(&s_fp_pe[i], PE_PINS[i]);
    fastpin_init(&s_fp_pf[i], PF_PINS[i]);
  }
  fastpin_init(&s_fp_pi1, PI1_PIN);
  fastpin_init(&s_fp_pi2, PI2_PIN);
}
static void write_nib(const fastpin *f, uint8_t v) {
  fast_wr(&f[0], v & 1); fast_wr(&f[1], v & 2);
  fast_wr(&f[2], v & 4); fast_wr(&f[3], v & 8);
}

// ---------------------------------------------------------------------------
// Panel-lamp de-noise on PE0 / PE1
// ---------------------------------------------------------------------------
// Those two lines are triple-purpose (see the lamp note in engine.h): they are
// µPD444C address A8/A9, they are the CP and RS instrument-data bits, and they
// are the ONLY two lines reaching the BASIC VARIATION and 1ST/2ND PART lamp
// drivers. What turns that traffic back into a steady lamp on the real machine
// is not the CPU, it is R9/C5 and R10/C4 integrating the line over ~100 ms.
//
// Mirroring the latch bit-for-bit hands the panel the raw traffic and leans on
// those capacitors to sort it out, and they do not quite manage it here: the
// emulated machine's RAM bursts are periodic (one per sequencer step), so the
// average dips in step time and the OTHER lamp of the pair — the A lamp with
// BASIC VARIATION on B — visibly flickers along with it.
//
// So do the integrating here, where the answer can be a hard decision instead
// of an analog average sitting near a transistor's threshold:
//
//   - only intervals with NO RAM access in them are integrated. During an
//     access the line is an address and carries no lamp information at all;
//     excluding those is most of the win, and it is exactly the distinction the
//     host already tracks for its own RAM decode (/CE, d650_host.h).
//   - the decision has hysteresis, so a line hovering near the balance point
//     settles instead of chattering.
//   - the COMMON TRIG window passes RAW, so CP and RS still fire on the same
//     data the CPU put up. The pending raw value is flushed to the pins at the
//     trigger's rising edge, ahead of PI2, so the voice gate sees it inside
//     IC6's stretched ~1 ms output exactly as before.
//   - if no clean interval turns up for LAMP_STALE_MS the filter stands down
//     and the mirror goes back to raw, so a program that never lets go of the
//     bus is no worse off than before this existed.
//
// PE2/PE3 are untouched: they are HT/MT data and the RAM chip select, and they
// drive no lamp.
static const uint16_t LAMP_TRIG_HOLD_US = 1500;   // data window, as in engine.h
static const int16_t  LAMP_ACC_US       = 4000;   // evidence clamp, +/- 4 ms
static const int16_t  LAMP_HYST_US      = 2000;   // decision threshold
static const uint16_t LAMP_STALE_MS     = 1000;   // no clean interval: stand down

// claim-127: set while the config menu owns the panel. Declared here because
// hook_port and poll_matrix both consult it and both sit above the menu code.
static bool s_panel_borrowed = false;
static uint8_t  s_pe_raw    = 0;      // last PORTE latch the CPU wrote
static uint8_t  s_lamp      = 0;      // decided PE0/PE1
static bool     s_lamp_ok   = false;  // a decision exists (else mirror raw)
static int16_t  s_lamp_acc[2] = { 0, 0 };
static uint32_t s_lamp_us   = 0;      // last lamp_service() timestamp
static uint32_t s_lamp_ok_ms = 0;     // last clean interval
static bool     s_ce_seen   = false;  // a RAM access happened in this interval
static bool     s_trig_on   = false;  // inside the COMMON TRIG data window
static uint32_t s_trig_us   = 0;

static void write_pe() {
  uint8_t v = s_pe_raw;
  if (!s_trig_on && s_lamp_ok) v = (uint8_t)((v & 0x0C) | s_lamp);
  write_nib(s_fp_pe, v);
#ifdef SUPEROS_LAMP_PROBE
  // Same question asked of the 1981 program: how long does PE1 sit at a level
  // the lamp filter did not ask for? s_lamp bit 1 IS the emulator's decided
  // lamp level, so it is the honest `wanted` on this side.
  lamp_probe_edge(g_lamp_emu, (uint8_t)((v >> 1) & 1), (uint8_t)((s_lamp >> 1) & 1));
#endif
}

static void lamp_service(uint32_t now) {
  const uint32_t gap = now - s_lamp_us;
  s_lamp_us = now;

  if (s_trig_on && (uint32_t)(now - s_trig_us) >= LAMP_TRIG_HOLD_US) {
    s_trig_on = false;
    write_pe();                       // data window over: lamps get the lines back
  }

  const bool clean = !s_ce_seen && !s_trig_on;
  s_ce_seen = false;
  if (!clean) {
    if (s_lamp_ok && (uint32_t)(millis() - s_lamp_ok_ms) >= LAMP_STALE_MS) {
      s_lamp_ok = false;              // nothing clean to go on: back to raw
      write_pe();
    }
    return;
  }
  s_lamp_ok_ms = millis();

  const int16_t w = (int16_t)(gap > 1000 ? 1000 : gap);
  uint8_t next = s_lamp;
  for (uint8_t i = 0; i < 2; ++i) {
    int16_t a = (int16_t)(s_lamp_acc[i] + (((s_pe_raw >> i) & 1) ? w : (int16_t)-w));
    if (a >  LAMP_ACC_US) a =  LAMP_ACC_US;
    if (a < -LAMP_ACC_US) a = -LAMP_ACC_US;
    s_lamp_acc[i] = a;
    if (a >=  LAMP_HYST_US) next |= (uint8_t)(1 << i);
    if (a <= -LAMP_HYST_US) next &= (uint8_t)~(1 << i);
  }
  if (!s_lamp_ok || next != s_lamp) {
    s_lamp    = next;
    s_lamp_ok = true;
    write_pe();
  }
}

static void hook_port(void *, int port, uint8_t v) {
  switch (port) {
    case UCOM4_PORTD: write_nib(s_fp_pd, v); break;
    case UCOM4_PORTE: s_pe_raw = v; write_pe(); break;
    case UCOM4_PORTF: write_nib(s_fp_pf, v); break;
    // PORTC is the µPD444C data bus and is deliberately NOT mirrored. The RAM is
    // emulated in SRAM; the physical chips are still fitted and drive that bus,
    // so writing it is a bus fight — and with the SW1b mod it also stamps on the
    // pin 30 -> pin 2 path. The latch still lives in the host for the emulated
    // read/write cycle; it just never reaches a pin.
    case UCOM4_PORTC: break;
    case UCOM4_PORTG:                                 // step-LED rows
      s_pg = (uint8_t)(v & 0x0F);
      if (!s_panel_borrowed) PORTF = (uint8_t)((s_pg << 4) | (s_ph & 0x0F));
      break;
    case UCOM4_PORTH:                                 // scan selects
      s_ph = (uint8_t)(v & 0x0F);
      if (!s_panel_borrowed) PORTF = (uint8_t)((s_pg << 4) | s_ph);
      break;
    case UCOM4_PORTI: {
      const bool ce   = ((v >> 1) & 1) != 0;              // RAM CE
      const bool trig = ((v >> 2) & 1) != 0;              // COMMON TRIG
      if (ce) s_ce_seen = true;                           // this interval is dirty
      fast_wr(&s_fp_pi1, ce);
      if (trig) {
        // Instrument data on the pins BEFORE the trigger edge: the lamp filter
        // may be holding PE0/PE1 (CP and RS) at a lamp level.
        s_trig_us = micros();
        if (!s_trig_on) { s_trig_on = true; write_pe(); }
      }
      fast_wr(&s_fp_pi2, trig);
      break;
    }
    default: break;
  }
}

// ---------------------------------------------------------------------------
// Tempo / DIN / MIDI clock -> one pulse stretcher
// ---------------------------------------------------------------------------
// Ported from the other SuperOS firmwares' emu_sync scheme. Handing the CPU the
// raw line level does NOT work, for a reason that only shows up at the top of
// the tempo knob: the oscillator's pulses get narrower than the CPU's
// ~1.9 ms status sampling, edges get eaten, and MORE knob then means FEWER seen
// ticks — the tempo audibly slows down as you turn it up.
//
// So the physical line is sampled here at ~1.4 kHz, each rising edge is queued
// (2 deep), and the queue is re-synthesized as clean pulses on the level that
// emu_read_clock() serves: high for 2x the /INT sampler period, then low for at
// least 1x, so the CPU can never miss either phase. Anything the sampler cannot
// see (the very narrow pulses the oscillator emits while stopped), the original
// CPU could not see either.
//
// MIDI clock rides the SAME stretcher: a 0xF8 queues an edge exactly like a
// physical rising edge, and while MIDI clock is live (a 0xF8 within 300 ms) the
// physical line is ignored. That is what makes the emulator follow an external
// MIDI clock even though the real 808 had no MIDI at all.
// With the statically translated core the emulated machine holds real time
// even on the divided core clock, so the stretcher must NOT scale with F_CPU
// any more: scaling it would stretch every tempo-clock pulse 4x and the
// sequencer would run at a quarter speed even though the CPU keeps up.
#ifdef UCOM4_NATIVE
#ifdef D650_SVC_PROBE
// Where the wall clock goes outside the emulated CPU. Bench build only: every
// bracket costs two micros() calls, which at 4 MHz are not free themselves.
static uint16_t s_us_clk = 0, s_us_lamp = 0, s_us_pm = 0, s_us_outer = 0;
static uint16_t s_us_run = 0;   // worst single d650_run call
#define SVC_T0()      const uint32_t svc_t0_ = micros()
// MAX, not total: a stall that happens four times a second disappears into a
// per-second total but stands out immediately as a worst case.
#define SVC_ADD(acc)  do { const uint16_t d_ = (uint16_t)(micros() - svc_t0_); \
                           if (d_ > (acc)) (acc) = d_; } while (0)
#else
#define SVC_T0()      ((void)0)
#define SVC_ADD(acc)  ((void)0)
#endif
static const uint32_t EMU_SLOWDOWN      = 1;
#else
static const uint32_t EMU_SLOWDOWN      = 16000000UL / F_CPU;
#endif
static const uint32_t CLK_PULSE_US      = 3800UL * EMU_SLOWDOWN;   // 2x /INT
static const uint32_t CLK_MIN_LOW_US    = 1900UL * EMU_SLOWDOWN;   // 1x /INT
// Physical-line poll interval. Every tick waits out half of this on average
// before the emulator even notices the edge, and that sits on TOP of the ~1.9 ms
// status cadence the CPU samples on (which is authentic and stays). 700 us was
// costing ~350 us of avoidable mean lag against an external master; 250 us cuts
// it to ~125. The floor is the loop pass itself, which is longer than this, so
// in practice the line is now polled once per pass and there is nothing further
// to win here without moving to an ISR (see emu_pcint: the PA lines are the scan
// matrix, so a pin-change ISR fires on every column and cannot be qualified).
//
// The cost is the 15 us select park below happening more often, so read
// s_clk_park_us in poll_matrix before shrinking this again.
static const uint32_t CLK_SAMPLE_GAP_US = 250;

static uint8_t  s_clk_level    = 0;   // the level the CPU reads
static uint8_t  s_clk_pending  = 0;   // queued edges (max 2)
static uint8_t  s_clk_raw_prev = 0;
static uint32_t s_clk_fall_us  = 0;   // scheduled end of the high phase
static uint32_t s_clk_low_us   = 0;   // earliest next rise
static uint32_t s_clk_sample_us = 0;
static uint32_t s_clk_park_us   = 0;   // when the select park last restored
static uint8_t  s_status_div    = 0;   // status group runs 1 sample in 4
// D650_PROBE_G5_ONLY strips everything the probe publishes EXCEPT the 21-byte
// G5 record at CYC_PROBE_BASE. The auxiliary blocks (EE_TLM_RST/PNL/SEEN) and
// the per-scan maintenance that feeds
// them cost 333 B of app flash, and combined-nat-probe is over the 0xD700
// ceiling by exactly that order. G5 does not read any of them.
#if defined(D650_CYCLE_PROBE) && !defined(D650_PROBE_G5_ONLY)
#define D650_PROBE_AUX 1
#endif
#ifdef D650_PROBE_AUX
#warning "D650_PROBE_AUX publishes into the EE_TLM_* telemetry region, which shares its base with EE_IMG_DATA. Free on a combined build (the image is embedded, so nothing lives there) and NOT free on a build with an uploaded program image, where this overwrites the image every 4 s. Panel debugging only. Add -DD650_PROBE_G5_ONLY to strip it."
// Exactly what the matrix reads, per select column, before any debounce or
// decode: the only way to tell a mis-scanned panel from a mis-decoded one.
static uint8_t s_raw_pa[4], s_raw_pb[4], s_raw_status;
static uint8_t s_seen_pa[4], s_seen_pb[4];
#endif
static uint8_t  s_midi_run     = 0;   // MIDI transport, OR'd into the RUN status
static uint32_t s_last_f8_ms   = 0;   // MIDI clock liveness window

// MIDI clock counts only when the config menu says so. CLK_SRC_INTERNAL is the
// stock machine: the tempo knob and the rear sync input, and an
// incoming MIDI clock ignored outright. CLK_SRC_MIDI follows a MIDI clock
// whenever one is arriving and falls back to the physical line when it stops --
// the same rule the sequencer firmware uses, from the same setting (step 5 in
// the config menu), so the two firmwares cannot disagree about it.
static inline bool midi_clock_enabled() {
  return g_settings.clock_source == CLK_SRC_MIDI;
}
static inline bool midi_clock_live() {
  return midi_clock_enabled() && (millis() - s_last_f8_ms) < 300;
}
#ifdef D650_CYCLE_PROBE
static uint16_t s_clk_edges = 0;   // tempo-clock edges accepted this second
#endif
static inline void clk_queue_edge()  {
#ifdef D650_CYCLE_PROBE
  s_clk_edges++;
#endif
  if (s_clk_pending < 2) s_clk_pending++;
}

// Served at the exact instant the CPU reads its status group.
static uint8_t emu_read_clock(void *) { return s_clk_level; }

// Defined with the rest of the snapshot code below.
static void status_sample(uint8_t pa);

// Physical line sampler. Runs between interpreter batches (never concurrent with
// emulated port I/O), so parking the selects for 15 us is safe: the mirrored
// LED/scan state is restored before the CPU runs again. The clock line's bit
// within the status nibble is calibrated, not assumed (see controls.h).
static void clock_sample(uint32_t now_us) {
  if ((int32_t)(now_us - s_clk_sample_us) < (int32_t)CLK_SAMPLE_GAP_US) return;
  s_clk_sample_us = now_us;
  const uint8_t save_f = PORTF;
  PORTF = 0x0F;                     // selects parked -> the STATUS group is gated on
  delayMicroseconds(15);            // actively driven line: no key-settle needed
  const uint8_t pa = (uint8_t)(PINB & 0x0F);
  PORTF = save_f;
  // A REAL timestamp of when the park ended, and it has to be one.
  //
  // This was briefly `now_us + 30` to save a micros() call, which broke the
  // guard in poll_matrix completely: that compares (now - s_clk_park_us)
  // against the settle window using the SAME `now`, so a park end in the future
  // makes the unsigned subtraction wrap to ~4.29 billion, which is never less
  // than 120, and the settle wait never happened. poll_matrix then sampled the
  // key matrix while the lines were still settling through their 15 K
  // pull-downs and read switches as closed that were not -- which on the panel
  // looks like a three-position switch stuck in its middle position, e.g. 1ST
  // PART playing as 1ST + 2ND.
  s_clk_park_us = micros();
  // The park drops the CPU's select for those 15 us, so the key lines start
  // settling through their 15 K pull-downs all over again. poll_matrix times its
  // settle from the CPU's select change, which knows nothing about this, so
  // publish the restore instant and let it wait that out too. Harmless at the old
  // 700 us gap (a park rarely landed inside a ~475 us dwell); at 250 us it is
  // load-bearing.
#ifdef D650_PROBE_AUX
  s_raw_status = pa;
#endif
  const uint8_t raw = (uint8_t)((pa >> g_panel_map.clock_bit) & 1);
  if (raw && !s_clk_raw_prev && !midi_clock_live()) clk_queue_edge();
  s_clk_raw_prev = raw;
  // RUN / TAP / FILL IN ride along in this window, but they do not need the
  // clock line's rate. The clock is sampled ~1.1 kHz to catch the narrow pulses
  // the tempo oscillator emits at the top of its range; three momentary buttons
  // debounce fine at a quarter of that, and doing them every time cost three
  // shift-and-decide passes on every sample.
  if (++s_status_div & 3) return;
  // RUN / TAP / FILL IN are gated on in exactly this window, so read them here
  // too. They cannot come from the CPU's own scan the way the key columns do:
  // a key column is held for its ~475 us LED dwell, but the all-selects-high
  // state the STATUS group needs lasts a couple of instructions and passes
  // between polls. This window already exists, so the group costs nothing extra.
  status_sample(pa);
}

// Turn queued edges into sampler-visible highs and lows.
static void clock_service(uint32_t now_us) {
  if (s_clk_level && (int32_t)(now_us - s_clk_fall_us) >= 0) {
    s_clk_level  = 0;
    s_clk_low_us = now_us + CLK_MIN_LOW_US;
  }
  if (s_clk_pending && !s_clk_level && (int32_t)(now_us - s_clk_low_us) >= 0) {
    s_clk_pending--;
    s_clk_level   = 1;
    s_clk_fall_us = now_us + CLK_PULSE_US;
  }
}

void emu_pcint() { /* the clock is polled + stretched here; no ISR needed */ }

// ---------------------------------------------------------------------------
// Panel snapshot
// ---------------------------------------------------------------------------
// The emulated CPU reads the matrix through io_read_a/b, which index H.in.
//
// PASSIVE: the snapshot is read during the CPU'S OWN column dwell. We never
// touch the selects.
//
// This used to run a full hw::ScanMatrix every 4 ms, parking the CPU's PH/PG
// latch for ~450 us to do it. Its own column dwell is ~475 us, so a scan could
// swallow an entire column, and the beat between the 250 Hz scan and the CPU's
// 526 Hz multiplex showed up as visibly flickering step LEDs. The real
// µPD650C read the panel inside its own scan; a second scan on top of it was
// the anomaly.
//
// So: watch the select latch the CPU is driving, wait for the lines to settle
// inside that dwell, and take ONE sample of PINB — which carries both nibbles at
// once (PA0-3 = AVR PB0-3, the step switches = AVR PB4-7). Zero display
// disruption, and each row is sampled once per CPU scan cycle (~1.9 ms), so the
// 3-sample debounce lands at ~6 ms.
//
// The status group is remapped through the calibrated PanelMap into the slot
// order the emulated CPU expects (RUN, TAP, FILL, CLOCK), so a machine whose
// status bits sit in a different order than it assumes still behaves.
static uint8_t s_db[D650_IN_COUNT];      // 3-sample debounce shift registers

// Key rows come back through diodes into 15 K pull-downs and need the same
// settle hw.h documents. The status group is actively driven and needs almost
// none — the same distinction clock_sample() already relies on.
static const uint16_t PM_KEY_SETTLE_US    = 120;

// Take all THREE debounce samples INSIDE one column dwell, spaced, rather than
// one per dwell (claim-113).
//
// One-per-dwell was not debouncing. Measured against the 1981 program on the
// host: it revisits a given column every 7.58 ms, so three samples landed
// 7.58 ms apart, and contact bounce is over in about 1 ms. Nothing was being
// filtered; the spacing bought 22.7 ms of latency before the emulated CPU saw
// the key at all, and another 22.7 ms before it saw the release. SuperOS
// samples the same 3-deep register once per ~1 ms loop pass and registers in
// ~2 ms, which is why entering a step is easy on one side of the same panel and
// needs a deliberate slow press on the other.
//
// The dwell is long enough to hold the whole decision: measured at 191 machine
// cycles (1.68 ms) with a 73-cycle minimum, against a poll_matrix that runs
// every ~211 us. 120 us of settle then two more reads 400 us apart spends
// 920 us of that, so the register fills within the dwell and the key is
// delivered after ONE scan period instead of three.
//
// The old comment's warning is still right and is why the GAP exists: samples
// taken back to back fill the register with duplicates of one instant and the
// debounce stops debouncing. Spacing them across the dwell is what makes them
// three independent looks at the contact, which is the thing 7.58 ms apart
// never was. A dwell too short to fit three reads simply banks fewer and the
// register holds its last decision, exactly as before.
static const uint16_t PM_SAMPLE_GAP_US    = 400;
static const uint8_t  PM_SAMPLES_PER_DWELL = 3;   // the depth of s_db

static uint8_t  s_pm_ph    = 0xFF;   // select latch this dwell belongs to
static uint32_t s_pm_since = 0;      // when the CPU last changed it
static uint8_t  s_pm_n     = 0;      // samples banked in THIS dwell
static uint32_t s_pm_last  = 0;      // when the last one was taken

// ---------------------------------------------------------------------------
// CLEAR key
// ---------------------------------------------------------------------------
// RETIRED, claim-125: CLEAR used to be WITHHELD from the emulated CPU for
// EMU_CLR_DTAP_MS (600 ms) while a five-state machine watched for a double
// press, which rebooted into SuperOS. A press held past the window passed
// through; a lone short press was replayed as a synthetic 120 ms pulse.
//
// All of that existed because a CLEAR double-press was the ONLY way out of the
// emulator, and CLEAR is destructive (PATTERN CLEAR erases the selected
// pattern, COMPOSE erases the rhythm track), so the first press could not be
// handed over until the gesture resolved.
//
// It is not the only way out any more. Since claim-122 the config menu opens IN
// PLACE on CLEAR HELD + TAP, and menu STEP 9 switches firmware from there, with
// the STEP 1 + STEP 16 power-on escape still behind both. So the gesture was
// costing every CLEAR press 600 ms of latency to protect a route nothing needs.
//
// What is kept is s_clr_real, the debounced PHYSICAL level, because the menu
// chord needs the real key rather than a gated one.
static uint8_t  s_clr_real = 0;   // debounced physical level

// Where CLEAR sits in the snapshot: PH0 column, PA bit clear_bit (controls.h).
// Resolved once in emu_setup: g_panel_map is a calibrated table in RAM, and
// reading it per debounced cell put a load and a mask in the middle of the
// panel scan for an answer that cannot change while the emulator is running.
static uint8_t s_clr_idx = 16;
static uint8_t s_status_src[3] = { 0, 1, 2 };
static void panel_idx_init() {
  s_clr_idx = (uint8_t)(16 + (g_panel_map.clear_bit & 3));
  s_status_src[0] = g_panel_map.run_bit;
  s_status_src[1] = g_panel_map.tap_bit;
  s_status_src[2] = g_panel_map.fillin_bit;
}
static inline uint8_t clr_slot() { return s_clr_idx; }

#ifndef SUPEROS_NO_KEYQUEUE
// ---------------------------------------------------------------------------
// Write-mode direct step entry (claim-131, second take)
// ---------------------------------------------------------------------------
// A regular 808 owner taps step entry "as fast as i can input them ... no
// lag"; on ours fast entry missed. The 1981 program accepts roughly one
// entry per 1.5-2 SEQUENCER STEPS (write at tick 1 only, press latch starved
// until an idle servicing pass; tools/d650/fasttap.c measured it against the
// real image), so its ceiling scales with tempo. The first fix QUEUED presses
// and replayed them at the program's own pace -- lossless but late, and the
// player's verdict was immediate: "they should be instant just like the
// superos mode".
//
// So the bridge now writes the entry itself. The pattern store IS emulated
// RAM the bridge owns (H.ext); the program streams both its step LEDs and
// its playback from that RAM every scan, so a toggled bit is visible and
// audible exactly as if the program had written it. The address is computed
// from the PROGRAM'S OWN latches -- mINST (ram 0x12), mROWADR (ram 0x17),
// A9 (the variation) from the live bank register mBANK (ram 0x0E), which is
// what makes an AB dial land on the variation the machine is actually
// playing, and A8 (the part half) from the MODE itself: 2ND PART edits the
// second half by definition, and mBANK's own A8 is the compound-rhythm
// playback alternation, not the edit target (measured: it stays 0 in 2ND
// PART, which put 12 of 36 equivalence configs 256 nibbles off until the
// mode supplied the bit):
//
//     nibble = (mINST/4)*1024 + A9*512 + (mode==2ND)*256 + key*16 + mROWADR
//     bit    = mINST % 4
//
// measured, not derived: inst x key x bank sweeps against the real program
// on the host, one changed nibble each, zero exceptions
// (tools/d650/fasttap.c re-proves the equivalence on every run).
//
// In the write modes the step keys are HIDDEN from the emulated CPU: entry
// is the only thing its key path does there (stopped it does nothing at
// all, A606's finding), and both paths acting on one press would
// double-toggle. Everywhere else -- MANUAL PLAY's live voices, PLAY and
// COMPOSE's selects -- every key passes through untouched. Two deliberate
// departures from stock, both toward SuperOS: entry works with the
// transport STOPPED, and a press does not audition the voice.
//
// SUPEROS_NO_KEYQUEUE strips it: four instrument envs (-lamp, -hwm,
// -4mhz-nat-svc, -4mhz-nat-profile) sit within a few hundred bytes of the
// ceiling, and their questions do not involve step entry.
static bool s_de_gate = false;

static inline uint8_t de_mode() {
  // MODE latches ride H.in via the PA cells of select column 0 (controls.h):
  // 6 = 1ST PART, 7 = 2ND PART, the two step-write modes.
  return (uint8_t)(H.in[16] | (H.in[17] << 1) | (H.in[18] << 2));
}

static __attribute__((noinline)) void de_toggle(uint8_t key) {
  const uint8_t inst = (uint8_t)(D650_CPU(&H)->ram[0x12] & 0x0F);  // mINST
  const uint8_t row  = (uint8_t)(D650_CPU(&H)->ram[0x17] & 0x0F);  // mROWADR
  const uint8_t bank = (uint8_t)(D650_CPU(&H)->ram[0x0E] & 0x03);  // mBANK
  if (inst >= 12) return;                    // dial mid-detent: no target
  const uint16_t nib = (uint16_t)(((uint16_t)(inst >> 2) << 10)
                                  | ((uint16_t)(bank & 2) << 8)    // A9 * 512
                                  | ((de_mode() == 7) ? 256u : 0u) // A8: 2ND
                                  | ((uint16_t)key << 4) | row);
  uint8_t *cell = &H.ext[nib >> 1];
  const uint8_t m = (uint8_t)((1u << (inst & 3)) << ((nib & 1) << 2));
  *cell ^= m;
  H.ext_dirty = 1;                           // reach patt_service like a /WE
}

static void key_direct_service() {
  if (s_panel_borrowed) return;              // the menu owns the matrix
  const bool gate = (de_mode() >= 6);
  if (gate != s_de_gate) s_de_gate = gate;   // db_step republishes lazily
}
#endif  // SUPEROS_NO_KEYQUEUE

// Resolve ONE debounce shift register into the snapshot the CPU reads.
//
// This used to be a commit_snapshot() that swept all forty inputs, and it ran
// twice per emulation pass (the key scan and the status window each ended with
// one). At 4 MHz that sweep was on the order of 150 us a call against a pass
// that only has room for ~850 us of emulation, so the emulated machine was
// losing about a third of real time to re-deciding thirty-eight inputs that
// had not been sampled. Each sampler now commits only the cell it just shifted.
// Shift one sample into a debounce register and publish the result, in one
// pass over s_db[i]. Three variants because the three groups need different
// gating and the generic version was re-testing every special case on every
// cell of every scan: the step switches need none of it.
#define DB_SHIFT(i, bit) \
  const uint8_t v_ = (uint8_t)(((s_db[i] << 1) | (bit)) & 0x07); \
  s_db[i] = v_; \
  if (v_ != 0x07 && v_ != 0x00) return;   /* mixed: hold the last decision */

// The 16 step switches. The debounce register itself carries the press edge:
// the shift lands on 0b111 exactly once per press, on the third consecutive
// closed sample, so that transition feeds the write-mode queue with no
// per-key level shadow. The published H.in cell belongs to the queue's
// presenter whenever its gate is up (claim-131 above).
static inline void db_step(uint8_t i, uint8_t bit) {
#ifdef SUPEROS_NO_KEYQUEUE
  DB_SHIFT(i, bit)
  H.in[i] = (uint8_t)(v_ != 0);
#else
  const uint8_t o  = s_db[i];
  const uint8_t v_ = (uint8_t)(((o << 1) | bit) & 0x07);
  s_db[i] = v_;
  if (v_ != 0x07 && v_ != 0x00) return;   /* mixed: hold the last decision */
  if (s_de_gate) {
    // The press edge is the shift landing on 0b111: exactly once per press,
    // on the third consecutive closed sample (~2 ms after the finger).
    if (v_ == 0x07 && o != 0x07) de_toggle(i);
    H.in[i] = 0;                          /* entry replaced: hide the key */
  } else {
    H.in[i] = (uint8_t)(v_ != 0);
  }
#endif
}

// The PA cells, one of which is the gated CLEAR: the physical level feeds the
// double-tap machine and the CPU is handed whatever that decides, published by
// clear_gesture.
static inline void db_pa(uint8_t i, uint8_t bit) {
  DB_SHIFT(i, bit)
  const uint8_t lvl = (uint8_t)(v_ != 0);
  // claim-125: record the physical level for the menu chord AND hand it to the
  // CPU. It used to return here, withholding the key while the double-press
  // gesture decided what it was; there is no gesture to decide any more.
  if (i == s_clr_idx) s_clr_real = lvl;
  H.in[i] = lvl;
}

// RUN / TAP / FILL IN. A MIDI Start/Continue runs the machine even with the
// panel flip-flop idle: the CPU reads RUN as a level, so the transport simply
// ORs into it.
static inline void db_status(uint8_t i, uint8_t bit) {
  DB_SHIFT(i, bit)
  uint8_t lvl = (uint8_t)(v_ != 0);
  if (i == D650_IN_RUN && s_midi_run) lvl = 1;
  H.in[i] = lvl;
}

#ifdef D650_CYCLE_PROBE
// Emulated-throughput probe. BENCH BUILD ONLY: it writes EEPROM once a second,
// which is both slow (~27 ms of the second) and finite (100k cycles is about a
// day of running), so it is never in a shipping image.
//
// Reported over EEPROM rather than SysEx because the combined build's outgoing
// MIDI is unreliable, so the only trustworthy readback is ISP while the machine
// keeps running. Read it with:
//   avrdude -c atmelice_isp -p at90usb1286 -B 125kHz -U eeprom:r:ee.bin:r
//
// The number that matters is machine cycles per second against the real chip's
// 113636. ucom4.c is tuned to just meet that at 16 MHz (a 141-clock budget per
// machine cycle); at 4 MHz the same budget is 35 clocks, so this is the
// measurement that says how far off real time the emulated machine is.
//
// EEPROM 0x00B..0x01F:  5A | cycles/s (u32 LE) | passes/s (u16 LE) | ... | A5
//
// MOVED from 0x010 on 2026-08-26, measured not suspected. The record is 21
// bytes, so at 0x010 it ran to 0x024 and overwrote the first FIVE bytes of
// EE_EMU_PATT (0x020), the emulator's own uPD444C pattern image, once every
// 4 s. Diffing an EEPROM dump before and after a G5 run is what found it:
// 0x020..0x024 went FF FF FF FF FF -> 00 00 00 00 A5, which is the tail of the
// probe record sitting in the pattern store. 0x00B is the first free byte
// above EE_MENU_ON_BOOT and 0x00B+21 lands exactly on EE_EMU_PATT.
//
// The static_assert is the part that matters more than the move: this overlap
// existed because NOTHING CHECKED, and hardcoding the next free address is what
// made it possible. Three overlaps have now been found here by reading the map,
// never by a symptom: this one, the auxiliary blocks inside EE_IMG_DATA, and the
// PC profile's ready marker writing the last byte of EE_EMU_PATT. A telemetry
// address that lands in a data region is silent by construction: it corrupts the
// thing being measured on and reads as a fault in it. Every address is now an
// integer constant in combined.h, derived from the block before it.
#define CYC_PROBE_BASE ((uint8_t *)EE_CYC_PROBE_ADDR)
// On INTEGERS, not on the pointer macro. The pointer form compiles on avr-gcc as
// an extension and fails under clang ("static assertion expression is not an
// integral constant expression"), so a host build of this file would have to
// delete the assert to go green, which is the one outcome worth designing out.
static_assert(EE_CYC_PROBE_ADDR >= EE_MENU_ON_BOOT_ADDR + 1,
              "the G5 probe record overlaps the boot-menu one-shot flag");
static_assert(EE_CYC_PROBE_ADDR + EE_CYC_PROBE_BYTES <= EE_EMU_PATT_ADDR,
              "the G5 probe record overlaps EE_EMU_PATT: it would overwrite the "
              "emulator's pattern image every window");
static uint32_t s_cyc_acc  = 0;
static uint16_t s_pass_acc = 0;
static uint32_t s_cyc_ms   = 0;
static uint32_t s_run_us   = 0;   // us spent inside d650_run this second
static uint16_t s_owed_peak = 0;    // worst cycle backlog seen this window
static uint16_t s_gap_peak  = 0;    // longest pass-to-pass stall, us
static bool     s_probe_wrote = false;  // this pass published, so it stalled
static uint16_t s_hitches   = 0;    // passes longer than 2 ms this window
static uint16_t s_ran_peak  = 0;    // largest single d650_run return
static void pace_reset();           // defined with the pacing state below
static void cycle_probe(uint32_t ran) {
  s_cyc_acc += ran;
  s_pass_acc++;
  // Worst overshoot of the cycle budget. The translated core can only leave at
  // a backward jump, so a long forward-only stretch of program runs past its
  // batch and the pass gets correspondingly long.
  if (ran > s_ran_peak) s_ran_peak = (uint16_t)ran;
  const uint32_t now = millis();
  // Four seconds, not one. An EEPROM byte costs ~3.3 ms to write and this
  // record is 21 of them, so publishing every second spent ~70 ms of every
  // second doing it -- 7% of the machine, charged to the thing being measured.
  if ((uint32_t)(now - s_cyc_ms) < 4000) return;
  s_cyc_ms = now;
#ifdef UCOM4_NATIVE
  const uint16_t nc = s_ran_peak; s_ran_peak = 0; ucom4_nat_cycles = 0;
  ucom4_nat_calls = 0;
  const uint8_t  ns = ucom4_nat_state();
#else
  const uint16_t nc = 0; const uint8_t ns = 0xEE;
#endif
  // ucom4_int_cycles / ucom4_isr_count are UCOM4_NATIVE-only telemetry (their
  // externs are guarded at the top of this file). The plain interpreter build
  // (env:combined-probe, 16 MHz) has neither, so guard these two fields or the
  // probe fails to compile. cycles/s, passes/s, hitches and backlog below do not
  // depend on them, so the interpreter throughput number is still measured.
#ifdef UCOM4_NATIVE
  const uint16_t io_ = (uint16_t)(ucom4_int_cycles / 4); ucom4_int_cycles = 0;
  const uint16_t ic = (uint16_t)(ucom4_isr_count / 4);  ucom4_isr_count = 0;
#else
  const uint16_t io_ = 0;
  const uint16_t ic = 0;
#endif
  const uint16_t ce_ = (uint16_t)(s_clk_edges / 4); s_clk_edges = 0;
  const uint16_t ih = s_hitches;  s_hitches = 0;
  s_cyc_acc  /= 4;                 // the window is 4 s; publish per-second
  s_pass_acc /= 4;
  uint8_t v[EE_CYC_PROBE_BYTES] = {
    0x5A,
    (uint8_t)s_cyc_acc,  (uint8_t)(s_cyc_acc >> 8),
    (uint8_t)(s_cyc_acc >> 16), (uint8_t)(s_cyc_acc >> 24),
    (uint8_t)s_pass_acc, (uint8_t)(s_pass_acc >> 8),
    ns, (uint8_t)nc, (uint8_t)(nc >> 8),
    (uint8_t)s_gap_peak, (uint8_t)(s_gap_peak >> 8),
    (uint8_t)ih, (uint8_t)(ih >> 8),
    (uint8_t)ic, (uint8_t)(ic >> 8),
    (uint8_t)io_, (uint8_t)(io_ >> 8),
    (uint8_t)ce_, (uint8_t)(ce_ >> 8),
    0xA5
  };
  s_run_us = 0;
  d650_st_write_block(v, CYC_PROBE_BASE, sizeof v);
  // The block above is ~8 changed EEPROM bytes at 3.3 ms each: this pass just
  // stalled ~25 ms and it is the measurement, not the machine. Do not let it
  // land in either worst-case figure.
#ifdef D650_PROBE_AUX
  {
    extern uint8_t g_reset_cause, g_boot_count;
    uint8_t rst[EE_TLM_RST_BYTES] = { 0xD2, g_reset_cause, g_boot_count, 0xA5 };
    d650_st_write_block(rst, (uint8_t *)EE_TLM_RST_ADDR, sizeof rst);
  }
  {
    uint8_t pnl[EE_TLM_PNL_BYTES] = { 0xD1,
      s_raw_pa[0], s_raw_pa[1], s_raw_pa[2], s_raw_pa[3],
      s_raw_pb[0], s_raw_pb[1], s_raw_pb[2], s_raw_pb[3],
      s_raw_status, 0xA5 };
    d650_st_write_block(pnl, (uint8_t *)EE_TLM_PNL_ADDR, sizeof pnl);
    uint8_t seen[EE_TLM_SEEN_BYTES] = { 0xD3,
      s_seen_pa[0], s_seen_pa[1], s_seen_pa[2], s_seen_pa[3],
      s_seen_pb[0], s_seen_pb[1], s_seen_pb[2], s_seen_pb[3], 0xA5 };
    d650_st_write_block(seen, (uint8_t *)EE_TLM_SEEN_ADDR, sizeof seen);
  }
#endif
  s_owed_peak   = 0;
  s_gap_peak    = 0;
  s_probe_wrote = true;
  // Do not make the emulated CPU repay time the measurement stole: without
  // this the backlog sits at the clamp for the ~20 passes it takes to work off
  // a 25 ms debt, and every window reports "behind" when it is not.
  pace_reset();
#ifdef D650_SVC_PROBE
  {
    uint8_t sv[EE_TLM_SVC_BYTES] = { 0xC3,
      (uint8_t)s_us_clk,   (uint8_t)(s_us_clk >> 8),
      (uint8_t)s_us_lamp,  (uint8_t)(s_us_lamp >> 8),
      (uint8_t)s_us_pm,    (uint8_t)(s_us_pm >> 8),
      (uint8_t)s_us_outer, (uint8_t)(s_us_outer >> 8),
      (uint8_t)s_us_run,   (uint8_t)(s_us_run >> 8),
      0xA5 };
    s_us_clk = s_us_lamp = s_us_pm = s_us_outer = s_us_run = 0;
    d650_st_write_block(sv, (uint8_t *)EE_TLM_SVC_ADDR, sizeof sv);
  }
#endif
  s_cyc_acc  = 0;
  s_pass_acc = 0;
}
#endif

#ifdef UCOM4_PC_PROFILE
// Emulated-PC histogram, published once after a 10 s window and then left
// alone: 32 buckets of 64 program bytes, each scaled to a percent of the total
// so it fits a byte. EE_TLM_PROF_ADDR, with EE_TLM_PROF_RDY_ADDR as the "ready"
// marker. The marker used to be at 0x81F, the last byte of the pattern store.
// Bench only. Read with the same ISP eeprom dump as the cycle probe.
extern "C" uint16_t ucom4_pc_hist[32];
static uint32_t s_prof_ms   = 0;
static bool     s_prof_done = false;
static void pc_profile_service() {
  if (s_prof_done) return;
  // Only profile a RUNNING machine. Idle and playing execute almost disjoint
  // parts of the program, and it is the playing one that has to hold real time.
  if (!s_midi_run && !H.in[D650_IN_RUN]) {
    for (uint8_t i = 0; i < 32; ++i) ucom4_pc_hist[i] = 0;
    s_prof_ms = millis();
    return;
  }
  if (s_prof_ms == 0) { s_prof_ms = millis(); return; }
  if ((uint32_t)(millis() - s_prof_ms) < 10000) return;
  uint32_t total = 0;
  for (uint8_t i = 0; i < 32; ++i) total += ucom4_pc_hist[i];
  if (!total) total = 1;
  uint8_t pct[EE_TLM_PROF_BYTES];
  for (uint8_t i = 0; i < 32; ++i)
    pct[i] = (uint8_t)((uint32_t)ucom4_pc_hist[i] * 100u / total);
  d650_st_write_block(pct, (uint8_t *)EE_TLM_PROF_ADDR, sizeof pct);
  // The marker was at 0x81F, which is the LAST BYTE OF EE_EMU_PATT (0x020 +
  // 2048 - 1), so every completed profile run wrote 0x5A into the emulator's
  // pattern store. Measured by reading the map, not by a symptom: nothing
  // reports it, and it would have surfaced as a corrupt pattern.
  d650_st_write((uint8_t *)EE_TLM_PROF_RDY_ADDR, 0x5A);
  s_prof_done = true;
}
#endif

static void poll_matrix(uint32_t now_us) {
  if (s_panel_borrowed) return;          // the menu is driving the matrix
  // s_ph, not PORTF: clock_sample() parks the real port for 15 us at a time and
  // reading that back would look like a select change every sample.
  const uint8_t ph = (uint8_t)(s_ph & 0x0F);
  if (ph != s_pm_ph) { s_pm_ph = ph; s_pm_since = now_us; s_pm_n = 0; return; }
  if (s_pm_n >= PM_SAMPLES_PER_DWELL) return;

  // Active-low at the CPU pin: it raises all four and clears the one it
  // wants. All high = the STATUS group is gated on instead.
  uint8_t row = 4;
  for (uint8_t s = 0; s < 4; ++s) if (!((ph >> s) & 1)) { row = s; break; }

  if (row >= 4) return;                 // STATUS: see status_sample()
  if ((uint32_t)(now_us - s_pm_since) < PM_KEY_SETTLE_US) return;
  if (s_pm_n && (uint32_t)(now_us - s_pm_last) < PM_SAMPLE_GAP_US) return;
  if ((uint32_t)(now_us - s_clk_park_us) < PM_KEY_SETTLE_US) return;  // see there
  s_pm_last = now_us;
  ++s_pm_n;

  const uint8_t pins = PINB;
  const uint8_t pa   = (uint8_t)(pins & 0x0F);         // PA0-3
  const uint8_t pb   = (uint8_t)((pins >> 4) & 0x0F);  // the 16 step switches
#ifdef D650_PROBE_AUX
  s_raw_pa[row] = pa; s_raw_pb[row] = pb;    // ground truth for panel debugging
  // Sticky: every line that has EVER read high since boot. An ISP read resets
  // the part, so the instantaneous snapshot only ever catches whatever was held
  // at that instant; this records a press whenever it happens.
  s_seen_pa[row] |= pa; s_seen_pb[row] |= pb;
#endif

  if (row < 4) {
    for (uint8_t j = 0; j < 4; ++j) {
      const uint8_t kb = (uint8_t)(row * 4 + j);            // step switches
      db_step(kb, (uint8_t)((pb >> j) & 1));
      const uint8_t ka = (uint8_t)(16 + row * 4 + j);       // PA cells
      db_pa(ka, (uint8_t)((pa >> j) & 1));
    }
  }
}

// The STATUS group, sampled from clock_sample()'s parked window (see there).
// The clock slot itself is skipped: it is served live by emu_read_clock.
static void status_sample(uint8_t pa) {
  for (uint8_t j = 0; j < 3; ++j)
    db_status((uint8_t)(D650_IN_STATUS + j),
              (uint8_t)((pa >> s_status_src[j]) & 1));
}

// ---------------------------------------------------------------------------
// Pattern-store persistence
// ---------------------------------------------------------------------------
// The emulated µPD444C image is battery-backed on the real machine. Here it
// lives in internal EEPROM and is written back in small chunks whenever it goes
// dirty and the machine has been quiet for a moment, so a save can never stall
// the interpreter mid-measure.
static uint16_t s_save_off  = 0;
static bool     s_saving    = false;
static uint32_t s_dirty_ms  = 0;

static void patt_load() {
  if (d650_st_read(EE_EMU_MAGIC) == EE_EMU_MAGIC_VAL) {
    d650_st_read_block(H.ext, EE_EMU_PATT, D650_EXT_BYTES);
  } else {
    // Fresh area: an erased/garbage image reads as every step of every pattern
    // filled, which sounds like a fault. Seed it empty and stamp the magic.
    memset(H.ext, 0, sizeof H.ext);
    for (uint16_t i = 0; i < D650_EXT_BYTES; i += 64) {
      d650_st_write_block(H.ext + i, EE_EMU_PATT + i, 64);
      wdt_reset();
    }
    d650_st_write(EE_EMU_MAGIC, EE_EMU_MAGIC_VAL);
  }
  d650_clear_dirty(&H);
}

// Start one EEPROM byte and return. The hardware finishes it in ~3.3 ms on its
// own; nothing here waits for it.
//
// This is the difference between a clean machine and one that stutters when you
// enter a step. eeprom_update_byte BUSY-WAITS for the cell, and patt_service
// used to push a 64-byte block through it from inside the emulator's pass loop:
// every changed byte stopped the emulated CPU dead for 3.3 ms, which is more
// than three /INT periods, so the display multiplex froze, panel scanning
// stopped and the sequencer missed clock edges -- for as long as the save took.
// Editing a pattern is exactly when that happens, and exactly when it is most
// obvious.
static void ee_start_write(uint8_t *addr, uint8_t v) {
  EEAR = (uint16_t)(uintptr_t)addr;
  EEDR = v;
  const uint8_t sreg = SREG;
  cli();                        // EEMPE -> EEPE must be within four cycles
  EECR |= (uint8_t)(1 << EEMPE);
  EECR |= (uint8_t)(1 << EEPE);
  SREG = sreg;
}
static inline bool ee_busy() { return (EECR & (1 << EEPE)) != 0; }

static void patt_service() {
  // Never start a save while the machine is playing.
  //
  // The µPD444C store is not just patterns: per the memory map it also holds
  // RHYTHM TRACK / PRESCALE / STEP, and the sequencer rewrites those as it
  // advances. So the store is permanently dirty while running, and this used to
  // respond by saving continuously -- measured at ~300 EEPROM byte writes a
  // second, which is both a stall every few passes and, at 100k cycles of
  // endurance, a cell worn out in about five minutes of playing.
  //
  // Nothing is lost by waiting: a step edit is still saved the moment the
  // transport stops, and patt_flush covers a firmware switch or reboot.
  const bool playing = s_midi_run || H.in[D650_IN_RUN];
  if (d650_dirty(&H) && !s_saving && !playing) {
    s_dirty_ms = millis();
    d650_clear_dirty(&H);
    s_saving   = true;
    s_save_off = 0;
  }
  if (!s_saving) return;
  if ((uint32_t)(millis() - s_dirty_ms) < 400) return;   // let edits settle
  if (ee_busy()) return;                                 // one still in flight

  // Skip bytes that already match, up to a bounded run per pass: a read is
  // cheap, a write is not, and after a single step edit almost the whole image
  // is unchanged.
  uint8_t scan = 64;
  while (s_save_off < D650_EXT_BYTES && scan--) {
    if (d650_st_read(EE_EMU_PATT + s_save_off) != H.ext[s_save_off]) {
      ee_start_write(EE_EMU_PATT + s_save_off, H.ext[s_save_off]);
      s_save_off++;
      return;                          // one write per pass, never blocking
    }
    s_save_off++;
  }
  if (s_save_off >= D650_EXT_BYTES) s_saving = false;
}

// Finish the store synchronously, for the one moment there is no next loop pass
// to finish it in: the firmware switch. The chunked writer above can have up to
// 400 ms of settle plus a partial pass outstanding, and the reboot would take
// those edits with it. eeprom_update_block skips unchanged bytes, so this is
// cheap on a store that is mostly already written.
// The synchronous counterpart, for the way out: a firmware switch or a reboot
// must not leave half an image behind, so this one does wait.
static void patt_flush() {
  if (!s_saving && !d650_dirty(&H)) return;
  while (ee_busy()) { }              // let any async byte finish first
  d650_clear_dirty(&H);
  for (uint16_t i = 0; i < D650_EXT_BYTES; i += 64) {
    d650_st_write_block(H.ext + i, EE_EMU_PATT + i, 64);
    wdt_reset();
  }
  s_saving = false;
}

// The CLEAR gesture machine described above db_pa(). Called once per
// loop pass; s_clr_real is refreshed by every snapshot commit.
void superos_menu_takeover();

// CLEAR HELD + TAP PRESSED opens the SuperOS config menu IN PLACE (claim-122).
//
// The same chord as on the SuperOS side, and legal for the same reason: TAP is
// a ~15 ms one-shot in the 1981 circuit, so it supplies the EDGE while CLEAR
// supplies the HOLD. Reversed it would be unbuildable.
//
// TAP is read from the debounced status snapshot the emulated CPU sees, and
// CLEAR from s_clr_real, which is the PHYSICAL level before the double-press
// machine gates it. Reading the gated one would mean waiting out
// EMU_CLR_DTAP_MS before the chord could even be noticed.
//
// On return the same TAP is not read twice, and the panel is handed back to the
// emulated CPU. CLEAR itself needs no unwinding since claim-125: it is passed
// straight through to the CPU rather than gated behind a gesture machine.
static uint8_t s_tap_prev = 0;
// While the config menu owns the panel (claim-127) the emulated machine KEEPS
// RUNNING. Only its PANEL I/O is suspended:
//
//   * PORTF writes are dropped. PORTF is the LED rows and the scan selects and
//     nothing else -- the instrument data and the COMMON TRIG are on PORTD,
//     PORTE and PI2 -- so the emulated program still sounds, in time, while the
//     menu draws. That is the whole reason this can work at all.
//   * poll_matrix is suspended, because it would otherwise sample PINB during
//     the MENU's scan and hand the emulated CPU key presses aimed at the menu.
//     The CPU simply sees no key changes, which is true: the player is in the
//     menu.
//
// s_pg and s_ph are still tracked, so the moment the panel comes back the next
// PORTF write restores whatever column the emulated program believed it had.
void emu_panel_borrow(bool on) {
  s_panel_borrowed = on;
  if (!on) { PORTF = (uint8_t)((s_pg << 4) | (s_ph & 0x0F)); s_pm_ph = 0xFF; }
}

static void menu_gesture(void) {
  if (s_panel_borrowed) return;          // no re-entry from inside the menu
  const uint8_t tap = H.in[D650_IN_TAP];
  const uint8_t rising = (uint8_t)(tap && !s_tap_prev);
  s_tap_prev = tap;
  if (!rising || !s_clr_real) return;

  emu_panel_borrow(true);
  superos_menu_takeover();          // runs the emulator; only the panel is ours
  emu_panel_borrow(false);

  s_tap_prev = 1;                   // do not read the same TAP twice
  write_pe();                       // the menu drove the panel; restore ours
}

// CLEAR now goes STRAIGHT TO THE EMULATED CPU (claim-125).
//
// It used to be withheld for EMU_CLR_DTAP_MS while a state machine watched for
// a DOUBLE PRESS, which rebooted the machine into SuperOS. That gesture existed
// because nothing else could leave the emulator. Since claim-122 the config
// menu opens IN PLACE on CLEAR + TAP, and menu STEP 9 switches firmware from
// there, so the reboot-on-CLEAR is redundant -- and the player asked for it to
// go: "it seem it didnt remove the hold clear to reset it".
//
// Removing the gesture removes its whole cost, which is the part worth having.
// The withholding was never free: EVERY CLEAR press was delayed 600 ms and then
// replayed as a synthetic 120 ms pulse, on a key whose stock jobs are PATTERN
// CLEAR, clearing the rhythm track, and the STEP NUMBER chord. The 1981 program
// now sees the real key, with the real timing, debounced and nothing else.
//
// s_clr_real stays: menu_gesture() reads it for the CLEAR-held half of the
// chord, which needs the PHYSICAL level rather than anything gated.

// ---------------------------------------------------------------------------
// MIDI: firmware switch and program-image upload
// ---------------------------------------------------------------------------
// The emulator understands exactly two things over MIDI — the 0x4D firmware
// switch that gets you back to SuperOS, and a program-image upload. Everything
// else is ignored; the emulated machine has no MIDI of its own.
static uint8_t s_sx_state = 0;   // 0 idle, 1 saw F0, 2 saw 7D, 3 in 0x4D

// One MIDI real-time byte, from either port.
static void midi_realtime(uint8_t b) {
  switch (b) {
    case 0xF8:
      if (!midi_clock_enabled()) break;         // INTERNAL: DIN/knob only
      s_last_f8_ms = millis(); clk_queue_edge();
      break;
    case 0xFA:                                  // Start
    case 0xFB: s_midi_run = 1; H.in[D650_IN_RUN] = 1; break;   // Continue
    case 0xFC: s_midi_run = 0; break;           // Stop
    default: break;
  }
}

// One non-realtime MIDI byte, from either port. Split out of the DIN loop so
// USB can reach the same parser: see usb_sysex_chunk below.
static void midi_byte(uint8_t b) {
  // The image receiver needs the whole stream, including F0/F7. On boards with
  // no room to store one it is a stub that never fires.
  if (s_imgrx.Byte(b)) {
#ifdef D650_IMG_UPLOADABLE
    img_save(s_img);
#endif
    s_have_img = true;
    d650_init(&H, s_img, nullptr);       // restart on the new image
    d650_drivers drv = { hook_port, nullptr, emu_read_clock };
    H.drv = drv;
    patt_load();
  }

  switch (s_sx_state) {
    case 0: if (b == 0xF0) s_sx_state = 1; break;
    case 1: s_sx_state = (b == 0x7D) ? 2 : 0; break;
    case 2:
      if (b == 0x4D) s_sx_state = 3;
      else if (b == 0x4A) {
        // Reboot into the resident SysEx bootloader (same command as the
        // SuperOS side), so a firmware update needs no panel access. Park
        // BOOT_MAGIC in GPIOR0 (survives the jump, cleared by any real
        // reset) so the bootloader stays resident, and detach USB first so
        // the bootloader's own device enumerates cleanly.
        patt_flush();
        cli();
#ifdef SUPEROS_USB_MIDI
        UDIEN = 0;
        UDCON |= (1 << DETACH);
        _delay_ms(30);                 // host must register the drop first
        USBCON = 0;
        PLLCSR = 0;
#endif
        GPIOR0 = 0xB7;                 // BOOT_MAGIC, keep in sync with bootload.c
        asm volatile("jmp 0x1F000");   // boot section (BOOTRST, BOOTSZ=01)
      }
      else if (b == 0xF7) s_sx_state = 0;
      else s_sx_state = 4;                      // some other command: skip
      break;
    case 3:
      if (b <= 1) {
        patt_flush();
        combined_switch_firmware(b);            // does not return
      }
      s_sx_state = 4;
      break;
    default: if (b == 0xF7) s_sx_state = 0; break;
  }
}

#ifdef SUPEROS_USB_MIDI
// SysEx arriving over USB-C, fed to the same parser the DIN port uses.
//
// This closes a trap. The drain below takes real-time bytes only, which was a
// deliberate choice -- the emulated machine has no notion of a MIDI note -- but
// 0x4D and 0x4A are not notes, they are the only two ways out of emulator mode,
// and on a rig wired with USB-C and no DIN cable neither could be delivered.
// The machine could still be escaped from the panel, but nothing on the host
// could reach it: not the firmware switch, and not even the jump into the
// bootloader that a firmware update needs. Measured on hardware 2026-08-25.
//
// No reassembly buffer: bytes go straight into the byte-wise state machine, so
// this costs nothing in RAM, which a combined build at 91% cannot spare.
static bool s_usb_sx_open = false;

static void usb_sysex_chunk(const uint8_t *d, uint16_t n, bool complete) {
  if (n) {
    // Normalize framing: chunks carry the wire bytes (F0..F7), but tolerate a
    // core that strips them, or the state machine never leaves state 0.
    if (!s_usb_sx_open && d[0] != 0xF0) midi_byte(0xF0);
    s_usb_sx_open = true;
    for (uint16_t i = 0; i < n; ++i) {
      const uint8_t b = d[i];
      // Real-time can be embedded mid-SysEx; it must not reach the parser.
      if (b >= 0xF8) { midi_realtime(b); continue; }
      midi_byte(b);
    }
    if (complete && d[n - 1] != 0xF7) midi_byte(0xF7);
  }
  if (complete) s_usb_sx_open = false;
}
#endif

static void midi_poll() {
#ifdef SUPEROS_USB_MIDI
  // USB-C, in emulator mode. The port enumerates either way -- the USB stack
  // comes up in main.cpp before the firmware split -- but nothing here used to
  // DRAIN it, so a MIDI clock arriving over USB was received by the chip and
  // then thrown away, and the emulated machine could only be synced from the
  // DIN socket. Notes are still ignored, because the emulated 808 has no notion
  // of one; clock, transport and SysEx are the three things that mean anything.
  //
  // Bounded drain. The sequencer firmware runs 48 of these a pass, which it can
  // afford; here it cost 236 ms of every second (measured), because this loop
  // runs on the emulator's pass cadence of ~700 Hz and an empty usbMIDI.read()
  // is not free. Eight packets a pass is 5600 a second, far more than a MIDI
  // clock stream can produce, and the endpoint holds anything that does not fit
  // until the next pass.
  static bool hooked = false;
  if (!hooked) { hooked = true; usbMIDI.setHandleSystemExclusive(usb_sysex_chunk); }
  for (uint8_t n = 0; n < 8; ++n) {
    // false means empty OR a SysEx continuation packet was consumed, so this
    // keeps going rather than breaking, or a multi-packet SysEx would dribble
    // in at one packet per pass and never complete.
    if (!usbMIDI.read()) continue;
    const uint8_t ut = usbMIDI.getType();
    if (ut >= 0xF8) midi_realtime(ut);
  }
#endif
  while (Serial1.available() > 0) {
    const uint8_t b = (uint8_t)Serial1.read();

    // System Real-Time can interleave ANYWHERE, including mid-SysEx, so it is
    // handled first and never reaches the parsers below — ImgRx aborts on any
    // status byte, and a clock tick arriving during an image upload would
    // otherwise kill the transfer.
    if (b >= 0xF8) { midi_realtime(b); continue; }

    midi_byte(b);
  }
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------
// One /INT half-period's worth of emulated time: 950 us x 113636 cycles/s.
// The batch sets the service cadence (they run once per batch) and the interrupt
// cadence (an edge can only be delivered between batches), so matching it to the
// half-period puts one service pass and one /INT edge in step with each other.
// Bigger costs interrupt resolution; smaller pays the ~520-clock cost of
// entering the translated core more often per emulated cycle.
static const int16_t EMU_BATCH = 108;

// ---------------------------------------------------------------------------
// Emulated-time pacing
// ---------------------------------------------------------------------------
// The µPD650C-085 retires 113636 machine cycles a second (455 kHz / 4). The
// translated core is now quicker than that on a 4 MHz AT90USB1286 -- left to
// run flat out it measured 111% of real time -- and faster is not better here:
// everything the program times by counting its own instructions (the step-LED
// dwell, key repeat, the fill-in) would run 11% quick against a tempo clock
// that is still arriving in real seconds.
//
// So the CPU is handed exactly the cycles the real part would have retired
// since the last pass, and any wall time left over goes to the panel and clock
// services instead. 7447/65536 = 0.1136322 cycles per microsecond, 36 ppm under
// the true 0.1136364; the fractional remainder is carried, so the error does
// not accumulate.
static const uint16_t PACE_CYC_PER_US_Q16 = 7447;
static const int32_t  PACE_MAX_OWED       = 2000;  // ~18 ms: never burst after a stall
static uint32_t s_pace_us   = 0;
static uint16_t s_pace_frac = 0;
static int32_t  s_owed      = 0;

void pace_reset() { s_owed = 0; s_pace_us = micros(); }

static int16_t cycles_due(uint32_t now) {
  uint32_t dt = now - s_pace_us;
  s_pace_us = now;
  if (dt > 20000u) dt = 20000u;          // a long stall is not a cycle debt
  const uint32_t n = dt * PACE_CYC_PER_US_Q16 + s_pace_frac;
  s_pace_frac = (uint16_t)n;
  s_owed += (int32_t)(n >> 16);
  if (s_owed > PACE_MAX_OWED) s_owed = PACE_MAX_OWED;
  if (s_owed <= 0) return 0;
  return (s_owed > EMU_BATCH) ? EMU_BATCH : (int16_t)s_owed;
}

static uint32_t s_int_us   = 0;
static uint8_t  s_int_lvl  = 0;

// /INT is a square wave at the manual's 1.9 ms period, so a half-period here.
static const uint16_t INT_HALF_US = 950;

void emu_setup() {
  fastpins_init();                 // after hw::Init has set pin directions? no:
                                   // caching regs/masks needs no pinMode order
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  hw::Init();

  // The panel decode table is DATA (controls.h), and this firmware reads four
  // fields out of it: the status-group bit indices for RUN / TAP / FILL IN and,
  // above all, the TEMPO CLOCK. Only superos_setup() used to load it, so in
  // emulator mode g_panel_map stayed zero-initialised — clock_bit = 0 — and the
  // clock sampler read PA0, the START/STOP flip-flop, instead of PA3.
  //
  // The machine then saw exactly one clock edge per RUN press (low to high, once)
  // and played a ONE-STEP sequence that never advanced again. Load the table here
  // too: SetDefaults() first, then a calibrated table if the store has one, so
  // this tracks calibration instead of hard-coding bit 3.
  panel_map_boot_load();
  panel_idx_init();                // resolve the panel-map lookups once
#ifdef D650_PROBE_AUX
  // Only ever read back through the EE_TLM_RST block, so it goes with it.
  { extern uint8_t g_boot_count; g_boot_count++; }
#endif

  s_have_img = img_load(s_img);
#ifdef D650_IMG_EMBEDDED
  if (!s_have_img) {                      // fall back to the compiled-in image
    memcpy_P(s_img, D650_IMG_EMBED, D650_IMG_SIZE);
    s_have_img = true;
  }
#endif

  d650_drivers drv = { hook_port, nullptr, emu_read_clock };
  d650_init(&H, s_img, &drv);
  patt_load();

  s_imgrx.Bind(s_img);
  s_imgrx.Reset();
  s_int_us = micros();
  s_pace_us = s_int_us;
  s_pace_frac = 0;
  s_owed = 0;
}

// No image stored: hold the machine quiet and blink step LED 1 so it is obvious
// what is missing, while the MIDI poll waits for an upload.
static void imgwait() {
  midi_poll();
  const bool on = (millis() >> 9) & 1;
  PORTF = on ? 0x1E : 0x0F;
}

#ifdef UCOM4_NATIVE
// Time-sliced loop for the translated core. The old shape ran ONE 24-cycle
// batch per pass with every service around it, so the pass period (measured
// 1.2-1.6 ms) was LONGER than the 950 us /INT half-period: interrupt edges
// were delivered late (the display multiplex ran slow), and the mainline got
// the scraps of the batch budget (~4k of the 86k cycles/s it needs).
//
// The per-pass services: /INT on a schedule, the tempo-clock stretcher, the
// lamp integrator and the panel sampler, run between small batches of emulated
// cycles.
//
// These were tried as a callback from inside the translated core, to avoid
// leaving and re-entering it every batch (650 clocks, measured). It backfired:
// a call on every hot back edge made gcc spill around all of them and the
// device went from 97.7k to 44.8k emulated cycles/s. Out here they cost only
// themselves.

// How early a scheduled /INT edge may be delivered. A pass costs about 850 us
// and the half-period is 950, so with an exact test a half-period whose instant
// fell just after a pass boundary waited a whole extra pass: the delivered rate
// sat at 515/s against the real 526 and the shortfall showed up as a slow
// display and panel cadence. s_int_us still advances by exactly INT_HALF_US, so
// allowing an edge a little early jitters its instant without letting the
// long-run rate drift.
static const uint16_t INT_EARLY_US = 300;

static void emu_services(uint32_t now) {
  if ((int32_t)((now + INT_EARLY_US) - (s_int_us + INT_HALF_US)) >= 0) {
    s_int_us += INT_HALF_US;
    if ((int32_t)(now - s_int_us) > (int32_t)(4u * INT_HALF_US))
      s_int_us = now;               // fell far behind: resync, do not burst
    s_int_lvl ^= 1;
    d650_clock(&H, s_int_lvl);
  }
  { SVC_T0(); clock_sample(now); clock_service(now); SVC_ADD(s_us_clk); }
  if ((uint32_t)(now - s_lamp_us) >= 200) { // integrator gains nothing finer
    SVC_T0(); lamp_service(now); SVC_ADD(s_us_lamp);
  }
  // Its own micros(), not `now`: clock_sample parked the port and d650_run has
  // executed a whole batch since `now` was taken, and poll_matrix has to time
  // its settle windows from the present, not from the top of the pass.
  { SVC_T0(); poll_matrix(micros()); SVC_ADD(s_us_pm); }
#ifndef SUPEROS_NO_KEYQUEUE
  key_direct_service();              // claim-131: write-mode gate upkeep
#endif
}

// Here the heavy once-per-pass services run first, then emulation runs for a
// ~1.9 ms slice with only the time-critical hooks inside: /INT on a
// SCHEDULE (s_int_us += half, not = now, so cadence holds even when a pass
// runs long; clamped if we fall far behind), the clock stretcher, the lamp
// integrator and the matrix sampler between small batches.
void emu_loop() {
  if (!s_have_img) { imgwait(); return; }

  { SVC_T0();
    menu_gesture();            // CLEAR held + TAP -> config menu, in place
    midi_poll();
    patt_service();
    SVC_ADD(s_us_outer); }
#ifdef UCOM4_PC_PROFILE
  pc_profile_service();
#endif

#if defined(D650_CYCLE_PROBE) && !defined(D650_NO_MICROBENCH)
  // One-shot pure-emulation benchmark, before the loop's services can blur it.
  // Publishes to EE_TLM_BENCH_ADDR: ms for 100k cycles idle-only, then ms for
  // 100k cycles with /INT toggled every 108 cycles (the real cadence's shape).
  // It was at 0x830, INSIDE the PC profile's own 0x820..0x83F histogram, and
  // the two are enabled together on env:combined-4mhz-profile and
  // env:combined-4mhz-hle-profile: two live telemetry blocks writing the same
  // bytes in one build.
  {
    static bool bdone = false;
    if (!bdone) {
      bdone = true;
      uint8_t r[EE_TLM_BENCH_BYTES] = { 0xB7, 0,0, 0,0, 0,0, 0,0, 0,0, 0xA5 };
      // [1..2] ms for 100k cyc at batch 24   [3..4] ms at batch 240
      // [5..6] native cycles seen in a 40k-cyc window  [7..8] native calls
      // [9..10] ms for 100k cyc with io hooks bypassed (write-only stub)
      wdt_reset();
      uint32_t t0 = millis();
      for (uint16_t i = 0; i < 4166; ++i) { d650_run(&H, 24); if (!(i & 511)) wdt_reset(); }
      uint16_t ms = (uint16_t)(millis() - t0);
      r[1] = (uint8_t)ms; r[2] = (uint8_t)(ms >> 8);
      wdt_reset();
      t0 = millis();
      for (uint16_t i = 0; i < 416; ++i) { d650_run(&H, 240); if (!(i & 63)) wdt_reset(); }
      ms = (uint16_t)(millis() - t0);
      r[3] = (uint8_t)ms; r[4] = (uint8_t)(ms >> 8);
      wdt_reset();
      ucom4_nat_cycles = 0; ucom4_nat_calls = 0;
      for (uint16_t i = 0; i < 1666; ++i) { d650_run(&H, 24); if (!(i & 511)) wdt_reset(); }
      r[5] = (uint8_t)ucom4_nat_cycles; r[6] = (uint8_t)(ucom4_nat_cycles >> 8);
      r[7] = (uint8_t)ucom4_nat_calls;  r[8] = (uint8_t)(ucom4_nat_calls >> 8);
      {
        // io bypass: null the write hook so hook_port never runs; reads keep
        // working (H.in). Restores after. Bench-only state surgery.
        void (*saved)(void *, int, uint8_t) = D650_CPU(&H)->io.write_port;
        D650_CPU(&H)->io.write_port = 0;
        wdt_reset();
        t0 = millis();
        for (uint16_t i = 0; i < 4166; ++i) { d650_run(&H, 24); if (!(i & 511)) wdt_reset(); }
        ms = (uint16_t)(millis() - t0);
        D650_CPU(&H)->io.write_port = saved;
        r[9] = (uint8_t)ms; r[10] = (uint8_t)(ms >> 8);
      }
      d650_st_write_block(r, (uint8_t *)EE_TLM_BENCH_ADDR, sizeof r);
      {
        // per-call cost, isolated: 20k cycles as 20k single-cycle calls
        uint8_t q[EE_TLM_BENCH_Q_BYTES] = { 0xB8, 0, 0, 0xA5 };
        wdt_reset();
        uint32_t q0 = millis();
        for (uint16_t i = 0; i < 20000; ++i) { d650_run(&H, 1); if (!(i & 1023)) wdt_reset(); }
        uint16_t qms = (uint16_t)(millis() - q0);
        q[1] = (uint8_t)qms; q[2] = (uint8_t)(qms >> 8);
        d650_st_write_block(q, (uint8_t *)EE_TLM_BENCH_Q_ADDR, sizeof q);
        // pure entry/exit: budget 0, the while never runs
        uint8_t z[EE_TLM_BENCH_Z_BYTES] = { 0xB9, 0, 0, 0xA5 };
        wdt_reset();
        q0 = millis();
        for (uint16_t i = 0; i < 20000; ++i) { d650_run(&H, 0); if (!(i & 1023)) wdt_reset(); }
        qms = (uint16_t)(millis() - q0);
        z[1] = (uint8_t)qms; z[2] = (uint8_t)(qms >> 8);
        d650_st_write_block(z, (uint8_t *)EE_TLM_BENCH_Z_ADDR, sizeof z);
      }
    }
  }
#endif
  // One micros() a pass. It is not a cheap call on a divided clock -- the
  // Arduino core reads the timer under cli(), folds in the overflow counter and
  // scales by 16 -- and this loop runs ~1100 times a second, so the second read
  // that used to sit in the while condition was pure overhead. The slice test
  // uses the same `now` on the next time round.
  const uint32_t slice_end = micros() + 2 * INT_HALF_US;
  uint32_t now = micros();
  do {
    emu_services(now);
    const int16_t batch = cycles_due(now);
#ifdef D650_CYCLE_PROBE
    // Peak backlog. s_owed is the cycle debt the emulated CPU has run up
    // against real time: it sits near one batch while the emulator is keeping
    // pace and climbs toward the clamp as soon as it cannot. This is the honest
    // saturation measure -- cycles/s alone cannot tell "exactly keeping up"
    // from "capped by the pacer and falling behind".
    if (!s_probe_wrote && s_owed > (int32_t)s_owed_peak) s_owed_peak = (uint16_t)s_owed;
#endif
    if (batch > 0) {
#ifdef D650_CYCLE_PROBE
#ifdef D650_SVC_PROBE
      const uint32_t r0 = micros();      // two more micros() a pass: bench only
      const uint32_t ran = d650_run(&H, (uint32_t)batch);
      { const uint16_t d_ = (uint16_t)(micros() - r0);
        if (d_ > s_us_run) s_us_run = d_; }
#else
      const uint32_t ran = d650_run(&H, (uint32_t)batch);
#endif
      s_owed -= (int32_t)ran;
      cycle_probe(ran);
#else
      s_owed -= (int32_t)d650_run(&H, (uint32_t)batch);
#endif
    }
    {
      const uint32_t prev = now;
      now = micros();
#ifdef D650_CYCLE_PROBE
      // Longest wall gap between two passes. The emulated CPU is stopped for
      // all of it, so this is the length of any hitch the machine can produce.
      const uint16_t gap = (uint16_t)(now - prev);
      if (s_probe_wrote) s_probe_wrote = false;    // the probe's own write
      else {
        if (gap > s_gap_peak) s_gap_peak = gap;
        if (gap > 2000) s_hitches++;   // how OFTEN, not just how bad
      }
#else
      (void)prev;
#endif
    }
  } while ((int32_t)(now - slice_end) < 0);
}

#else  // interpreter-era loop shape
void emu_loop() {
  if (!s_have_img) { imgwait(); return; }

  const uint32_t now = micros();

  // /INT edges at the fixed 1.9 ms machine cadence.
  if ((uint32_t)(now - s_int_us) >= INT_HALF_US) {
    s_int_us  = now;
    s_int_lvl ^= 1;
    d650_clock(&H, s_int_lvl);
  }

  // Clock: sample the physical line, then advance the stretcher. Both run
  // between interpreter batches, never inside one.
  clock_sample(now);
  clock_service(now);

  // Panel lamps: integrate the interval that just ended before the next batch
  // dirties it. Also between batches, so "no RAM access in this interval" is a
  // property of a whole batch rather than of a point inside one.
  lamp_service(now);

  // Let the interpreter run. The batch is sized so the loop still services
  // /INT on time: at 16 MHz a ucom4 machine cycle is ~10 us of emulated time,
  // and this budget lands well inside the half-period above.
#ifdef UCOM4_PC_PROFILE
  pc_profile_service();
#endif
#ifdef D650_CYCLE_PROBE
  cycle_probe(d650_run(&H, 24));
#else
  d650_run(&H, 24);
#endif

  // Passive: called every pass, samples only inside the CPU's own column dwell.
  // Its own micros(), not `now`: clock_sample() and d650_run() have both run
  // since then, and the select-park check inside compares against a timestamp
  // taken by clock_sample THIS pass. Handing it the stale `now` would make that
  // subtraction wrap and the check would never hold.
  poll_matrix(micros());

  menu_gesture();            // CLEAR held + TAP -> config menu, in place

  midi_poll();
  patt_service();
}
#endif  // UCOM4_NATIVE loop shape

#endif  // SUPEROS_COMBINED
