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

#include "../pins.h"
#include "../hw.h"
#include "../controls.h"
#include "../combined.h"
#include "emu/d650_host.h"
#include "image_store.h"
#ifdef D650_IMG_EMBEDDED
#include "image_embed.h"          // generated locally; gitignored
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

static void write_nib(const uint8_t *pins, uint8_t v) {
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(pins[i], (v >> i) & 1 ? HIGH : LOW);
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
  write_nib(PE_PINS, v);
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
    case UCOM4_PORTD: write_nib(PD_PINS, v); break;
    case UCOM4_PORTE: s_pe_raw = v; write_pe(); break;
    case UCOM4_PORTF: write_nib(PF_PINS, v); break;
    // PORTC is the µPD444C data bus and is deliberately NOT mirrored. The RAM is
    // emulated in SRAM; the physical chips are still fitted and drive that bus,
    // so writing it is a bus fight — and with the SW1b mod it also stamps on the
    // pin 30 -> pin 2 path. The latch still lives in the host for the emulated
    // read/write cycle; it just never reaches a pin.
    case UCOM4_PORTC: break;
    case UCOM4_PORTG:                                 // step-LED rows
      s_pg = (uint8_t)(v & 0x0F);
      PORTF = (uint8_t)((s_pg << 4) | (s_ph & 0x0F));
      break;
    case UCOM4_PORTH:                                 // scan selects
      s_ph = (uint8_t)(v & 0x0F);
      PORTF = (uint8_t)((s_pg << 4) | s_ph);
      break;
    case UCOM4_PORTI: {
      const bool ce   = ((v >> 1) & 1) != 0;              // RAM CE
      const bool trig = ((v >> 2) & 1) != 0;              // COMMON TRIG
      if (ce) s_ce_seen = true;                           // this interval is dirty
      digitalWrite(PI1_PIN, ce ? HIGH : LOW);
      if (trig) {
        // Instrument data on the pins BEFORE the trigger edge: the lamp filter
        // may be holding PE0/PE1 (CP and RS) at a lamp level.
        s_trig_us = micros();
        if (!s_trig_on) { s_trig_on = true; write_pe(); }
      }
      digitalWrite(PI2_PIN, trig ? HIGH : LOW);
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
static const uint32_t EMU_SLOWDOWN      = 16000000UL / F_CPU;
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
static uint8_t  s_midi_run     = 0;   // MIDI transport, OR'd into the RUN status
static uint32_t s_last_f8_ms   = 0;   // MIDI clock liveness window

static inline bool midi_clock_live() { return (millis() - s_last_f8_ms) < 300; }
static inline void clk_queue_edge()  { if (s_clk_pending < 2) s_clk_pending++; }

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
  // The park drops the CPU's select for those 15 us, so the key lines start
  // settling through their 15 K pull-downs all over again. poll_matrix times its
  // settle from the CPU's select change, which knows nothing about this, so
  // publish the restore instant and let it wait that out too. Harmless at the old
  // 700 us gap (a park rarely landed inside a ~475 us dwell); at 250 us it is
  // load-bearing.
  s_clk_park_us = micros();
  const uint8_t raw = (uint8_t)((pa >> g_panel_map.clock_bit) & 1);
  if (raw && !s_clk_raw_prev && !midi_clock_live()) clk_queue_edge();
  s_clk_raw_prev = raw;
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

static uint8_t  s_pm_ph    = 0xFF;   // select latch this dwell belongs to
static uint32_t s_pm_since = 0;      // when the CPU last changed it
static bool     s_pm_taken = false;  // one sample per dwell, or the shift
                                     // register fills with duplicates and the
                                     // debounce stops debouncing

// ---------------------------------------------------------------------------
// CLEAR key: a double-press goes back to SuperOS
// ---------------------------------------------------------------------------
// Nothing else on the panel can leave the emulator. The stock program has no
// config menu and the µPD650C-085 has no idea a second firmware exists, so
// before this the only ways out were SysEx 0x4D and the STEP 1 + STEP 16
// power-on escape. This is the mirror of the config menu's step 9 going the
// other way, on the same gesture SuperOS uses to open its menu.
//
// THE KEY IS WITHHELD FROM THE EMULATED CPU UNTIL THE GESTURE RESOLVES. Stock
// CLEAR is destructive — PATTERN CLEAR erases the selected pattern, COMPOSE
// erases the rhythm track, the write modes commit PRE SCALE — so handing the
// CPU the first press and switching on the second would wipe a pattern on the
// way out the door. Instead:
//
//   - a press held past the window PASSES THROUGH, so holding CLEAR (the STEP
//     NUMBER chord, and everything else that wants it down) still works, just
//     EMU_CLR_DTAP_MS late;
//   - a short press with no second one is REPLAYED to the CPU as a clean press,
//     so a stock CLEAR tap still lands, deferred by the same window;
//   - a second press inside the window reboots into SuperOS, and the CPU never
//     saw either press.
//
// The replay is fed as a level for EMU_CLR_REPLAY_MS. The CPU samples its
// matrix on the 1.9 ms /INT cadence and debounces on top of that, so the press
// has to outlast both; 120 ms is a comfortable multiple and still well short of
// anything the program would read as a hold.
static const uint16_t EMU_CLR_DTAP_MS   = 600;   // matches the SuperOS gesture
static const uint16_t EMU_CLR_REPLAY_MS = 120;

enum : uint8_t { CLR_IDLE = 0, CLR_PEND, CLR_WAIT, CLR_REPLAY, CLR_THRU };
static uint8_t  s_clr_st   = CLR_IDLE;
static uint32_t s_clr_ms   = 0;
static uint8_t  s_clr_real = 0;   // debounced physical level
static uint8_t  s_clr_cpu  = 0;   // level the emulated CPU is given

// Where CLEAR sits in the snapshot: PH0 column, PA bit clear_bit (controls.h).
static inline uint8_t clr_slot() {
  return (uint8_t)(16 + (g_panel_map.clear_bit & 3));
}

static void commit_snapshot() {
  for (uint8_t i = 0; i < D650_IN_COUNT; ++i) {
    if (i == D650_IN_CLOCK) continue;      // served live by emu_read_clock
    const uint8_t v = (uint8_t)(s_db[i] & 0x07);
    if (v == 0x07) H.in[i] = 1;
    else if (v == 0x00) H.in[i] = 0;
  }
  // A MIDI Start/Continue runs the machine even with the panel flip-flop idle:
  // the CPU reads RUN as a level, so the transport simply ORs into it.
  if (s_midi_run) H.in[D650_IN_RUN] = 1;
  // CLEAR is gated: debounce the physical level into s_clr_real for the gesture
  // machine, and hand the CPU whatever that decides. Debounced straight off
  // s_db rather than read back out of H.in — H.in[ci] holds the GATED level, and
  // the loop above leaves an entry alone while its shift register is mixed, so
  // reading it back would feed the gate its own output through every bounce.
  const uint8_t ci = clr_slot();
  const uint8_t cv = (uint8_t)(s_db[ci] & 0x07);
  if (cv == 0x07)      s_clr_real = 1;
  else if (cv == 0x00) s_clr_real = 0;
  H.in[ci] = s_clr_cpu;
}

static void poll_matrix(uint32_t now_us) {
  // s_ph, not PORTF: clock_sample() parks the real port for 15 us at a time and
  // reading that back would look like a select change every sample.
  const uint8_t ph = (uint8_t)(s_ph & 0x0F);
  if (ph != s_pm_ph) { s_pm_ph = ph; s_pm_since = now_us; s_pm_taken = false; return; }
  if (s_pm_taken) return;

  // Active-low at the CPU pin: it raises all four and clears the one it
  // wants. All high = the STATUS group is gated on instead.
  uint8_t row = 4;
  for (uint8_t s = 0; s < 4; ++s) if (!((ph >> s) & 1)) { row = s; break; }

  if (row >= 4) return;                 // STATUS: see status_sample()
  if ((uint32_t)(now_us - s_pm_since) < PM_KEY_SETTLE_US) return;
  if ((uint32_t)(now_us - s_clk_park_us) < PM_KEY_SETTLE_US) return;  // see there
  s_pm_taken = true;

  const uint8_t pins = PINB;
  const uint8_t pa   = (uint8_t)(pins & 0x0F);         // PA0-3
  const uint8_t pb   = (uint8_t)((pins >> 4) & 0x0F);  // the 16 step switches

  if (row < 4) {
    for (uint8_t j = 0; j < 4; ++j) {
      const uint8_t kb = (uint8_t)(row * 4 + j);            // step switches
      s_db[kb] = (uint8_t)((s_db[kb] << 1) | ((pb >> j) & 1));
      const uint8_t ka = (uint8_t)(16 + row * 4 + j);       // PA cells
      s_db[ka] = (uint8_t)((s_db[ka] << 1) | ((pa >> j) & 1));
    }
  }
  commit_snapshot();
}

// The STATUS group, sampled from clock_sample()'s parked window (see there).
// The clock slot itself is skipped: it is served live by emu_read_clock.
static void status_sample(uint8_t pa) {
  const uint8_t src[3] = { g_panel_map.run_bit, g_panel_map.tap_bit,
                           g_panel_map.fillin_bit };
  for (uint8_t j = 0; j < 3; ++j) {
    const uint8_t k = (uint8_t)(D650_IN_STATUS + j);
    s_db[k] = (uint8_t)((s_db[k] << 1) | ((pa >> src[j]) & 1));
  }
  commit_snapshot();
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

static void patt_service() {
  if (d650_dirty(&H) && !s_saving) {
    s_dirty_ms = millis();
    d650_clear_dirty(&H);
    s_saving   = true;
    s_save_off = 0;
  }
  if (!s_saving) return;
  if ((uint32_t)(millis() - s_dirty_ms) < 400) return;   // let edits settle
  d650_st_write_block(H.ext + s_save_off, EE_EMU_PATT + s_save_off, 64);
  s_save_off = (uint16_t)(s_save_off + 64);
  if (s_save_off >= D650_EXT_BYTES) s_saving = false;
}

// Finish the store synchronously, for the one moment there is no next loop pass
// to finish it in: the firmware switch. The chunked writer above can have up to
// 400 ms of settle plus a partial pass outstanding, and the reboot would take
// those edits with it. eeprom_update_block skips unchanged bytes, so this is
// cheap on a store that is mostly already written.
static void patt_flush() {
  if (!s_saving && !d650_dirty(&H)) return;
  d650_clear_dirty(&H);
  for (uint16_t i = 0; i < D650_EXT_BYTES; i += 64) {
    d650_st_write_block(H.ext + i, EE_EMU_PATT + i, 64);
    wdt_reset();
  }
  s_saving = false;
}

// The CLEAR gesture machine described above commit_snapshot(). Called once per
// loop pass; s_clr_real is refreshed by every snapshot commit.
static void clear_gesture(uint32_t now) {
  switch (s_clr_st) {
    case CLR_IDLE:
      s_clr_cpu = 0;
      if (s_clr_real) { s_clr_st = CLR_PEND; s_clr_ms = now; }
      break;

    case CLR_PEND:                          // down, still deciding what it is
      s_clr_cpu = 0;
      if (!s_clr_real) { s_clr_st = CLR_WAIT; s_clr_ms = now; }
      else if ((uint32_t)(now - s_clr_ms) >= EMU_CLR_DTAP_MS) s_clr_st = CLR_THRU;
      break;

    case CLR_WAIT:                          // one short press banked
      s_clr_cpu = 0;
      if (s_clr_real) {
        patt_flush();
        combined_switch_firmware(FW_SUPEROS);          // does not return
      } else if ((uint32_t)(now - s_clr_ms) >= EMU_CLR_DTAP_MS) {
        s_clr_st = CLR_REPLAY; s_clr_ms = now;
      }
      break;

    case CLR_REPLAY:                        // hand the CPU the press it missed
      s_clr_cpu = 1;
      if ((uint32_t)(now - s_clr_ms) >= EMU_CLR_REPLAY_MS) {
        s_clr_cpu = 0;
        s_clr_st  = CLR_IDLE;               // resyncs on the next pass if held
      }
      break;

    case CLR_THRU:                          // held: the CPU gets it live
      s_clr_cpu = s_clr_real;
      if (!s_clr_real) s_clr_st = CLR_IDLE;
      break;

    default: s_clr_st = CLR_IDLE; break;
  }
}

// ---------------------------------------------------------------------------
// MIDI: firmware switch and program-image upload
// ---------------------------------------------------------------------------
// The emulator understands exactly two things over MIDI — the 0x4D firmware
// switch that gets you back to SuperOS, and a program-image upload. Everything
// else is ignored; the emulated machine has no MIDI of its own.
static uint8_t s_sx_state = 0;   // 0 idle, 1 saw F0, 2 saw 7D, 3 in 0x4D

static void midi_poll() {
  while (Serial1.available() > 0) {
    const uint8_t b = (uint8_t)Serial1.read();

    // System Real-Time can interleave ANYWHERE, including mid-SysEx, so it is
    // handled first and never reaches the parsers below — ImgRx aborts on any
    // status byte, and a clock tick arriving during an image upload would
    // otherwise kill the transfer.
    if (b >= 0xF8) {
      switch (b) {
        case 0xF8: s_last_f8_ms = millis(); clk_queue_edge(); break;
        case 0xFA:                                  // Start
        case 0xFB: s_midi_run = 1; break;           // Continue
        case 0xFC: s_midi_run = 0; break;           // Stop
        default: break;
      }
      continue;
    }

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
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------
static uint32_t s_int_us   = 0;
static uint8_t  s_int_lvl  = 0;

// /INT is a square wave at the manual's 1.9 ms period, so a half-period here.
static const uint16_t INT_HALF_US = 950;

void emu_setup() {
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
}

// No image stored: hold the machine quiet and blink step LED 1 so it is obvious
// what is missing, while the MIDI poll waits for an upload.
static void imgwait() {
  midi_poll();
  const bool on = (millis() >> 9) & 1;
  PORTF = on ? 0x1E : 0x0F;
}

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
  d650_run(&H, 24);

  // Passive: called every pass, samples only inside the CPU's own column dwell.
  // Its own micros(), not `now`: clock_sample() and d650_run() have both run
  // since then, and the select-park check inside compares against a timestamp
  // taken by clock_sample THIS pass. Handing it the stale `now` would make that
  // subtraction wrap and the check would never hold.
  poll_matrix(micros());

  clear_gesture(millis());   // CLEAR double-press -> back to SuperOS

  midi_poll();
  patt_service();
}

#endif  // SUPEROS_COMBINED
