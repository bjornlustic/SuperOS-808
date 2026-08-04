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

static void hook_port(void *, int port, uint8_t v) {
  switch (port) {
    case UCOM4_PORTD: write_nib(PD_PINS, v); break;
    case UCOM4_PORTE: write_nib(PE_PINS, v); break;
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
    case UCOM4_PORTI:
      digitalWrite(PI1_PIN, (v >> 1) & 1 ? HIGH : LOW);   // RAM CE
      digitalWrite(PI2_PIN, (v >> 2) & 1 ? HIGH : LOW);   // COMMON TRIG
      break;
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
        if (b <= 1) combined_switch_firmware(b);  // does not return
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

  midi_poll();
  patt_service();
}

#endif  // SUPEROS_COMBINED
