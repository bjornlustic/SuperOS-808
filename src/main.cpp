// SuperOS-808 — TR-808 firmware for the AT90USB1286 CPU replacement
// (any AT90USB1286 board in the µPD650C socket; Teensy++ 2.0 class)
//
// Loop structure: one ~1 ms pass that scans the panel matrix
// while multiplexing the step-LED frame, debounces, services the trigger pulse,
// drains MIDI, runs the transport and clock, dispatches to a mode/tool handler,
// then builds the next LED frame.
//
// Everything above that is the TR-808's own operating model, from the owner's
// manual:
//   - 16 STEP buttons x A/B = 32 one-measure patterns; buttons 1-12 are BASIC
//     RHYTHM, 13-16 are INTRO/FILL IN
//   - a measure is the 1st PART then the 2nd PART concatenated, with independent
//     step counts; A and B share the layout
//   - CLEAR commits PRE SCALE, and CLEAR + step sets that part's step count
//   - TAP inserts intros/fills in the play modes and writes steps in real time
//     in the write modes
//   - COMPOSE records rhythm-track measures live; tracks store SLOTS, so the
//     variation switch applies at playback
//
// The one thing that has no stock equivalent is the SuperOS layer, and the 808
// has exactly one free momentary button to hang it off. See
// docs/CONTROL_SCHEME.md for the CLEAR gesture design.

#include <Arduino.h>
#include <avr/interrupt.h>

#include "pins.h"
#include "hw.h"
#include "controls.h"
#include "pattern.h"
#include "engine.h"
#include "settings.h"
#include "flash_persist.h"
#include "midi_api.h"
#ifdef SUPEROS_COMBINED
#include "combined.h"
#endif

Settings g_settings;
PanelMap g_panel_map;

// ---------------------------------------------------------------------------
// MIDI out
// ---------------------------------------------------------------------------
// Notes go through midi.cpp's TX queue so they can never split a SysEx dump in
// two. Realtime bytes write directly: the MIDI spec lets them interleave
// anywhere, and clock must not wait behind a queued dump.
static inline bool midi_out_live() { return g_settings.out_mode == OUT_MODE_OUT; }
static inline void midiNoteOn(uint8_t n, uint8_t v) {
  if (!midi_out_live()) return;
  const uint8_t m[3] = { 0x90, (uint8_t)(n & 0x7F), (uint8_t)(v & 0x7F) };
  midi_tx_msg(m, 3);
}
static inline void midiNoteOff(uint8_t n) {
  if (!midi_out_live()) return;
  const uint8_t m[3] = { 0x80, (uint8_t)(n & 0x7F), 0 };
  midi_tx_msg(m, 3);
}
// Non-blocking: if the UART FIFO is momentarily full (e.g. a step firing a stack
// of voices), drop the byte rather than block — a stalled loop would let the RX
// FIFO overflow and slip the sequencer off the incoming clock.
static inline void midiRT(uint8_t b) {
  if (!midi_out_live()) return;
  if (Serial1.availableForWrite() > 0) Serial1.write(b);
}

// ---------------------------------------------------------------------------
// Debounced inputs (3-sample shift register, sampled once per ~1 ms loop pass)
// ---------------------------------------------------------------------------
struct PinState {
  uint8_t state = 0;
  void push(bool high) { state = (uint8_t)((state << 1) | (high ? 1 : 0)); }
  bool rising()  const { return (state & 0x07) == 0x03; }
  bool falling() const { return (state & 0x07) == 0x04; }
  bool held()    const { return (state & 0x07) != 0; }

  // Edge on the FIRST sample that reads high (pattern 0,0,1) instead of waiting
  // for a second confirming high (0,1,1). Same required-stable-low qualifier, one
  // whole loop pass sooner.
  //
  // TEMPO / DIN CLOCK ONLY. The status group is read once at the top of the pass,
  // so rising() cannot report an edge until the NEXT pass, and the sequencer steps
  // a fixed ~1 ms late on top of the up-to-1 ms sampling latency. Measured against
  // Ableton as ~1.5 ms of lag, which is this.
  //
  // Not used for buttons: for them the second sample is cheap insurance and 1 ms
  // is inaudible. It is also not the guard that matters here. The known way to get
  // a phantom clock edge on this machine is the COMMON-TRIG pulse coupling into
  // the gated PA status sense, and that is handled separately by holding the
  // debouncer while eng.PulseActive() (see the push below). The confirming sample
  // was belt-and-braces on top of that, paid for on every single tick.
  bool rising_now() const { return (state & 0x07) == 0x01; }
};

static Engine   eng;
static Controls panel;

static PinState stepB[NUM_STEP_BTNS];
static PinState clearB, tapB, pedalFillB, runB, clkB;

// Rotary state. The decode is a table lookup that returns the previous value on
// no match (controls.h), so these both hold the value and feed the lookup.
static uint8_t modeVal = MODE_1ST_PART;
static uint8_t instVal = INST_BD;
static uint8_t preVal  = 2;
static uint8_t varVal  = VAR_A;
static uint8_t fillVal = FILL_MANUAL;

static uint8_t  disp_section = 0;  // 0..3: which 16-step section is shown/edited
static int8_t   anchor       = -1; // chain anchor (step index) in MANUAL PLAY
static uint8_t  prev_mode    = MODE_1ST_PART;
static uint16_t prev_fired   = 0;  // last step's voices, for MIDI note-offs
static uint16_t frame        = 0;  // step-LED frame shown by ScanAndDisplay

// ---------------------------------------------------------------------------
// CLEAR gesture machine — the whole SuperOS control surface
// ---------------------------------------------------------------------------
//   tap (< TAP_MS, nothing chorded)   -> the stock CLEAR action for the mode,
//                                        or exit a latched tool
//   hold (>= HOLD_MS, alone)          -> tool map; a step press latches a tool
//   CLEAR + step before HOLD_MS       -> stock chorded action / tool step action
//   two taps within DTAP_MS           -> open/close the config menu
//   hold >= LONG_MS                   -> long action / menu confirm
//
// A tap can only be recognised on RELEASE, so the stock tap action fires on
// release. The one stock behaviour needing press-time response is chase-delete,
// which is a hold anyway and starts once HOLD_MS passes.
//
// THE GESTURE IS A PRESS BURST, NOT A PAIR OF INDEPENDENT TAPS. Short presses
// accumulate in `presses` and the burst is only dispatched once it closes, so a
// single tap is DEFERRED by CLEAR_DTAP_MS. Without that, the first press of a
// double-tap fires the single-tap action on its way to the menu — which in a
// write mode means committing the PRE SCALE dial into the pattern. Opening the
// config menu would silently change the pattern's step timing to wherever the
// dial happened to be resting. Measured on hardware: a pattern at 4 steps/beat
// jumped to 8 because the dial sat at position 4.
//
// The earlier form tracked one pending tap plus an "armed" flag and rebuilt the
// pair from scratch on every press, which left a hole: any double-press the
// window did not quite catch degenerated back into TWO single taps, and in a
// write mode both of them re-stamped the resting PRE SCALE dial. Counting the
// burst closes that — a second press cannot resurrect the first one's tap,
// whatever the timing — and the count is also why a triple-press is a menu
// toggle rather than a menu toggle plus a stray stock action.
//
// The window is measured from the last RELEASE to the next PRESS, so it is a
// gap, not a budget the second press has to fit inside as well. An earlier form
// timed release-to-release, which meant the gap plus the whole second press had
// to land inside the window and an ordinary double-tap read as two singles.
static const uint16_t CLEAR_TAP_MS  = 350;
static const uint16_t CLEAR_HOLD_MS = 350;
static const uint16_t CLEAR_DTAP_MS = 600;
static const uint16_t CLEAR_LONG_MS = 2000;

struct ClearGesture {
  uint32_t down_ms   = 0;
  uint32_t last_up   = 0;
  uint8_t  presses   = 0;       // short presses banked in the current burst
  bool     chorded   = false;
  bool     consumed  = false;
  bool     hold_seen = false;
  bool     long_seen = false;
  bool     tap       = false;
  bool     dtap      = false;

  void update(const PinState &b, uint32_t now) {
    tap = dtap = false;
    if (b.rising()) {
      down_ms = now; chorded = false; consumed = false;
      hold_seen = false; long_seen = false;
    }
    if (b.held()) {
      const uint32_t dt = now - down_ms;
      if (dt >= CLEAR_HOLD_MS) hold_seen = true;
      if (dt >= CLEAR_LONG_MS) long_seen = true;
    }
    if (b.falling()) {
      const uint32_t dt = now - down_ms;
      if (dt < CLEAR_TAP_MS && !chorded && !consumed) {
        last_up = now;
        // Two banked presses ARE the double-tap; fire it on this release rather
        // than waiting the window out, so the menu still opens the instant the
        // gesture is complete. Only the single tap has to be deferred.
        if (++presses >= 2) { dtap = true; presses = 0; }
      } else {
        // A hold, a chord, or a consumed press ends the burst outright, so a
        // tap-then-hold cannot leave a stale tap queued behind the tool map.
        presses = 0;
      }
    }
    // The burst closed with one press banked: that is a single tap.
    if (presses && !b.held() && (uint32_t)(now - last_up) >= CLEAR_DTAP_MS) {
      tap = true;
      presses = 0;
    }
  }
  bool map_open() const { return hold_seen; }
};

static ClearGesture clr;

// Tool IDs == step number (1..16). See docs/CONTROL_SCHEME.md.
enum {
  TOOL_NONE = 0,
  TOOL_LENGTH = 1, TOOL_PRESCALE = 2, TOOL_COPY = 3, TOOL_XFORM = 4,
  TOOL_ACCENT = 5, TOOL_PROB = 6, TOOL_RATCHET = 7, TOOL_POLY = 8,
  TOOL_MUTE = 9, TOOL_RESLICE = 10, TOOL_ARP = 11, TOOL_DIR = 12,
  TOOL_GEN = 13, TOOL_SAVE = 14, TOOL_SECTION = 15, TOOL_SWING = 16,
};
static constexpr uint16_t TOOLS_IMPLEMENTED = 0xFFFF;

static uint8_t  s_tool        = TOOL_NONE;
static bool     s_menu        = false;
static bool     s_menu_hold   = false;
static uint8_t  s_pre_ref     = 0xFF;
static uint8_t  s_prob_sel    = 0xFF;
static uint8_t  s_ratchet_sel = 0xFF;
static uint32_t s_clear_up_ms = 0;

// PRE SCALE dial value as of the last commit, 0xFF before the first scan. A
// bare CLEAR tap only stamps the dial into the pattern when it has MOVED since
// then — see the commit site at the bottom of loop().
static uint8_t  s_pre_committed = 0xFF;

#ifdef SUPEROS_COMBINED
static constexpr uint8_t EMU_STEP = 8;   // menu step 9: reboot into the emulator
#endif

static inline bool mode_is_write(uint8_t m) {
  return m == MODE_1ST_PART || m == MODE_2ND_PART;
}
static inline uint16_t led_bit(uint8_t n) { return (uint16_t)1 << n; }

// ---------------------------------------------------------------------------
// Tempo tracker for the stopped blink
// ---------------------------------------------------------------------------
// While STOPPED the tempo clock on the status line is a NARROW pulse train
// straight from the oscillator — the run flip-flop (IC2A/B) only stretches it
// into a square wave while running (service manual p.5, Fig. 8/9), far too
// narrow for the ~1 ms polled scan to catch. A pin-change interrupt grabs the
// pulses whenever the scan selects are idle and keeps a smoothed estimate, so
// the stopped blink tracks the TEMPO knob live.
//
// The run/stop flip-flops DIVIDE the oscillator as well as reshape it, so the
// stopped pulse rate is a multiple of the musical 24 PPQN rate the CPU sees
// while running. The running clock is the trusted musical reference, so the
// tracker keeps two estimates, measures their ratio at each stop, and advances
// the blink phase in HALF-ticks of the musical clock per stopped pulse.
static volatile uint32_t s_clk_last_us = 0;
static volatile uint32_t s_per_run     = 20833;  // 24-PPQN period (120 BPM default)
static volatile uint32_t s_per_stop    = 0;
static volatile uint8_t  s_half_tick   = 0;      // musical phase in half-ticks, mod 48
static volatile uint8_t  s_k_half      = 0;
static volatile uint8_t  s_stop_seen   = 0;
static volatile bool     s_clk_running = false;

static inline void half_tick_advance(uint16_t h) {
  s_half_tick = (uint8_t)((s_half_tick + h) % 48);
}

// Shared by the ISR and the polled path (call with interrupts off).
static void clk_track_edge(uint32_t now) {
  const uint32_t d = now - s_clk_last_us;

  if (s_clk_running) {
    const uint32_t est = s_per_run;
    if (d < est - (est >> 2)) return;       // duplicate sighting of one pulse
    s_clk_last_us = now;
    uint8_t n = (uint8_t)((d + (est >> 1)) / est);
    if (n < 1) n = 1;
    if (n > 24) n = 24;
    const uint32_t d1 = d / n;
    if (d1 >= 5000UL && d1 <= 150000UL) s_per_run = (est * 3 + d1) >> 2;
    half_tick_advance((uint16_t)(2 * n));
    return;
  }

  const uint32_t est = s_per_stop;
  const uint32_t min_gap = est ? est - (est >> 2) : 2500UL;
  if (d < min_gap) return;
  s_clk_last_us = now;

  uint8_t n = 1;
  if (!est) {
    if (d >= 2500UL && d <= 150000UL) s_per_stop = d;
  } else {
    n = (uint8_t)((d + (est >> 1)) / est);
    if (n < 1) n = 1;
    if (n > 24) n = 24;
    const uint32_t d1 = d / n;
    if (d1 >= 2500UL && d1 <= 150000UL) s_per_stop = (est * 3 + d1) >> 2;
  }

  if (s_k_half == 0) {
    uint32_t h = (2 * d + (s_per_run >> 1)) / s_per_run;
    if (h < 1) h = 1;
    if (h > 16) h = 16;
    half_tick_advance((uint16_t)h);
    if (s_per_stop && ++s_stop_seen >= 12) {
      uint32_t k = (2 * s_per_stop + (s_per_run >> 1)) / s_per_run;
      if (k < 1) k = 1;
      if (k > 8) k = 8;
      s_k_half = (uint8_t)k;
    }
  } else {
    half_tick_advance((uint16_t)(s_k_half * n));
  }
}

// The tempo clock arrives on one of the four PA status lines (AVR PB0-3). Which
// one is calibrated, so the ISR masks against the live map rather than a fixed
// bit; all four PCINTs vector here and the level test picks out the clock line.
#ifdef SUPEROS_COMBINED
void superos_pcint()                    // called from combined.cpp's PCINT0 ISR
#else
ISR(PCINT0_vect)
#endif
{
  if ((PORTF & 0x0F) != 0x0F) return;   // matrix scan in progress: PB != status
  if (!(PINB & (uint8_t)(1 << g_panel_map.clock_bit))) return;   // rising only
  if (eng.PulseActive()) return;        // COMMON-TRIG crosstalk on the status
                                        // sense: don't feed the tracker a
                                        // phantom edge either
  clk_track_edge(micros());
}

static bool tempo_blink() {
  uint8_t t;
  { const uint8_t sreg = SREG; cli(); t = s_half_tick; SREG = sreg; }
  return t < 24;
}

// --- external MIDI clock sync ----------------------------------------------
static const uint16_t MCLK_TIMEOUT_MS = 300;
static uint32_t s_last_mclk_ms = 0;
static bool     s_hw_run       = false;   // panel START/STOP flip-flop latch
static bool     s_want_run     = false;   // combined transport (panel OR MIDI)

// --- external-sync start alignment -------------------------------------------
// RUN and the tempo clock are debounced identically, but a sync master
// raises RUN on (or a hair after) a clock edge, and depending on where the two
// edges fall against the ~1 ms sampling grid the clock edge's debounced
// recognition can land one loop pass BEFORE the run's. That tick would be
// swallowed while still "stopped" and step 1 would fire a whole 24-PPQN pulse
// late. Remember when the last polled clock edge was seen; a start replays an
// edge this recent so step 1 stays on the master's downbeat.
static const uint32_t CLK_EDGE_GRACE_US = 3000;
static uint32_t s_clk_edge_us   = 0;
static bool     s_clk_edge_seen = false;

static void send_note_offs() {
  for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
    if (prev_fired & led_bit(i)) midiNoteOff(INSTRUMENT_NOTE[i]);
  prev_fired = 0;
}

// USB policy: builds not made for USB MIDI tear the USB engine down here.
// The Teensy core's usb_init() (run before setup()) otherwise leaves an
// unserviced USB controller running, and a live engine measurably corrupts the
// high-impedance panel matrix.
#ifndef SUPEROS_USB_MIDI
static void usb_shutdown_hw() {
  UDIEN  = 0;
  UDCON  = 1;                 // detach
  USBCON = (1 << FRZCLK);
  PLLCSR = 0;
}
#endif

#ifdef SUPEROS_COMBINED
// combined.cpp owns setup()/loop() and dispatches to whichever firmware the
// EEPROM selector picked, so ours are renamed out of the way.
#define setup superos_setup
#define loop  superos_loop
#endif

// Shared with the emulator firmware (see combined.h). Must run before anything
// reads the panel, in either firmware.
void panel_map_boot_load() {
  flash_persist_begin();
  load_panel_map(g_panel_map);
}

void setup() {
#ifndef SUPEROS_USB_MIDI
  usb_shutdown_hw();
#endif
  // Pull up the MIDI RX line so an unplugged jack can't leave the input floating
  // and self-clock the UART from matrix/EMI noise — spurious 0xF8 bytes would
  // drive the sequencer.
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  hw::Init();
  eng.Init();

  panel_map_boot_load();         // mount the store + decode table, before any read
  load_all(eng);
  load_settings(g_settings);

  eng.SetIndicators(eng.live_variation(), false);

  // Pin-change interrupt on all four PA status inputs (AVR PB0-3). The ISR
  // filters down to the calibrated clock line, so recalibrating the map does not
  // need a different mask here.
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
  PCICR  |= _BV(PCIE0);

  delay(150);
  // boot signature: three quick CC#119 pulses = sequencer firmware alive
  for (uint8_t i = 0; i < 3; ++i) {
    Serial1.write(0xB0); Serial1.write((uint8_t)0x77); Serial1.write((uint8_t)0x7F); delay(60);
    Serial1.write(0xB0); Serial1.write((uint8_t)0x77); Serial1.write((uint8_t)0x00); delay(60);
  }
}

// ---------------------------------------------------------------------------
// Mode handlers
// ---------------------------------------------------------------------------

// PATTERN WRITE, 1st PART / 2nd PART.
//
// Step buttons toggle the selected instrument's steps in the shown part. CLEAR
// held plus a step button sets THAT part's step count and commits the PRE SCALE
// dial — the panel's own "STEP NUMBER" legend, and the procedure on OM p.15
// ("Press the CLEAR button but do not let it up. While holding it down press the
// switch for Step #12"). A CLEAR tap with no step commits the PRE SCALE alone
// (OM p.13). TAP writes the step that is playing right now.
static void handle_pattern_write(uint8_t inst) {
  if (s_tool != TOOL_SECTION)
    disp_section = (modeVal == MODE_2ND_PART) ? 1 : 0;

  // A step press landing just after a CLEAR hold is almost always a late
  // tool-map press, not a step entry — swallow it.
  const bool grace = (uint32_t)(millis() - s_clear_up_ms) < 250;

  if (clearB.held()) {
    // STEP NUMBER: set this part's length, and commit the prescale with it.
    for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
      if (!stepB[b].rising()) continue;
      clr.chorded = true;
      eng.SetSectionLength(disp_section, (uint8_t)(b + 1));
      eng.CommitPrescale(preVal);
      s_pre_committed = preVal;    // the dial is now what the pattern holds
      midi_send_layout_update(eng.cur_pat);
    }
    return;                       // CLEAR held: no step editing this pass
  }

  for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
    if (!stepB[b].rising() || grace) continue;
    const uint8_t s = (uint8_t)(disp_section * NUM_STEP_BTNS + b);
    if (b >= eng.SectionLength(disp_section)) continue;   // outside this part
    eng.ToggleStep(inst, s);
    midi_send_step_update(eng.cur_pat, inst, s, eng.cur().step_get(inst, s));
  }

  if (eng.running) {
    // Chase-delete: while CLEAR is HELD past the hold threshold, erase this
    // instrument's steps as the chase light passes them.
    if (clr.hold_seen && clearB.held() && eng.step_advanced && eng.step >= 0) {
      const uint8_t cs = (uint8_t)eng.step;
      if (eng.cur().step_get(inst, cs)) {
        eng.ClearStep(inst, cs);
        midi_send_step_update(eng.cur_pat, inst, cs, false);
      }
    }
    // TAP programming (OM p.13): record the step that is playing right now.
    if (tapB.rising() && eng.step >= 0) {
      eng.SetStep(inst, (uint8_t)eng.step);
      midi_send_step_update(eng.cur_pat, inst, (uint8_t)eng.step, true);
    }
  }
}

// MANUAL PLAY: play the 32 patterns, with intro/fill switching.
// Step buttons 1-12 select the BASIC RHYTHM, 13-16 arm an INTRO/FILL IN, TAP
// inserts it. Holding two basic buttons builds a chain (a SuperOS addition;
// the stock machine has no chaining outside a rhythm track).
static void handle_manual_play() {
  for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s) {
    if (!stepB[s].rising()) continue;

    if (slot_is_fill(s)) { eng.SelectFillSlot(s); anchor = -1; continue; }

    int8_t other = -1;
    for (uint8_t a = 0; a < NUM_BASIC; ++a)
      if (a != s && stepB[a].held()) { other = (int8_t)a; break; }

    if (other >= 0) {
      const uint8_t lo = other < (int8_t)s ? (uint8_t)other : s;
      const uint8_t hi = other < (int8_t)s ? s : (uint8_t)other;
      uint8_t slots[CHAIN_MAX], n = 0;
      for (uint8_t p = lo; p <= hi && n < CHAIN_MAX; ++p) slots[n++] = p;
      eng.SelectChain(slots, n);
      anchor = other;
    } else {
      eng.SelectSlot(s);
      anchor = (int8_t)s;
    }
  }
  if (anchor >= 0 && !stepB[anchor].held()) anchor = -1;

  if (tapB.rising()) eng.TapFill();
}

// PLAY: rhythm-track playback. INSTRUMENT/TRACK picks track 1-12.
static void handle_track_play(uint8_t inst) {
  if (!eng.running) eng.cur_track = inst % NUM_TRACKS;
  if (tapB.rising()) eng.TapFill();
}

// COMPOSE: record a rhythm track in real time. The operator clears the track
// with CLEAR, presses START, switches patterns with the STEP buttons as it runs,
// and presses STOP on the measure that should be last (OM p.19).
static void handle_compose(uint8_t inst) {
  if (!eng.running) {
    eng.cur_track = inst % NUM_TRACKS;
    if (clr.tap) { eng.TrackClear(); clr.tap = false; }
  }
  for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s) {
    if (!stepB[s].rising()) continue;
    if (slot_is_fill(s)) eng.SelectFillSlot(s);
    else                 eng.SelectSlot(s);
  }
  if (tapB.rising()) eng.TapFill();
}

// PATTERN CLEAR slot-occupancy cache, for the mode's LED frame.
//
// Reading one pattern through the block store costs over a millisecond (a full
// page read plus a bitwise CRC16 over ~240 bytes), and the frame needs all 16
// slots. Doing that inline in build_frame() every pass stretched the ~1 ms loop
// to ~20 ms: the LED multiplex strobed visibly and most button presses fell
// between debounce samples, so the mode read as dead. The mask is rebuilt only
// when it can be stale — mode entry, variation change, after a clear, and once
// a second while stopped (to track editor pushes) — never per pass.
static uint16_t s_occ_mask = 0;
static uint8_t  s_occ_var  = 0xFF;   // variation the mask was built for; 0xFF = stale
static uint32_t s_occ_ms   = 0;

static void occ_rebuild(uint8_t var) {
  uint16_t m = 0;
  for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
    Pattern t;
    eng.ReadPattern(pat_index(var, b), t);
    if (!t.Empty()) m |= led_bit(b);
  }
  s_occ_mask = m;
  s_occ_var  = var;
  s_occ_ms   = millis();
}

// PATTERN CLEAR: pick a slot with a step button, then press CLEAR to erase it.
// With BASIC VARIATION at AB both memories of the slot go at once (OM p.16).
static void handle_pattern_clear_mode() {
  for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
    if (stepB[b].rising()) eng.SetEditSlot(b);
  if (clr.tap) {
    clr.tap = false;
    eng.ClearPatternSlot(eng.sel_slot, varVal);
    midi_send_pattern_dump(pat_index(0, eng.sel_slot));
    midi_send_pattern_dump(pat_index(1, eng.sel_slot));
    if (!eng.running) save_dirty(eng);
    s_occ_var = 0xFF;                 // the mask just changed: rebuild it
  }
}

// ---------------------------------------------------------------------------
// Tool layer
// ---------------------------------------------------------------------------
// TAP inside a tool is both a chord modifier (TAP + step) and an action of its
// own on release. This latches which one it was, the way the 606 does with
// GROUP: set when a step is chorded, so the bare-tap action is suppressed.
static bool s_tap_used = false;

static void handle_tool(uint8_t tool, uint8_t inst) {
  const uint8_t sec = disp_section;
  if (tapB.rising()) s_tap_used = false;

  switch (tool) {
    case TOOL_LENGTH:
      // Bare step = this section's step count, the stock STEP NUMBER value —
      // the master length every voice follows by default.
      //
      // TAP + step = the SELECTED VOICE's own loop length (absolute across the
      // sections: section n, key k = n*16 + k + 1), so one voice can run
      // shorter or longer than the bar. A bare TAP puts that voice back to
      // following the pattern. Whether the voice's loop free-runs against the
      // bar or resets with it is that voice's switch in the POLY tool. The
      // GLOBAL LENGTH override that used to sit on TAP here lives on the POLY
      // tool's TAP now.
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].rising()) {
          if (tapB.held()) {
            s_tap_used = true;
            eng.SetInstLength(inst, (uint8_t)(sec * NUM_STEP_BTNS + b + 1));
            midi_send_pattern_dump(eng.cur_pat);
          } else {
            eng.SetSectionLength(sec, (uint8_t)(b + 1));
            midi_send_layout_update(eng.cur_pat);
          }
        }
      if (tapB.falling() && !s_tap_used) {
        eng.SetInstLength(inst, 0);            // this voice follows the pattern
        midi_send_pattern_dump(eng.cur_pat);
      }
      break;
    case TOOL_PRESCALE:
      // Steps 1-4 set the PRE SCALE outright; the LED frame shows which one the
      // pattern holds. Marking the dial as committed keeps a later CLEAR tap
      // from undoing this with whatever detent the dial is resting on.
      for (uint8_t b = 0; b < 4; ++b)
        if (stepB[b].rising()) {
          eng.CommitPrescale(b);
          s_pre_committed = preVal;
          midi_send_layout_update(eng.cur_pat);
        }
      break;
    case TOOL_COPY:   // 1 clear / 2 copy / 3 paste, the 606's layout
      if (stepB[0].rising()) { eng.ClearPattern(eng.cur_pat); midi_send_pattern_dump(eng.cur_pat); }
      if (stepB[1].rising())   eng.CopyPatternToBuf();
      if (stepB[2].rising()) { eng.PastePatternFromBuf(); midi_send_pattern_dump(eng.cur_pat); }
      break;
    case TOOL_XFORM:  // 1 rot left / 2 rot right / 3 reverse / 4 randomize
      if (stepB[0].rising()) eng.RotateInst(inst, sec, false);
      if (stepB[1].rising()) eng.RotateInst(inst, sec, true);
      if (stepB[2].rising()) eng.ReverseInst(inst, sec);
      if (stepB[3].rising()) eng.RandomizeInst(inst, sec, preVal & 3);   // PRE SCALE = density
      if (stepB[0].rising() || stepB[1].rising() || stepB[2].rising() || stepB[3].rising())
        midi_send_pattern_dump(eng.cur_pat);
      break;
    case TOOL_ACCENT:
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].rising()) {
          const uint8_t s = (uint8_t)(sec * NUM_STEP_BTNS + b);
          eng.ToggleStep(INST_AC, s);
          midi_send_step_update(eng.cur_pat, INST_AC, s, eng.cur().step_get(INST_AC, s));
        }
      break;
    // PROBABILITY and RATCHET are two-phase, and both take TAP where the 606
    // takes CLEAR: on this machine CLEAR is the gateway key, so a tap inside a
    // tool exits it and cannot also mean "wipe". TAP is free in every tool here.
    case TOOL_PROB:
      // Pick: the voice's steps are lit and the ones carrying a chance blink;
      // press a step to select it. Set: step n = fires n/16 of the time, and
      // step 16 = ALWAYS, which frees the slot. TAP wipes the whole voice.
      //
      // Step 16 used to pass 16 straight into prob_set, where it overflowed the
      // instrument nibble: the press looked dead and cost a slot. See prob_set.
      if (s_prob_sel == 0xFF) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) { s_prob_sel = (uint8_t)(sec * NUM_STEP_BTNS + b); break; }
        if (tapB.rising()) {
          eng.cur().prob_clear_inst(inst);
          eng.mark_pat_dirty(eng.cur_pat);
          midi_send_pattern_dump(eng.cur_pat);
        }
      } else {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            eng.cur().prob_set(inst, s_prob_sel, (uint8_t)((b + 1) & 15));
            eng.mark_pat_dirty(eng.cur_pat);
            s_prob_sel = 0xFF;
            midi_send_pattern_dump(eng.cur_pat);
            break;
          }
        if (tapB.rising()) {                     // back to always, stay in pick
          eng.cur().prob_set(inst, s_prob_sel, 0);
          eng.mark_pat_dirty(eng.cur_pat);
          s_prob_sel = 0xFF;
          midi_send_pattern_dump(eng.cur_pat);
        }
      }
      break;
    case TOOL_RATCHET:
      // Same shape. Set: the key number is the TOTAL hit count — key 1 = a
      // single hit (which CLEARS the ratchet), 2/3/4 = 2x/3x/4x retrigger,
      // anything above also 4x (ratchet_set clamps). TAP also = single hit.
      // Key 1 used to mean 2x, so there was no key that put a step back to one
      // hit: "clearing" a ratcheted step re-armed it and it kept double-firing.
      if (s_ratchet_sel == 0xFF) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) { s_ratchet_sel = (uint8_t)(sec * NUM_STEP_BTNS + b); break; }
        if (tapB.rising()) {
          eng.cur().ratchet_clear_inst(inst);
          eng.mark_pat_dirty(eng.cur_pat);
          midi_send_pattern_dump(eng.cur_pat);
        }
      } else {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            eng.cur().ratchet_set(inst, s_ratchet_sel, b);   // key n = n hits
            eng.mark_pat_dirty(eng.cur_pat);
            s_ratchet_sel = 0xFF;
            midi_send_pattern_dump(eng.cur_pat);
            break;
          }
        if (tapB.rising()) {
          eng.cur().ratchet_set(inst, s_ratchet_sel, 0);
          eng.mark_pat_dirty(eng.cur_pat);
          s_ratchet_sel = 0xFF;
          midi_send_pattern_dump(eng.cur_pat);
        }
      }
      break;
    case TOOL_POLY:
      // Steps 1-12 = the twelve voices, LED lit = POLYMETER (that row's counter
      // free-runs against the bar), dark = bar-reset (the row restarts with the
      // pattern). One key per voice, so the whole machine's loop behaviour is
      // one glance instead of one voice at a time through the INSTRUMENT dial.
      //
      // 15 = master all rows to bar-reset, 16 = master all rows to polymeter.
      // Both rewrite every pattern in the store, so they only fire while
      // stopped — the 606 puts them on CLEAR and TAP for the same reason.
      //
      // TAP + step = GLOBAL LENGTH: force that many steps on every pattern,
      // absolute across the sections (section n, key k = n*16 + k + 1), which
      // is what makes a chain of differently written patterns run to one bar.
      // A bare TAP clears it and every pattern goes back to its own layout.
      // It traded places with the per-voice length, which now rides the Length
      // tool's TAP where the operator looks for it first.
      //
      // The override is a performance control, not pattern data: it is never
      // saved, and it bypasses the part split while it is on (see pos_to_abs_).
      // No early break out of the TAP branch: held() is still true on the pass
      // that reports falling() (the debounce shift register reads 0b100 there),
      // so bailing out here would mean the bare-TAP action below never runs.
      if (tapB.held()) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            s_tap_used = true;
            eng.global_len = (uint8_t)(sec * NUM_STEP_BTNS + b + 1);
          }
      } else {
        for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
          if (stepB[b].rising()) {
            eng.SetInstPoly(b, !eng.GetInstPoly(b));
            midi_send_pattern_dump(eng.cur_pat);
          }
        if (!eng.running && (stepB[14].rising() || stepB[15].rising())) {
          eng.ApplyMasterPoly(stepB[15].rising());
          save_dirty(eng);
          for (uint8_t p = 0; p < NUM_PATTERNS; ++p) midi_send_pattern_dump(p);
        }
      }
      if (tapB.falling() && !s_tap_used) eng.global_len = 0;
      break;
    case TOOL_MUTE:
      // Steps 1-12 are the voices (1 = ACCENT). Only 12 of the 16 keys can be a
      // voice here, so the spare pair carries the 606's all/none, which it puts
      // on CLEAR and TAP — neither of which is free inside a tool on this
      // machine. 15 = mute everything but accent, 16 = unmute everything.
      for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
        if (stepB[b].rising()) eng.mute_mask ^= (uint16_t)1 << b;
      if (stepB[14].rising()) eng.mute_mask = (uint16_t)(0x0FFF & ~1u);
      if (stepB[15].rising()) eng.mute_mask = 0;
      break;
    case TOOL_RESLICE: {
      uint8_t win = 0;
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) if (stepB[b].held()) { win = (uint8_t)(b + 1); break; }
      if (win) eng.ResliceOn(win); else eng.ResliceOff();
      break;
    }
    case TOOL_ARP:
      for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
        if (stepB[b].rising()) eng.ArpAdd(b);
      if (stepB[15].rising()) eng.ArpClear();
      break;
    case TOOL_DIR:
      for (uint8_t b = 0; b < 4; ++b) if (stepB[b].rising()) eng.play_dir = b;
      break;
    case TOOL_GEN:
      for (uint8_t b = 0; b < 4; ++b)
        if (stepB[b].rising()) { eng.RandomizeInst(inst, sec, b); midi_send_pattern_dump(eng.cur_pat); }
      break;
    case TOOL_SAVE:
      if (stepB[0].rising() && !eng.running) save_dirty(eng);
      break;
    case TOOL_SECTION:
      for (uint8_t b = 0; b < NUM_SECTIONS; ++b)
        if (stepB[b].rising()) disp_section = b;
      break;
    case TOOL_SWING:
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].rising()) eng.SetSwing(b);
      break;
    default: break;
  }

  // Tools that read PRE SCALE react only to a CHANGE made after entry, never to
  // wherever the dial happens to be resting.
  if (s_pre_ref != 0xFF && preVal != s_pre_ref) {
    s_pre_ref = preVal;
    if (tool == TOOL_SECTION) disp_section = preVal;
    // The PRESCALE tool also follows the dial, so it doubles as the live
    // readout of the PRE SCALE decode: turn to detent n and step LED n lights.
    // That is the check to run against docs/HARDWARE_MAP.md if the dial ever
    // looks rotated again (see PanelMap::pre_code).
    if (tool == TOOL_PRESCALE) {
      eng.CommitPrescale(preVal);
      s_pre_committed = preVal;
      midi_send_layout_update(eng.cur_pat);
    }
  }
}

// ---------------------------------------------------------------------------
// Config menu (double-tap CLEAR)
// ---------------------------------------------------------------------------
static void handle_config_menu() {
  if (stepB[0].rising()) { g_settings.midi_channel = 0; midi_send_settings(); }
  if (stepB[1].rising()) {
    g_settings.midi_channel = (uint8_t)(g_settings.midi_channel >= 16 ? 1 : g_settings.midi_channel + 1);
    midi_send_settings();
  }
  if (stepB[4].rising()) {
    g_settings.clock_source = (uint8_t)(g_settings.clock_source == CLK_SRC_MIDI ? CLK_SRC_INTERNAL : CLK_SRC_MIDI);
    midi_send_settings();
  }
  if (stepB[5].rising()) {
    g_settings.out_mode = (uint8_t)(g_settings.out_mode == OUT_MODE_OUT ? OUT_MODE_THRU : OUT_MODE_OUT);
    midi_send_settings();
  }
  if (stepB[6].rising() && !eng.running) eng.ApplyMasterPoly(!eng.master_poly);
  if (stepB[7].rising()) midi_start_panel_calibration();
#ifdef SUPEROS_COMBINED
  // Step 9 boots the emulator, right now — the same thing SysEx 0x4D does, and
  // no more ceremony than the editor's button. There used to be a long-press
  // confirm on top (and before that, a confirm with no way to arm it, so step 9
  // did nothing at all). The confirm was guarding against an accidental one-way
  // trip; it is not one-way any more, because the emulator answers a CLEAR
  // double-press by coming back here (see emu_avr.cpp).
  if (stepB[EMU_STEP].rising()) {
    save_dirty(eng);
    combined_switch_firmware(FW_D650);        // does not return
  }
#endif
  if (stepB[15].rising()) { s_menu = false; s_menu_hold = true; }
}

// ---------------------------------------------------------------------------
// LED frames
// ---------------------------------------------------------------------------

static uint16_t build_frame(uint8_t mode, uint8_t inst) {
  uint16_t f = 0;

  if (mode == MODE_MANUAL_PLAY || mode == MODE_PLAY) {
    // Stock priority display (OM p.11): the rhythm that will play first FLASHES,
    // the other selected one stays lit. Arming an INTRO with TAP moves the flash
    // from the basic rhythm to the INTRO/FILL IN button.
    const uint8_t basic = eng.sel_slot;
    const uint8_t fill  = eng.fill_slot;
    const bool    fill_first = eng.intro_armed || eng.FillPlaying();
    const uint8_t flashing = fill_first ? fill : basic;
    const uint8_t steady   = fill_first ? basic : (eng.intro_armed ? fill : 0xFF);
    if (steady != 0xFF) f |= led_bit(steady);
    const bool on = eng.running ? eng.BeatBlink() : tempo_blink();
    if (on) f |= led_bit(flashing);
    return f;
  }

  if (mode == MODE_COMPOSE) {
    f = led_bit(eng.sel_slot);
    if (!eng.running && !tempo_blink()) f = 0;      // flashing = ready to compose
    return f;
  }

  if (mode == MODE_PATTERN_CLEAR) {
    // Every slot holding data lights, so you can see what you are erasing; the
    // selected one is inverted. Served from the occupancy cache — see
    // occ_rebuild for why this must never hit the store every pass. Rebuilds
    // only run while stopped: a ~20 ms rebuild under a running clock would
    // slip ticks, and clearing is a stopped operation anyway.
    const uint8_t v = (varVal == VAR_B) ? 1 : 0;
    if (!eng.running &&
        (s_occ_var != v || (uint32_t)(millis() - s_occ_ms) > 1000))
      occ_rebuild(v);
    f = s_occ_mask;
    f ^= led_bit(eng.sel_slot);
    return f;
  }

  // PATTERN WRITE. Show the shown part's steps for the selected instrument, with
  // the playhead inverted for the first tempo-clock tick of its step — the stock
  // machine's chase flash, measured off the program image (Engine::chase_lit).
  // Steps past this part's length stay dark.
  const uint8_t plen = eng.SectionLength(disp_section);
  for (uint8_t b = 0; b < plen && b < NUM_STEP_BTNS; ++b) {
    const uint8_t s = (uint8_t)(disp_section * NUM_STEP_BTNS + b);
    if (eng.cur().step_get(inst, s)) f |= led_bit(b);
  }
  if (eng.chase_lit()) {
    const uint8_t ph = eng.inst_playhead(inst);
    if (ph != 0xFF && (ph >> 4) == disp_section) f ^= led_bit((uint8_t)(ph & 15));
  }
  return f;
}

static uint16_t build_map_frame() {
  uint16_t f = TOOLS_IMPLEMENTED;
  if (s_tool != TOOL_NONE) f ^= led_bit((uint8_t)(s_tool - 1));
  return f;
}

static uint16_t build_tool_frame(uint8_t tool, uint8_t inst) {
  const uint8_t sec = disp_section;
  uint16_t f = 0;
  switch (tool) {
    case TOOL_LENGTH: {
      // TAP held: the selected voice's own loop length, dark = follows the
      // pattern. Otherwise a BLINKING marker means the global override is
      // running the show; solid means this section's own step count.
      if (tapB.held()) {
        const uint8_t il = eng.cur().ilen[inst % NUM_INSTRUMENTS];
        if (il && (uint8_t)((il - 1) >> 4) == sec) f = led_bit((uint8_t)((il - 1) & 15));
        break;
      }
      if (eng.global_len) {
        const uint8_t L = eng.global_len;
        if ((uint8_t)((L - 1) >> 4) == sec && tempo_blink()) f = led_bit((uint8_t)((L - 1) & 15));
        break;
      }
      const uint8_t L = eng.SectionLength(sec);
      if (L) f = led_bit((uint8_t)(L - 1));
      break;
    }
    case TOOL_PRESCALE: f = led_bit(eng.cur().prescale & 3); break;
    case TOOL_COPY:     f = 0x0003; break;
    case TOOL_XFORM:    f = 0x0007; break;
    case TOOL_ACCENT:
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (eng.cur().step_get(INST_AC, (uint8_t)(sec * NUM_STEP_BTNS + b))) f |= led_bit(b);
      break;
    // PROBABILITY / RATCHET, both phases. Pick shows the voice's PATTERN with
    // the affected steps blinking on top, so you can see what you are aiming at;
    // set shows the current VALUE as a bar, so the tool says what it is holding
    // instead of leaving you to press and listen. Both used to show only a bare
    // list of affected steps, and nothing at all about the value.
    case TOOL_PROB: {
      const Pattern &p = eng.cur();
      if (s_prob_sel == 0xFF) {
        uint16_t pm = 0;
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
          const uint8_t s = (uint8_t)(sec * NUM_STEP_BTNS + b);
          if (p.step_get(inst, s)) f |= led_bit(b);
          if (p.prob_get(inst, s)) pm |= led_bit(b);
        }
        f = tempo_blink() ? (uint16_t)(f | pm) : (uint16_t)(f & ~pm);
      } else {
        const uint8_t v = eng.cur().prob_get(inst, s_prob_sel);
        const uint8_t n = v ? v : NUM_STEP_BTNS;      // no slot = always = full bar
        for (uint8_t b = 0; b < n; ++b) f |= led_bit(b);
        if (tempo_blink() && (s_prob_sel >> 4) == sec) f ^= led_bit((uint8_t)(s_prob_sel & 15));
      }
      break;
    }
    case TOOL_RATCHET: {
      const Pattern &p = eng.cur();
      if (s_ratchet_sel == 0xFF) {
        uint16_t rm = 0;
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
          const uint8_t s = (uint8_t)(sec * NUM_STEP_BTNS + b);
          if (p.step_get(inst, s)) f |= led_bit(b);
          if (p.ratchet_get(inst, s)) rm |= led_bit(b);
        }
        f = tempo_blink() ? (uint16_t)(f | rm) : (uint16_t)(f & ~rm);
      } else {
        const uint8_t e = eng.cur().ratchet_get(inst, s_ratchet_sel);
        for (uint8_t b = 0; b <= e; ++b) f |= led_bit(b);   // n LEDs = n hits
        if (tempo_blink() && (s_ratchet_sel >> 4) == sec) f ^= led_bit((uint8_t)(s_ratchet_sel & 15));
      }
      break;
    }
    case TOOL_POLY:
      // TAP held: the global length override, blinking, dark = no override.
      // Otherwise one LED per voice, lit = polymeter, with the master keys lit
      // as labels — they go dark while running, when they are refused.
      if (tapB.held()) {
        const uint8_t L = eng.global_len;
        if (L && (uint8_t)((L - 1) >> 4) == sec && tempo_blink())
          f = led_bit((uint8_t)((L - 1) & 15));
        break;
      }
      f = (uint16_t)(eng.cur().poly & 0x0FFF);
      if (!eng.running) f |= led_bit(14) | led_bit(15);
      break;
    case TOOL_MUTE:    f = (uint16_t)(eng.mute_mask & 0x0FFF); break;
    case TOOL_RESLICE: f = eng.reslice_on ? led_bit((uint8_t)(eng.reslice_len - 1)) : 0; break;
    case TOOL_ARP:
      for (uint8_t i = 0; i < eng.arp_len && i < NUM_STEP_BTNS; ++i) f |= led_bit(i);
      break;
    case TOOL_DIR:     f = led_bit(eng.play_dir & 3); break;
    case TOOL_GEN:     f = 0x000F; break;
    case TOOL_SAVE:    f = eng.running ? 0 : 0x0001; break;
    case TOOL_SECTION: f = led_bit(sec); break;
    case TOOL_SWING:   f = led_bit(eng.cur().swing & 15); break;
    default: f = 0; break;
  }
  return f;
}

static uint16_t build_menu_frame() {
  uint16_t f = 0;
  f |= led_bit(g_settings.midi_channel == 0 ? 0 : 1);
  if (g_settings.clock_source == CLK_SRC_INTERNAL) f |= led_bit(4);
  if (g_settings.out_mode == OUT_MODE_THRU)        f |= led_bit(5);
  if (eng.master_poly)                             f |= led_bit(6);
  if (midi_calibration_active())                   f |= led_bit(7);
#ifdef SUPEROS_COMBINED
  f |= led_bit(EMU_STEP);
#endif
  f |= led_bit(15);                                 // exit
  if (!tempo_blink()) f &= (uint16_t)~led_bit(15);  // blink the exit key
  return f;
}

// ---------------------------------------------------------------------------
void loop() {
  const uint32_t now_ms = millis();

  // 1. One combined matrix-scan + LED-display pass (~1 ms; the loop heartbeat).
  // The clock-pulse interrupt is masked during the pass: the status lines hop
  // levels as the selects toggle, and those fake edges would poison the tempo
  // estimate.
  PCMSK0 &= (uint8_t)~(_BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3));
  {
    extern uint8_t g_led_dwell_us;   // SysEx 0x3F live-tunable LED dwell
    hw::ScanAndDisplay(frame, panel.cell, &panel.status_hi, g_led_dwell_us);
  }
  delayMicroseconds(8);              // let the status gate settle after the
                                     // selects restore, before we re-arm
  PCIFR  = _BV(PCIF0);               // drop edge noise from the scan itself
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
  midi_set_probe(panel.cell, panel.status_hi);   // feeds the 0x3B probe and the
                                                 // calibration stream

  // 2. Debounce everything.
  for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s) stepB[s].push(panel.step(s));
  clearB.push(panel.clear());
  tapB.push(panel.tap());
  pedalFillB.push(panel.fill_in());     // rear FILL IN pedal jack
  runB.push(panel.run());
  // The COMMON-TRIG pulse (and the instrument-data lines it strobes) couples
  // into the gated PA status sense, so a clock sample taken while a pulse is in
  // flight can read a phantom level. The pulse spans 1-2 scan passes — enough to
  // feed the debouncer a fake low-high-high and step the sequencer on a phantom
  // edge. Hold the clock debouncer at its last level for those samples: a real
  // edge under the pulse is then seen a pass or two late (bounded jitter,
  // corrected at the next clean edge) instead of firing an extra step.
  clkB.push(eng.PulseActive() ? (clkB.state & 1) : panel.tempo_clk());

  modeVal = panel.mode(modeVal);
  instVal = panel.instrument(instVal);
  preVal  = panel.prescale(preVal);
  varVal  = panel.variation(varVal);
  fillVal = panel.fill_every(fillVal);

  const uint8_t mode = modeVal;
  const uint8_t inst = instVal % NUM_INSTRUMENTS;

  // First scan: adopt whatever detent PRE SCALE is resting on as the committed
  // reference, so power-on is not itself read as "the operator turned the dial".
  if (s_pre_committed == 0xFF) s_pre_committed = preVal;

  // The BASIC VARIATION dial owns the variation, but only when it MOVES.
  // Asserting it on every pass meant anything else that set the variation —
  // a Program Change from a DAW, the editor — was overwritten by the dial about
  // a millisecond later, before it could reach the next measure boundary. Now
  // the dial writes on a change and otherwise leaves the value alone, so the
  // last deliberate setter wins and moving the dial still takes it back.
  static uint8_t s_var_panel = 0xFF;
  if (varVal != s_var_panel) { s_var_panel = varVal; eng.SetVariation(varVal); }
  eng.fill_every = fillVal;
  eng.if_var_b   = panel.if_variation_b();
  if (pedalFillB.rising()) eng.TapFill();   // the rear FILL IN pedal

  clr.update(clearB, now_ms);

  // 3. Engine housekeeping: end a pending trigger pulse, clear step_advanced.
  eng.Service();

  // 3b. Drain MIDI IN early so an external clock/transport can drive this pass.
  MidiClockIn mc;
  midi_rx_poll(eng, mc);

#ifdef SUPEROS_COMBINED
  if (midi_take_fw_switch() == FW_D650) {
    save_dirty(eng);
    combined_switch_firmware(FW_D650);
  }
#endif

  if (g_settings.clock_source != CLK_SRC_MIDI) {
    mc.pulses = 0; mc.transport = false; mc.started = false; mc.stopped = false;
  }
  if (mc.pulses)  s_last_mclk_ms = now_ms;
  if (mc.started) s_last_mclk_ms = now_ms;
  const bool ext_sync = (uint32_t)(now_ms - s_last_mclk_ms) < MCLK_TIMEOUT_MS;

  // A latched MIDI transport whose clock has been silent past the timeout means
  // the master is gone (cable pulled, app closed): release the latch and give
  // the transport back to the panel, instead of free-running forever.
  if (mc.transport && !ext_sync) {
    midi_transport_clear();
    mc.transport = false;
  }

  // 4. Transport — the panel START/STOP flip-flop OR an external MIDI transport.
  if (runB.rising())  s_hw_run = true;
  if (runB.falling()) s_hw_run = false;
  const bool want_run = s_hw_run || mc.transport;

  bool replay_tick = false;
  if ((want_run && !s_want_run) || (mc.started && want_run)) {
    eng.Start(mode == MODE_PLAY, mode == MODE_COMPOSE);
    { const uint8_t sreg = SREG; cli(); s_clk_running = true; SREG = sreg; }
    midiRT(0xFA);
    replay_tick = !ext_sync && !clkB.rising_now() && s_clk_edge_seen &&
                  (uint32_t)(micros() - s_clk_edge_us) < CLK_EDGE_GRACE_US;
  }
  if (!want_run && s_want_run) {
    eng.Stop();
    send_note_offs();
    midiRT(0xFC);
    save_dirty(eng);
    { const uint8_t sreg = SREG; cli();
      s_clk_running = false; s_per_stop = 0; s_stop_seen = 0; s_k_half = 0;
      SREG = sreg; }
  }
  s_want_run = want_run;

  // 5. Clock — external MIDI clock when present, otherwise the 808's own
  // 24-PPQN tempo clock (TEMPO knob / rear sync jack).
  uint8_t ticks = 0;
  if (ext_sync) {
    ticks = mc.pulses;
  } else if (clkB.rising_now()) {
    { const uint8_t sreg = SREG; cli(); clk_track_edge(micros()); SREG = sreg; }
    midiRT(0xF8);
    ticks = 1;
    s_clk_edge_us   = micros();
    s_clk_edge_seen = true;
  } else if (replay_tick) {
    ticks = 1;
    s_clk_edge_seen = false;
  } else if (s_clk_edge_seen &&
             (uint32_t)(micros() - s_clk_edge_us) >= CLK_EDGE_GRACE_US) {
    s_clk_edge_seen = false;   // stale; also guards the ~71 min micros() wrap
  }
  for (uint8_t t = 0; t < ticks; ++t) {
    if (eng.ClockTick()) {
      send_note_offs();
      for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
        if (eng.fired() & led_bit(i))
          midiNoteOn(INSTRUMENT_NOTE[i], eng.fired_accent() ? 127 : 96);
      prev_fired = eng.fired();
      midi_send_step_position(eng.cur_pat, eng.position());
    }
  }

  // 5b. Config menu: double-tap CLEAR to open, step 16 or another double-tap to
  // close. s_menu_hold keeps the mode handlers frozen until CLEAR is released so
  // the closing press can't fall through into a mode handler.
  if (!s_menu && !s_menu_hold && clr.dtap) {
    s_menu = true;
    clr.consumed = true;
  } else if (s_menu && clr.dtap) {
    s_menu = false;
    s_menu_hold = true;
  }
  if (s_menu_hold && !clearB.held()) s_menu_hold = false;

  if (s_menu || s_menu_hold) {
    if (s_menu) handle_config_menu();
    midi_tx_service(eng);
    if (midi_take_save_request(eng))  save_dirty(eng);
    if (midi_take_settings_save(eng)) save_settings(g_settings);
    if (midi_take_panelmap_save(eng)) save_panel_map(g_panel_map);
    eng.SetIndicators(eng.live_variation(), modeVal == MODE_2ND_PART);
    frame = build_menu_frame();
    return;
  }

  // 5c. Tool selection. Holding CLEAR past the hold threshold opens the tool
  // map; a step press while it is open latches that tool. Releasing with no step
  // pressed exits the current tool. Tools live only in the write modes, where
  // CLEAR's stock chord (STEP NUMBER) is the one thing they have to share with.
  const bool tools_ok = mode_is_write(mode);
  if (tools_ok && clr.map_open() && clearB.held()) {
    for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s)
      if (stepB[s].rising()) {
        s_tool = (uint8_t)(s + 1);
        clr.chorded  = true;
        clr.consumed = true;
        s_pre_ref = preVal;
      }
  }
  if (clearB.falling()) {
    s_clear_up_ms = now_ms;      // start the stray-step swallow window
    if (clr.hold_seen && !clr.chorded) s_tool = TOOL_NONE;
  }
  if (clr.tap && s_tool != TOOL_NONE) { s_tool = TOOL_NONE; clr.tap = false; }

  // 6. Dispatch.
  if (mode != prev_mode) {
    anchor = -1;
    s_tool = TOOL_NONE;          // turning the dial lands you in a base mode
    if (!eng.running) save_dirty(eng);
    s_occ_var = 0xFF;            // entering PATTERN CLEAR shows fresh data
    prev_mode = mode;
  }
  if (tools_ok && clr.map_open() && clearB.held()) {
    // viewing the map / picking a tool: handlers frozen
  } else if (tools_ok && s_tool != TOOL_NONE) {
    handle_tool(s_tool, inst);
  } else {
    switch (mode) {
      case MODE_1ST_PART:
      case MODE_2ND_PART:      handle_pattern_write(inst);  break;
      case MODE_MANUAL_PLAY:   handle_manual_play();        break;
      case MODE_PLAY:          handle_track_play(inst);     break;
      case MODE_COMPOSE:       handle_compose(inst);        break;
      case MODE_PATTERN_CLEAR: handle_pattern_clear_mode(); break;
      default: break;
    }
  }

  // A CLEAR tap in a write mode with no tool latched commits the PRE SCALE dial
  // into the pattern — the stock behaviour from OM p.13.
  //
  // ONLY IF THE DIAL HAS MOVED since the last commit, which is the same rule the
  // tools already follow (see s_pre_ref in handle_tool): react to a CHANGE the
  // operator made, never to wherever the dial happens to be resting. The stock
  // procedure is "set PRE SCALE, then press CLEAR", so a deliberate commit
  // always passes; what it stops is every OTHER CLEAR tap silently restamping
  // the resting detent over a pattern that was written at a different one.
  // That is what turned a mistimed double-press — one the burst counter did not
  // read as a menu gesture — into a pattern whose step timing had changed.
  if (clr.tap && tools_ok && s_tool == TOOL_NONE) {
    clr.tap = false;
    if (preVal != s_pre_committed) {
      s_pre_committed = preVal;
      eng.CommitPrescale(preVal);
      midi_send_layout_update(eng.cur_pat);
    }
  }

  // The arp is momentary, not a latch: leaving the ARP tool clears it so the
  // pattern comes back. A latched arp full of rests reads as a dead machine.
  static uint8_t s_tool_prev = TOOL_NONE;
  if (s_tool_prev == TOOL_ARP && s_tool != TOOL_ARP) eng.ArpClear();
  if (s_tool != s_tool_prev) { s_prob_sel = 0xFF; s_ratchet_sel = 0xFF; }
  s_tool_prev = s_tool;

  // 7. Web-editor MIDI link, outgoing side.
  midi_tx_service(eng);
  if (midi_take_save_request(eng))  save_dirty(eng);
  if (midi_take_settings_save(eng)) save_settings(g_settings);
  if (midi_take_panelmap_save(eng)) save_panel_map(g_panel_map);

  // 8. Panel indicator LEDs + next display frame.
  //
  // WHILE RUNNING the 1ST/2ND PART lamp follows the section being PLAYED, like
  // the stock machine's: a measure is the sections concatenated, so the lamp
  // flips as playback crosses from the 1st PART into the 2nd. Sections 0 and 2
  // are 1st PARTs, 1 and 3 are 2nd PARTs, hence (section & 1). Driving it from
  // the MODE dial instead lit 2ND PART the moment the dial moved, a whole part
  // — or a whole measure — before that part was audible.
  //
  // STOPPED there is nothing playing, so it shows the section being EDITED
  // rather than just the dial: the SECTION tool moves the display without
  // touching the dial, and sections 3/4 are the second parts of the extended
  // halves. That is also the one panel-only test separating a dead lamp from a
  // mis-decoded MODE knob — with the machine stopped, the SECTION tool can
  // light the 2ND PART lamp with the dial untouched.
  const uint8_t play_sec = eng.playing_section();
  const bool part2_lamp =
      (play_sec != 0xFF) ? ((play_sec & 1) != 0)
                         : (mode_is_write(mode) ? ((disp_section & 1) != 0)
                                                : (mode == MODE_2ND_PART));
  eng.SetIndicators(eng.live_variation(), part2_lamp);
  if (tools_ok && clr.map_open() && clearB.held()) frame = build_map_frame();
  else if (tools_ok && s_tool != TOOL_NONE)        frame = build_tool_frame(s_tool, inst);
  else                                             frame = build_frame(mode, inst);
  {
    extern uint16_t g_frame_override; extern uint8_t g_frame_override_en;
    if (g_frame_override_en) frame = g_frame_override;   // SysEx 0x3E bench test
  }
}
