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

#include <avr/wdt.h>
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "pins.h"
#include "usb_sof.h"
#include "loop_acct.h"
#include "hw.h"
#include "controls.h"
#include "pattern.h"
#include "engine.h"
#include "settings.h"
#include "flash_persist.h"
#include "boot_update.h"
#include "midi_api.h"
#ifdef SUPEROS_COMBINED
#include "combined.h"
#endif

#ifdef SUPEROS_CPU_DIV4
// Divide the core clock by 4: a 16 MHz crystal gives a 4 MHz CPU. This has to
// run in .init3, ahead of the Arduino core's timer setup and the USB core's
// usb_init(), so that every F_CPU-derived constant the build computed for
// 4 MHz matches the clock that is actually running.
//
// USB survives this. CLKPR divides clkCPU, clkI/O, clkADC and clkFLASH only
// (datasheet 7.9); clkUSB comes off the PLL, which is fed from XTAL1 and still
// sees the full 16 MHz, so the 48 MHz USB reference and PLLCSR=0x16 both stay
// correct. A 4 MHz CRYSTAL would be a different matter entirely: datasheet
// Table 7-11 gives valid PLL prescaler words for 8 and 16 MHz sources only,
// and marks every other combination Reserved, so USB would be impossible.
//
// The write is a timed sequence (CLKPCE, then the value within four cycles).
// Interrupts are still off this early, so no cli/sei is needed. naked is
// correct here: .init sections fall through and must not return.
extern "C" __attribute__((naked, used, section(".init3")))
void superos_clock_div4(void) {
  CLKPR = _BV(CLKPCE);
  CLKPR = _BV(CLKPS1);   // CLKPS = 0b0010 -> /4
}
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
#ifdef SUPEROS_USB_MIDI
  if (usb_sof_alive()) usbMIDI.sendRealTime(b);
#endif
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
// claim-129 cut this 350 -> 150; claim-132 cuts it to ZERO. The player, on
// the 150 ms build: "it should bring up the tool selection when i press the
// clear button instantly no more waiting". The hold threshold only ever
// bought a wait before the map appeared: the tap is recognised on RELEASE
// and stays dt-based, so tap actions and the deferred-tap machinery are
// untouched by the map being visible from the press. What IS gone is the
// stock CLEAR+step chord in the write modes (it needed the pre-map window):
// its function lives in the LENGTH tool, which the instant map makes
// quicker to reach than the chord ever was. One behaviour this sharpens
// deliberately: a latched tool now exits on the RELEASE of a bare CLEAR
// press (the falling-edge unlatch used to need a 150 ms hold first), so
// tool exit is instant too.
static const uint16_t CLEAR_HOLD_MS = 0;
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
  // claim-116. PRE SCALE is no longer a tool of its own: it folds into LENGTH,
  // which is where the player already goes to decide how long a thing is. SWING
  // takes the freed slot 2, and STEP 16 becomes the CONFIG MENU, so the menu is
  // a place on the tool map rather than a gesture you have to know about.
  TOOL_LENGTH = 1, TOOL_SWING = 2, TOOL_COPY = 3, TOOL_XFORM = 4,
  // Slot 5 was ACCENT and is now EMPTY (claim-117). The 808's accent is a
  // SINGLE SHARED LEVEL per step -- fire_pulse sets the amplitude of the one
  // COMMON TRIG pulse, so every voice firing in that window is accented
  // together and there is no such thing as accenting one instrument. The tool
  // implied a per-instrument control the machine cannot have. The accent TRACK
  // is unaffected: ACCENT is a position on the INSTRUMENT SELECT dial and is
  // programmed like any other voice, which is where it belonged all along.
  // ...and claim-126 CLOSES that hole: everything after it moves down one, so
  // the map is contiguous 1..14. The EMPTY slot is now 15. STEP 16 stays the
  // CONFIG MENU and does not shift, because a menu that moves whenever a tool
  // is retired is a menu nobody can find twice.
  TOOL_PROB = 5, TOOL_RATCHET = 6, TOOL_POLY = 7, TOOL_MUTE = 8,
  TOOL_RESLICE = 9, TOOL_ARP = 10, TOOL_DIR = 11, TOOL_GEN = 12,
  TOOL_SAVE = 13, TOOL_SECTION = 14, TOOL_MENU = 16,
};
// Slot 5 (bit 4) is empty since claim-117, so the tool map shows it dark and
// the latch below refuses it: a key that lights but does nothing is worse than
// one that does not light.
static constexpr uint16_t TOOLS_IMPLEMENTED = (uint16_t)(0xFFFF & ~(1u << 14));

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

#ifdef SUPEROS_TESTHOOKS
// G12 (0x45 engine dump): the CLEAR-layer UI state, which lives here rather than
// in Engine. Tool in the low five bits (TOOL_* is 0..16), menu open in bit 5,
// tool MAP open in bit 6.
//
// Bit 6 added 2026-08-26. The map being open and a tool being latched are
// DIFFERENT states and only the second was published, so a bare CLEAR hold --
// which opens the map and deliberately latches nothing -- was indistinguishable
// from the gesture having done nothing at all. G1 row F14 read that as a FAIL
// against firmware doing exactly what it is designed to do. An agent that
// cannot observe a state cannot test it, and this one was unobservable.
uint8_t superos_ui_state() {
  return (uint8_t)((s_tool & 0x1F) | (s_menu ? 0x20 : 0)
                   | (clr.map_open() ? 0x40 : 0));
}
#endif

#ifdef SUPEROS_COMBINED
static constexpr uint8_t EMU_STEP = 8;   // menu step 9: reboot into the OTHER firmware
// Set while the config menu is being driven from the D650C side (claim-122).
// The menu itself is identical either way; only step 9 has to know which
// firmware "the other one" is.
static bool s_menu_from_emu = false;
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
// G10: set while an incoming MIDI clock is driving. The 303 (the reference,
// claim-019) needs no such flag because it has ONE timebase: its LED phase is
// the sequencer's own clk_count, so slaving the sequencer slaves the LED by
// construction. The 808 has TWO, and the second one does not stop: the panel
// tempo oscillator free-runs whatever the CPU is slaved to (it is one of the
// two panel lines that are never idle). So on this machine the fix is not only
// to ADD the MIDI source, it is to MUTE the oscillator, or the phase advances
// from both and the blink runs at the SUM of the two rates.
static volatile bool     s_clk_ext     = false;

static inline void half_tick_advance(uint16_t h) {
  s_half_tick = (uint8_t)((s_half_tick + h) % 48);
}

// Shared by the ISR and the polled path (call with interrupts off).
static void clk_track_edge(uint32_t now) {
  // G10: while an external MIDI clock owns the phase, the panel oscillator's
  // edges are noise to the indicator and are dropped here rather than at the
  // call sites, so both the ISR and the polled path are covered by one test.
  // s_clk_last_us is still stamped: on the way back to internal the first
  // interval must be a real one, not the whole slaved interval.
#ifndef G10_BREAK
  if (s_clk_ext) { s_clk_last_us = now; return; }
#endif

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

// ---------------------------------------------------------------------------
// Pin-change ISR attribution (testhooks only)
// ---------------------------------------------------------------------------
// The stopped blink phase is FROZEN on this machine: 25 samples over 5 s with no
// MIDI clock gave distinct half_tick values [0] and per_run_us still at its
// compiled-in 20833, so no edge has ever reached clk_track_edge. The ISR below
// has three early-outs and a polled read cannot tell which one fires (a polled
// read cannot see this pulse train at all, L124). These counters separate all
// five surviving causes in ONE run:
//
//   entry == 0 and masked_edge == 0   no edges on PB0-3 at all: physical, or
//                                     the pins are not inputs
//   entry == 0 and masked_edge  > 0   edges exist but only ever land while the
//                                     scan has the mask off: the arming window
//                                     never coincides with the pulse train
//   rej_scan  dominates               PORTF never reads idle when the ISR runs
//   rej_lvl   dominates               the level test rejects: clock_bit is wrong
//                                     (this machine is UNCALIBRATED, 0x61 flag
//                                     0x04 clear, so clock_bit is the default 3)
//   rej_pulse dominates               the COMMON-TRIG crosstalk gate is stuck on
//
// pinb_or / pinb_and accumulate PINB & 0x0F at ISR entry, so they name WHICH of
// the four status lines is actually toggling without running calibration: a bit
// set in `or` and clear in `and` moved during the window.
#ifdef SUPEROS_PCINT_ATTRIB
static volatile uint16_t s_pc_entry       = 0;
static volatile uint16_t s_pc_rej_scan    = 0;
static volatile uint16_t s_pc_rej_lvl     = 0;
static volatile uint16_t s_pc_rej_pulse   = 0;
// REPLACES s_pc_masked_edge, which was VOID BY CONSTRUCTION and read 0 in all
// three measured conditions including the one with 106 real edges (L120's
// recurrence, claim-104). It sampled PCIF0 at re-arm, but the mask above is
// cleared for the scan and a masked pin does not contribute to PCIF0 at all,
// so the flag was pinned to zero by a line of this same function.
//
// The question it FAILED to answer is worth answering, and this measures it
// directly: what FRACTION of each loop pass is the ISR masked for? That
// fraction is the per-edge drop probability for the stopped tempo clock, and
// via clk_track_edge's period estimate it sets the stopped blink's RATE, not
// just whether it moves. The handoff asserted ~73% from a scan-length estimate
// and claim-104 showed that figure cannot be derived from edge counts, because
// rising and falling edges are sampled at the same duty.
static volatile uint32_t s_pc_masked_us = 0;     // time with PCMSK0 low bits clear
static volatile uint32_t s_pc_total_us  = 0;     // total elapsed over the same span
static volatile uint8_t  s_pc_pinb_or     = 0x00;
static volatile uint8_t  s_pc_pinb_and    = 0x0F;
#define PC_BUMP(v)  do { if ((v) < 0x3FFF) ++(v); } while (0)
#else
#define PC_BUMP(v)  do { } while (0)
#endif

// The tempo clock arrives on one of the four PA status lines (AVR PB0-3). Which
// one is calibrated, so the ISR masks against the live map rather than a fixed
// bit; all four PCINTs vector here and the level test picks out the clock line.
#ifdef SUPEROS_COMBINED
void superos_pcint()                    // called from combined.cpp's PCINT0 ISR
#else
ISR(PCINT0_vect)
#endif
{
#ifdef SUPEROS_PCINT_ATTRIB
  { const uint8_t pb = (uint8_t)(PINB & 0x0F);
    PC_BUMP(s_pc_entry); s_pc_pinb_or |= pb; s_pc_pinb_and &= pb; }
#endif
  if ((PORTF & 0x0F) != 0x0F) { PC_BUMP(s_pc_rej_scan); return; }  // matrix scan
                                        // in progress: PB != status
  if (!(PINB & (uint8_t)(1 << g_panel_map.clock_bit))) {           // rising only
    PC_BUMP(s_pc_rej_lvl); return;
  }
  if (eng.PulseActive()) {              // COMMON-TRIG crosstalk on the status
    PC_BUMP(s_pc_rej_pulse);            // sense: don't feed the tracker a
    return;                             // phantom edge either
  }
  clk_track_edge(micros());
}

static bool tempo_blink() {
  uint8_t t;
  { const uint8_t sreg = SREG; cli(); t = s_half_tick; SREG = sreg; }
  return t < 24;
}

#ifdef SUPEROS_TESTHOOKS
// -DG10_BREAK reverts the three lines of the G10 fix while KEEPING this probe,
// so the harness can be shown to go red against the exact defect it claims to
// detect. It is a build flag and never a default: a harness that has only ever
// been run against a working machine has not been shown to measure anything.
// claim-097 is why this is not optional here, where a harness passed 12/12 on
// firmware that was deliberately broken.
//
// G10 (0x45 engine dump, testhooks only): the tempo indicator's phase and the
// source driving it. The shipping image does NOT carry this and must not: it
// has 287 bytes of headroom and the phase is not something a host needs.
//
// This exists because G10 cannot be verified by watching the sequencer. The
// 303's reference check (claim-019) reads a playhead anchor, which proves the
// SEQUENCER is locked; on the 808 the sequencer was ALREADY locked and the
// indicator was the part that was not, so a sequencer-derived observation
// would have passed against the bug. The phase itself is the only honest
// channel.
//
//   out[0]  s_half_tick, 0..47
//   out[1]  1 = an external MIDI clock owns the phase
//   out[2]  what the LED is doing right now (tempo_blink())
//   out[3..5]  s_per_run in microseconds, 7 bits per byte, little end first
void superos_blink_state(uint8_t *out) {
  uint8_t t; uint8_t x; uint32_t per;
  { const uint8_t sreg = SREG; cli();
    t = s_half_tick; x = s_clk_ext ? 1 : 0; per = s_per_run; SREG = sreg; }
  out[0] = (uint8_t)(t & 0x7F);
  out[1] = x;
  out[2] = (uint8_t)(t < 24 ? 1 : 0);
  out[3] = (uint8_t)(per & 0x7F);
  out[4] = (uint8_t)((per >> 7) & 0x7F);
  out[5] = (uint8_t)((per >> 14) & 0x7F);
}

// The counters above, 12 bytes, READ AND CLEARED. Two reads a known interval
// apart therefore give rates, which is what attributing the frozen phase needs.
//
//   out[0..1]   entries          out[6..7]   rejected at PulseActive
//   out[2..3]   rejected at PORTF        out[8..9]   masked PER MILLE of the pass
//   out[4..5]   rejected at PINB level   out[10]     PINB&0x0F OR-accumulated
//                                        out[11]     PINB&0x0F AND-accumulated
//
// out[8..9] is 0..1000, or 1001 for "no pass timed yet", which is NOT the same
// as 0 and must not be shown as one: a zero there would read as "the ISR is
// never masked", the exact false reassurance the counter it replaced gave.
#ifdef SUPEROS_PCINT_ATTRIB
void superos_pcint_counts(uint8_t *out) {
  uint16_t e, rs, rl, rp; uint8_t po, pa; uint32_t mu, tu;
  { const uint8_t sreg = SREG; cli();
    e = s_pc_entry;  rs = s_pc_rej_scan;  rl = s_pc_rej_lvl;
    rp = s_pc_rej_pulse;
    mu = s_pc_masked_us;  tu = s_pc_total_us;
    po = s_pc_pinb_or;    pa = s_pc_pinb_and;
    s_pc_entry = s_pc_rej_scan = s_pc_rej_lvl = s_pc_rej_pulse = 0;
    s_pc_masked_us = s_pc_total_us = 0;
    s_pc_pinb_or = 0x00;  s_pc_pinb_and = 0x0F;
    SREG = sreg; }
  // Scaled in 32 bits before the divide: at a 1.4 ms pass this overflows only
  // past ~4300 s of accumulation, and the dump clears both every read.
  const uint16_t pm = tu ? (uint16_t)((mu * 1000UL) / tu) : (uint16_t)1001;
  const uint16_t v[5] = { e, rs, rl, rp, pm };
  for (uint8_t i = 0; i < 5; ++i) {
    out[2 * i]     = (uint8_t)(v[i] & 0x7F);
    out[2 * i + 1] = (uint8_t)((v[i] >> 7) & 0x7F);
  }
  out[10] = (uint8_t)(po & 0x0F);
  out[11] = (uint8_t)(pa & 0x0F);
}
#endif
#endif

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
  // Settings too, because BOTH firmwares need one of them: the emulator honours
  // g_settings.clock_source exactly as the sequencer does, so the config menu's
  // MIDI/INTERNAL choice (step 5) governs the emulated machine as well. Without
  // this the emulator ran on a default-constructed Settings and there was no way
  // to tell it to ignore an incoming MIDI clock.
  load_settings(g_settings);
}

#ifdef SUPEROS_LOOP_ACCT
// claim-085 Phase 1 conditions 3a and 4. Definitions for the externs in
// loop_acct.h; see that header for why the ladder injects sleep instead of
// burning it, and for the tick source's resolution and range.
volatile uint16_t g_la_ovf         = 0;
volatile uint32_t g_la_sleep_ticks = 0;
volatile uint32_t g_la_scan_passes = 0;
volatile uint32_t g_la_deb_passes  = 0;
uint32_t g_la_arm_ticks  = 0;
uint16_t g_la_ladder_us  = 0;
uint8_t  g_la_deb_div    = 1;
uint8_t  g_la_deb_phase  = 0;
uint8_t  g_la_status     = 0;

ISR(TIMER3_OVF_vect) { ++g_la_ovf; }
#endif

void setup() {
#ifdef SUPEROS_LOOP_ACCT
  la_init();                     // Timer3 free-running at clk_IO/8, before anything
                                 // else can read the clock. Nothing in this
                                 // firmware writes any TCCRn, so this is the only
                                 // owner of Timer3 in the build.
#endif
#ifndef SUPEROS_USB_MIDI
  usb_shutdown_hw();
#endif
  midi_usb_init();               // register the USB SysEx handler (no-op sans USB)
#if defined(SUPEROS_USB_MIDI) && !defined(SUPEROS_COMBINED)
  // VBUS gate (usb_sof.h): the core's usb_init ran pre-main with the engine
  // live; park it detached+frozen, then let the gate attach it right back if a
  // cable is already present. Unplugged, the USB engine clock never runs (a
  // live engine measurably corrupts the high-impedance panel matrix).
  // Combined builds do this in combined.cpp instead.
  UDCON  |= _BV(DETACH);
  USBCON |= _BV(FRZCLK);
  usb_vbus_gate();
#endif
  // Pull up the MIDI RX line so an unplugged jack can't leave the input floating
  // and self-clock the UART from matrix/EMI noise — spurious 0xF8 bytes would
  // drive the sequencer.
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  hw::Init();
  eng.Init();

  panel_map_boot_load();         // mount the store + decode table + settings
  load_all(eng);
#ifdef SUPEROS_COMBINED
  // Came back from the emulator on a CLEAR double-press: open the config menu,
  // which is where that gesture leads on this side too. One-shot.
  if (eeprom_read_byte(EE_MENU_ON_BOOT) == EE_MENU_ON_BOOT_VAL) {
    eeprom_update_byte(EE_MENU_ON_BOOT, 0xFF);
    s_menu = true;
  }
#endif

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
  // The MODE dial owns the section, but ONLY WHEN IT MOVES -- the same rule the
  // BASIC VARIATION dial already follows, and for the same reason (claim-120).
  //
  // Asserting it on every pass made sections 3 and 4 REACHABLE BUT UNEDITABLE.
  // The SECTION tool can select any of the four, but the step keys edit steps
  // only while NO tool is latched, and the instant the tool was released this
  // line snapped the display back to section 0 or 1 on the very next pass. So
  // the two extended sections could be looked at and never written to, which is
  // how a feature reads as broken while every part of it works.
  //
  // Now the dial takes the section back when the operator turns it, and the
  // SECTION tool's choice survives until then.
  static uint8_t s_mode_sec_ref = 0xFF;
  if (s_tool != TOOL_SECTION && modeVal != s_mode_sec_ref) {
    s_mode_sec_ref = modeVal;
    disp_section = (modeVal == MODE_2ND_PART) ? 1 : 0;
  }

  // A step press landing just after a CLEAR hold is almost always a late
  // tool-map press, not a step entry — swallow it.
  const bool grace = (uint32_t)(millis() - s_clear_up_ms) < 250;

  // The stock CLEAR+step chord (STEP NUMBER) lived here until claim-132: the
  // tool map now owns CLEAR from the press, the dispatcher freezes this
  // handler whenever CLEAR is down, and the chord's function lives in the
  // LENGTH tool. The dead block is gone rather than left to mislead.

  for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
    if (!stepB[b].rising() || grace) continue;
    // A press in a part that does not exist yet CREATES it (claim-133, "i
    // cant write any steps when in 2nd part mode"): a fresh pattern has
    // seclen [16,0,0,0], so 2ND PART -- and any section the SECTION tool can
    // show (claim-120) -- was all-dark and pressed like a dead panel. The
    // stock machine's 2ND PART write is how a 32-step rhythm comes to exist
    // at all, so the first press extends every zero section up to the shown
    // one to the full 16 and then lands normally. LENGTH and PATTERN CLEAR
    // take it back.
    if (eng.SectionLength(disp_section) == 0) {
      for (uint8_t s2 = 0; s2 <= disp_section; ++s2)
        if (eng.SectionLength(s2) == 0) eng.SetSectionLength(s2, NUM_STEP_BTNS);
      midi_send_layout_update(eng.cur_pat);
    }
    const uint8_t s = (uint8_t)(disp_section * NUM_STEP_BTNS + b);
    if (b >= eng.SectionLength(disp_section)) continue;   // outside this part
    // Past this VOICE's own loop length, by the same predicate the display uses
    // (claim-112). The step is dark because the voice can never read it, so a
    // press there would be an edit with no visible and no audible result: the
    // machine would silently accumulate hits nobody asked for. Existing steps
    // are still kept, which is why this blocks the PRESS and not the record.
    if (!eng.step_in_inst_loop(inst, s)) continue;
    eng.ToggleStep(inst, s);
    midi_send_step_update(eng.cur_pat, inst, s, eng.cur().step_get(inst, s));
  }

  if (eng.running) {
    // Chase-delete MOVED to PATTERN CLEAR mode (claim-129). Holding CLEAR in a
    // write mode is the tool-map chord, and one key cannot both open the map
    // and erase what is playing underneath it.
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
// when it can be stale (mode entry, variation change, after a clear, and once a
// second while stopped, to track editor pushes) and never per pass.
static uint16_t s_occ_mask = 0;
static uint8_t  s_occ_var  = 0xFF;   // variation the mask was built for; 0xFF = stale
static uint32_t s_occ_ms   = 0;

// noinline, and it buys more than it looks. This is a `static void` called only
// from superos_loop, so gcc inlined it -- and brought its 228-byte Pattern into
// superos_loop's frame, which is THE ROOT OF EVERY CHAIN ON THIS MACHINE.
// superos_loop measured 256 B and 228 of that was this one local, carried by
// paths that never open the PATTERN CLEAR display: the SysEx handler, the flash
// store, the layout mirror, all of them paying for a buffer used once a second
// in one mode. Out of line it costs its 231 B on its own chain and 28 on
// everybody else's (claim-080; the walker had to learn `ijmp` before any of this
// was visible).
static __attribute__((noinline)) void occ_rebuild(uint8_t var) {
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

  // RUNNING: holding CLEAR erases the SELECTED VOICE's steps as the chase light
  // passes them, the 606's live-erase (claim-129). This is where it belongs:
  // PATTERN CLEAR is the mode whose whole job is erasing, and it is the one
  // mode with no tool map competing for a CLEAR hold.
  //
  // STOPPED it behaves as it always did -- a CLEAR tap erases the whole slot,
  // which is the stock gesture and the one the D650C side gives you.
  if (eng.running && clr.hold_seen && clearB.held() &&
      eng.step_advanced && eng.step >= 0) {
    const uint8_t cs = (uint8_t)eng.step;
    const uint8_t inst = instVal % NUM_INSTRUMENTS;
    if (eng.cur().step_get(inst, cs)) {
      eng.ClearStep(inst, cs);
      midi_send_step_update(eng.cur_pat, inst, cs, false);
    }
    clr.chorded = true;        // an erase gesture, not a pending whole-slot tap
  }

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
// TAP INSIDE A TOOL IS AN EDGE, NEVER A HELD MODIFIER. Measured on hardware:
// the TAP status line reads as a short pulse, so tapB.held() drops within a
// few samples of the press even while the button is physically down. Two
// designs that chorded "hold TAP + step" both failed the same way on the
// machine: the TAP-held LED display flashed once and the step press then fell
// through to the bare-step action. Every TAP behaviour in the tool layer is
// therefore phase-based, like PROBABILITY and RATCHET: a TAP toggles a phase,
// the display blinks to show it, and the next step press acts in that phase.
//
// s_ilen_phase is the LENGTH / POLY tools' mode switch, and TAP is the ONLY
// thing that moves it: OFF = the tool's normal surface (master length / poly
// switches), ON = every step press writes the SELECTED VOICE's own loop
// length (absolute across the sections). It is deliberately STICKY: an
// earlier draft dropped back to master mode after one press and cleared the
// voice on the second TAP, and on hardware that read as the tool randomly
// alternating between master and voice length. In voice mode, pressing the
// step that already IS the voice's length toggles it off (the voice follows
// the pattern again). Cleared on tool change.
static bool s_ilen_phase = false;

static void handle_tool(uint8_t tool, uint8_t inst) {
  const uint8_t sec = disp_section;

  switch (tool) {
    case TOOL_LENGTH:
      // Bare step = this section's step count, the stock STEP NUMBER value:
      // the master length every voice follows by default.
      //
      // TAP toggles between MASTER mode (steps = this section's step count)
      // and VOICE mode (the display blinks at the beat; steps = the SELECTED
      // VOICE's own loop length, section n key k = n*16 + k + 1), so one
      // voice can run shorter or longer than the bar. In voice mode, pressing
      // the step that already is the voice's length clears it back to
      // following the pattern. Whether the voice's loop free-runs against the
      // bar or resets with it is that voice's switch in the POLY tool, whose
      // TAP mode is this same one. The GLOBAL LENGTH override has no panel
      // binding (eng.global_len stays for a future editor hook): its stint on
      // TAP meant the natural polymeter gesture silently armed it, and the
      // hold-TAP chord that replaced it is impossible on the hardware (see
      // s_ilen_phase above).
      if (s_ilen_phase) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            const uint8_t L = (uint8_t)(sec * NUM_STEP_BTNS + b + 1);
            eng.SetInstLength(inst, eng.cur().ilen[inst] == L ? 0 : L);
            midi_send_pattern_dump(eng.cur_pat);
            break;
          }
      } else {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            eng.SetSectionLength(sec, (uint8_t)(b + 1));
            midi_send_layout_update(eng.cur_pat);
          }
      }
      if (tapB.rising()) s_ilen_phase = !s_ilen_phase;
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
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
          const uint8_t s = (uint8_t)(sec * NUM_STEP_BTNS + b);
          if (!eng.step_in_inst_loop(inst, s)) break;   // dark: not selectable
          if (stepB[b].rising()) { s_prob_sel = s; break; }
        }
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
      // Same shape. Set: the key number is the TOTAL hit count: key 1 = a
      // single hit (which CLEARS the ratchet), 2/3/4 = 2x/3x/4x retrigger,
      // anything above also 4x (ratchet_set clamps). TAP also = single hit.
      // Key 1 used to mean 2x, so there was no key that put a step back to one
      // hit: "clearing" a ratcheted step re-armed it and it kept double-firing.
      if (s_ratchet_sel == 0xFF) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
          const uint8_t s = (uint8_t)(sec * NUM_STEP_BTNS + b);
          if (!eng.step_in_inst_loop(inst, s)) break;   // dark: not selectable
          if (stepB[b].rising()) { s_ratchet_sel = s; break; }
        }
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
      // TAP toggles the same sticky VOICE-LENGTH mode as the Length tool's:
      // while it is on (display blinking at the beat), steps set the SELECTED
      // VOICE's own loop length, and the step that already is the length
      // clears it. "TAP then a length" is the gesture an operator reaches for
      // in THIS tool when setting up a polymeter, so it means the same thing
      // in both places.
      if (s_ilen_phase) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            const uint8_t L = (uint8_t)(sec * NUM_STEP_BTNS + b + 1);
            eng.SetInstLength(inst, eng.cur().ilen[inst] == L ? 0 : L);
            midi_send_pattern_dump(eng.cur_pat);
            break;
          }
      } else {
        for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
          if (stepB[b].rising()) {
            eng.SetInstPoly(b, !eng.GetInstPoly(b));
            midi_send_pattern_dump(eng.cur_pat);
          }
        // STEP 16 TOGGLES master polymeter and its LED shows the STATE, not a
        // label (claim-117): one key answers "is it on?" and changes it. STEP
        // 15 stays the explicit "all voices back to bar-reset", which a toggle
        // cannot express in one press from an unknown state.
        //
        // BOTH WORK WHILE PLAYING (claim-118). The engine applies the musical
        // half in RAM at once and owes the store patch until the transport
        // stops, so the machine answers the press without a flash write landing
        // under the clock. The bulk re-dump waits with it: 32 dumps would flood
        // the TX ring mid-measure, and the pattern the player can actually hear
        // is the one that matters now.
        if (stepB[14].rising() || stepB[15].rising()) {
          eng.ApplyMasterPoly(stepB[15].rising() ? !eng.master_poly : false);
          if (eng.running) {
            midi_send_pattern_dump(eng.cur_pat);
          } else {
            save_dirty(eng);
            for (uint8_t p = 0; p < NUM_PATTERNS; ++p) midi_send_pattern_dump(p);
          }
        }
      }
      if (tapB.rising()) s_ilen_phase = !s_ilen_phase;
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
      eng.ResliceOn(win);            // win 0 is off
      break;
    }
    case TOOL_ARP:
      for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
        if (stepB[b].rising()) eng.ArpAdd(b);
      if (stepB[15].rising()) eng.ArpClear();
      break;
    case TOOL_DIR:
      // Two phases on TAP, the same sticky per-voice mode LENGTH and POLY use,
      // because "TAP then a value" already means "for THIS voice" in both of
      // them and a third meaning here would be a third thing to remember.
      //
      // MASTER (STEP 16 lit): steps 1-4 set the pattern's direction.
      // VOICE (display blinking): steps 1-4 set the SELECTED voice's own
      // direction, overriding the master. Pressing the direction the voice
      // already has clears the override, so the same key that sets it also
      // hands the voice back -- the gesture LENGTH uses to clear a voice
      // length, kept identical on purpose.
      if (s_ilen_phase) {
        for (uint8_t b = 0; b < 4; ++b)
          if (stepB[b].rising())
            eng.SetInstDir(inst, eng.GetInstDir(inst) == b ? 0xFF : b);
      } else {
        for (uint8_t b = 0; b < 4; ++b) if (stepB[b].rising()) eng.SetPlayDir(b);
      }
      if (tapB.rising()) s_ilen_phase = !s_ilen_phase;
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
    // The live PRE SCALE readout that the retired PRESCALE tool provided (turn
    // to detent n, step LED n lights) is the check to run against
    // docs/HARDWARE_MAP.md if the dial ever looks rotated again. It now lives
    // in LENGTH, which is the tool PRE SCALE folded into.
  }
}

// ---------------------------------------------------------------------------
// Config menu (CLEAR held + TAP, or the tool map's STEP 16)
// ---------------------------------------------------------------------------
// Settings changed here are flushed to flash once the menu closes (see the
// s_settings_dirty site in loop()). The menu itself only edits RAM: the panel
// used to leave it at that, so every menu change was lost at power-off unless
// the web editor happened to push a save. A flash write halts the CPU for a
// few ms, so the flush also waits for the sequencer to be stopped.
static bool s_settings_dirty = false;

static void handle_config_menu() {
  if (stepB[0].rising()) { g_settings.midi_channel = 0; s_settings_dirty = true; midi_send_settings(); }
  if (stepB[1].rising()) {
    g_settings.midi_channel = (uint8_t)(g_settings.midi_channel >= 16 ? 1 : g_settings.midi_channel + 1);
    s_settings_dirty = true;
    midi_send_settings();
  }
  if (stepB[4].rising()) {
    g_settings.clock_source = (uint8_t)(g_settings.clock_source == CLK_SRC_MIDI ? CLK_SRC_INTERNAL : CLK_SRC_MIDI);
    s_settings_dirty = true;
    midi_send_settings();
  }
  if (stepB[5].rising()) {
    g_settings.out_mode = (uint8_t)(g_settings.out_mode == OUT_MODE_OUT ? OUT_MODE_THRU : OUT_MODE_OUT);
    s_settings_dirty = true;
    midi_send_settings();
  }
  // Master polymeter is NOT here any more (claim-117). It was in two places at
  // once, and the POLY tool is the one that can show its state next to the
  // per-voice switches it applies to; a duplicate in a menu that shows a bare
  // lamp could only disagree with it.
  if (stepB[7].rising()) midi_start_panel_calibration();
#ifdef SUPEROS_COMBINED
  // Step 9 boots the emulator, right now — the same thing SysEx 0x4D does, and
  // no more ceremony than the editor's button. There used to be a long-press
  // confirm on top (and before that, a confirm with no way to arm it, so step 9
  // did nothing at all). The confirm was guarding against an accidental one-way
  // trip; it is not one-way any more, because the emulator answers a CLEAR
  // double-press by coming back here (see emu_avr.cpp).
  if (stepB[EMU_STEP].rising()) {
    // "the OTHER firmware", not "the emulator": since claim-122 this same menu
    // is reachable from INSIDE the emulator, where booting the emulator again
    // would be a reboot into where you already are.
    save_dirty(eng);
    combined_switch_firmware(s_menu_from_emu ? FW_SUPEROS : FW_D650);  // no return
  }
#endif
  // TAP is BACK, and back is never OUT: only PATTERN CLEAR leaves the menu.
  // The menu is flat today, so there is no level above this one to return to
  // and this is deliberately a no-op -- written down rather than left out, so
  // that adding a submenu has an obvious place to hook and cannot accidentally
  // reintroduce a TAP exit. STEP 16 used to close the menu and no longer does.
  if (tapB.rising()) { /* back: no level above the root yet */ }
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
    // The row itself is Engine::play_row (claim-115): it has to be reachable
    // from the host harness, which cannot link main.cpp. Only the blink SOURCE
    // is decided here, because it is a main.cpp concern -- the beat while
    // running, the tempo LED's own phase while stopped.
    return eng.play_row(eng.running ? eng.BeatBlink() : tempo_blink());
  }

  if (mode == MODE_COMPOSE) {
    f = led_bit(eng.sel_slot);
    if (!eng.running && !tempo_blink()) f = 0;      // flashing = ready to compose
    return f;
  }

  if (mode == MODE_PATTERN_CLEAR) {
    // Every slot holding data lights, so you can see what you are erasing; the
    // selected one is inverted. Served from the occupancy cache; see
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
    // A voice with its own loop length never reads past it, so lighting those
    // steps advertises hits that cannot sound (claim-112). `s` only climbs, so
    // the first one out of the loop ends the row.
    if (!eng.step_in_inst_loop(inst, s)) break;
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
  // The latched tool BLINKS (claim-126). It used to be XORed out of a map where
  // everything else is lit, so choosing a tool turned its key OFF -- the one key
  // you had just picked became the only dark one, which reads as "unavailable"
  // rather than "selected".
  if (s_tool != TOOL_NONE && !tempo_blink()) f &= (uint16_t)~led_bit((uint8_t)(s_tool - 1));
  return f;
}

static uint16_t build_tool_frame(uint8_t tool, uint8_t inst) {
  const uint8_t sec = disp_section;
  uint16_t f = 0;
  switch (tool) {
    case TOOL_LENGTH: {
      // Voice mode: BLINK the selected voice's own loop length AT THE BEAT
      // (the play modes' flash rate), or, when the voice has none, the
      // section length it currently follows; the blink itself says "steps set
      // this voice's length now". Master mode: this section's own step count,
      // solid.
      if (s_ilen_phase) {
        uint8_t L = eng.cur().ilen[inst % NUM_INSTRUMENTS];
        if (!L) {
          const uint8_t sl = eng.SectionLength(sec);
          L = sl ? (uint8_t)(sec * NUM_STEP_BTNS + sl) : 0;
        }
        const bool on = eng.running ? eng.BeatBlink() : tempo_blink();
        if (L && (uint8_t)((L - 1) >> 4) == sec && on)
          f = led_bit((uint8_t)((L - 1) & 15));
        break;
      }
      const uint8_t L = eng.SectionLength(sec);
      if (L) f = led_bit((uint8_t)(L - 1));
      break;
    }
    // TOOL_MENU never latches (the tool-map handler opens the menu instead), so
    // it has no tool frame; build_menu_frame owns the display from then on.
    case TOOL_COPY:     f = 0x0003; break;
    case TOOL_XFORM:    f = 0x0007; break;
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
        // Past this voice's own loop length the step can never sound, so it
        // must not be shown and must not be selectable (claim-112, extended to
        // these two tools in claim-121). `s` only climbs, so the first one out
        // ends the row.
          if (!eng.step_in_inst_loop(inst, s)) break;
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
        // Past this voice's own loop length the step can never sound, so it
        // must not be shown and must not be selectable (claim-112, extended to
        // these two tools in claim-121). `s` only climbs, so the first one out
        // ends the row.
          if (!eng.step_in_inst_loop(inst, s)) break;
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
      // Voice mode: same beat-rate blinking display as the Length tool's.
      // Normal mode: one LED per voice, lit = polymeter, with the master keys
      // lit as labels; they go dark while running, when they are refused.
      if (s_ilen_phase) {
        uint8_t L = eng.cur().ilen[inst % NUM_INSTRUMENTS];
        if (!L) {
          const uint8_t sl = eng.SectionLength(sec);
          L = sl ? (uint8_t)(sec * NUM_STEP_BTNS + sl) : 0;
        }
        const bool on = eng.running ? eng.BeatBlink() : tempo_blink();
        if (L && (uint8_t)((L - 1) >> 4) == sec && on)
          f = led_bit((uint8_t)((L - 1) & 15));
        break;
      }
      f = (uint16_t)(eng.cur().poly & 0x0FFF);
      f |= led_bit(14);                                // 15: all to bar-reset
      if (eng.master_poly) f |= led_bit(15);           // 16: master polymeter ON
      break;
    case TOOL_MUTE:    f = (uint16_t)(eng.mute_mask & 0x0FFF); break;
    case TOOL_RESLICE: f = eng.ResliceActive() ? led_bit((uint8_t)(eng.ResliceWindow() - 1)) : 0; break;
    case TOOL_ARP:
      for (uint8_t i = 0; i < eng.arp_len && i < NUM_STEP_BTNS; ++i) f |= led_bit(i);
      break;
    case TOOL_DIR:
      if (s_ilen_phase) {
        // Voice mode blinks at the beat, like the other per-voice phases. A
        // voice with NO override still shows a direction -- the master's -- so
        // the display answers "what is this voice doing" rather than going
        // blank and leaving the player to infer it.
        if (eng.running ? eng.BeatBlink() : tempo_blink())
          f = led_bit((uint8_t)((eng.InstDirOverridden(inst)
                                 ? eng.GetInstDir(inst) : eng.play_dir) & 3));
      } else {
        f = led_bit(eng.play_dir & 3);
        f |= led_bit(15);            // STEP 16 lit = this is the MASTER phase
      }
      break;
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
  if (midi_calibration_active())                   f |= led_bit(7);
#ifdef SUPEROS_COMBINED
  // STEP 9 switches to the OTHER firmware, and BLINKS while the menu is being
  // driven from the D650C side, so the key says which machine you are
  // standing in as well as what it does. The blink rides millis(), NOT
  // tempo_blink(): the D650C menu takeover pauses the emulated machine and
  // with it the tempo phase, so the "blink" froze at whatever level the
  // phase held and the player read a solid (or dark) key (claim-133,
  // "in d650c mode, 9 should be blinking").
  if (!s_menu_from_emu || ((millis() >> 8) & 1)) f |= led_bit(EMU_STEP);
#endif
  // No exit key is drawn any more: PATTERN CLEAR is the only way out
  // (claim-116), and STEP 16 blinking as "exit" would now be a lie the panel
  // tells. Removing the light is the honest half of removing the function.
  return f;
}

#ifdef SUPEROS_COMBINED
// Run the config menu FROM THE D650C SIDE, without rebooting (claim-122).
//
// Before this, the only way to reach the menu from the emulator was
// combined_switch_firmware(), which is a watchdog reset: it tore down USB,
// restarted the machine and came back in the other firmware. That is a lot of
// ceremony for changing a MIDI channel, and it means the emulator's state has
// to survive a reboot to feel continuous.
//
// It BLOCKS, and that is the honest design rather than a shortcut. The
// emulator's interpreter is not advanced while this runs, so the emulated
// machine is paused: its tempo stops, and it resumes exactly where it was. A
// non-blocking version would have to interleave two owners of PORTF -- the
// emulated CPU drives the LED rows and selects through hook_port, this drives
// them through hw::ScanAndDisplay -- and two owners of one port is how you get
// a display that flickers between two programs. Pausing is what makes the
// panel unambiguous.
//
// Pin-change interrupts are masked throughout: they vector to emu_pcint, which
// would be sampling a matrix this function is driving, and would feed the
// emulated machine's clock estimate garbage taken during someone else's scan.
void superos_menu_takeover() {
  extern uint8_t g_led_dwell_us;   // SysEx 0x3F live-tunable LED dwell
  const uint8_t saved_mask = PCMSK0;
  PCMSK0 &= (uint8_t)~(_BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3));

  s_menu_from_emu = true;
  uint16_t frame = 0;
  // The emulated machine KEEPS PLAYING while the menu is up (claim-127): the
  // player asked to be able to open this without the sequencer stopping. Its
  // panel I/O is suspended by emu_panel_borrow(), so PORTF is ours alone, but
  // its instrument data and COMMON TRIG are on other pins and are untouched.
  extern void emu_loop();

  // Require CLEAR to be RELEASED before an exit can be recognised. The gesture
  // that opened the menu (CLEAR held + TAP) leaves CLEAR down, and the exit is
  // a CLEAR tap, so without this the menu would open and close on one press.
  bool armed = false;

  for (;;) {
    wdt_reset();                   // this loop owns the machine; keep it fed
    hw::ScanAndDisplay(frame, panel.cell, &panel.status_hi, g_led_dwell_us);

    for (uint8_t i = 0; i < NUM_STEP_BTNS; ++i) stepB[i].push(panel.step(i));
    clearB.push(panel.clear());
    tapB.push(panel.tap());

    const uint32_t now_ms = millis();
    clr.update(clearB, now_ms);

    if (!clearB.held()) armed = true;
    if (armed && clr.tap) { clr.tap = false; break; }   // PATTERN CLEAR exits

    handle_config_menu();          // may not return (step 9 switches firmware)
    frame = build_menu_frame();
    emu_loop();                    // keep the emulated machine in time
  }

  PORTF = 0x0F;                    // park the matrix for whoever drives next
  s_menu_from_emu = false;
  PCIFR  = _BV(PCIF0);             // drop edges this function's own scan made
  PCMSK0 = saved_mask;
}
#endif

// The scan's between-column hook (claim-128). One line, and it is the whole
// point of the hook: the trigger data window has to close on time, not at the
// end of whatever the loop is doing, or the shared VARIATION lamp line picks up
// the loop's jitter as visible flicker.
void scan_tick() { eng.EndPulseIfDue(); }

// ---------------------------------------------------------------------------
void loop() {
#ifdef SUPEROS_LOOP_ACCT
  // The ladder's sleep rung, at the TOP because loop() has early returns (the
  // config-menu path at section 5b) and the bottom is therefore not a single
  // exit. For a duty ratio the position within the pass is irrelevant; having
  // exactly ONE site is not.
  la_ladder_sleep();
#endif
  const uint32_t now_ms = millis();

#if defined(SUPEROS_USB_MIDI) && !defined(SUPEROS_COMBINED)
  usb_vbus_gate();   // USB engine runs only while a USB-C cable is present
#endif

  // 1. One combined matrix-scan + LED-display pass (~1 ms; the loop heartbeat).
  // The clock-pulse interrupt is masked during the pass: the status lines hop
  // levels as the selects toggle, and those fake edges would poison the tempo
  // estimate.
#ifdef SUPEROS_PCINT_ATTRIB
  // Both stamps come from ONE micros() read so the masked span can never be
  // charged time the pass did not spend, and the pass total is measured from
  // the same instant every pass rather than from the mask's own edges.
  const uint32_t pc_t0 = micros();
  { static uint32_t s_pc_prev_us = 0;
    if (s_pc_prev_us) s_pc_total_us += pc_t0 - s_pc_prev_us;
    s_pc_prev_us = pc_t0; }
#endif
#ifndef PCINT_POSCTL
  PCMSK0 &= (uint8_t)~(_BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3));
#endif
  {
    extern uint8_t g_led_dwell_us;   // SysEx 0x3F live-tunable LED dwell
    hw::ScanAndDisplay(frame, panel.cell, &panel.status_hi, g_led_dwell_us);
  }
#ifdef SUPEROS_LOOP_ACCT
  // Condition 4's DENOMINATOR. NOT a number expected to fall: ScanAndDisplay
  // multiplexes the LED display and is not eliminable, only its rate is
  // negotiable, so a flat scan count is not a failed refactor (CORE's own
  // correction to the condition's wording).
  la_count_scan();
#endif
#ifdef SUPEROS_TESTHOOKS
  // G12 panel inject (0x4B), applied here and nowhere else: after the real scan,
  // before the debouncers, so an injected press goes through the same decode
  // table, debounce and gesture timing a finger does. Everything below this line
  // -- including midi_set_probe, so 0x3B reads back what was injected -- cannot
  // tell the difference, which is the point.
  midi_panel_inject(panel.cell, panel.status_hi);
#endif
  delayMicroseconds(8);              // let the status gate settle after the
                                     // selects restore, before we re-arm
  PCIFR  = _BV(PCIF0);               // drop edge noise from the scan itself
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
#ifdef SUPEROS_PCINT_ATTRIB
  s_pc_masked_us += micros() - pc_t0;
#endif
  midi_set_probe(panel.cell, panel.status_hi);   // feeds the 0x3B probe and the
                                                 // calibration stream

  // 2. Debounce everything.
#ifdef SUPEROS_LOOP_ACCT
  // Condition 4's NUMERATOR, and the decimation rung of the ladder. The block
  // bracketed here is exactly CORE's section 2: the six debouncer pushes (16
  // step buttons, CLEAR, TAP, the FILL IN pedal, RUN, the tempo clock) plus the
  // five dial reads, which is the work an event-driven dispatcher would skip on
  // a pass where nothing changed.
  //
  // THE CAVEAT THAT MAKES A DECIMATED RUN VALID ONLY AT REST, stated where the
  // code is because it is not obvious: PinState::rising() is a fixed 3-bit
  // pattern test on a shift register (`(state & 0x07) == 0x03`, line 103). A
  // skipped push does not clear an edge, it FREEZES it, so a key that is
  // actually down re-fires rising() once per skipped pass. With nothing pressed
  // and the dials at rest the whole block is inert and decimating it changes
  // nothing, which is the condition the host tool runs the decimation rung
  // under. The one input that is never at rest is the free-running tempo clock,
  // and decimating clkB only changes the rate of the 0xF8 this pass emits.
  // A DECIMATED RUN IS AN INSTRUMENT CALIBRATION, NOT A PLAYABLE MACHINE.
  if (la_debounce_due()) {
#endif
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
#ifdef SUPEROS_LOOP_ACCT
  }
#endif

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
  eng.EndPulseIfDue();   // claim-128: the drain is the tail's long pole

  // 3c. Apply any pending A<->B layout mirror. HERE, and not where it is asked
  // for, is the whole of claim-080's remaining fix: the copy needs a 228-byte
  // record buffer, and asking for it from handle_sysex -> SelectSlot ->
  // latch_prescale put that buffer 235 bytes deep on the SysEx chain. At loop
  // depth it costs the same RAM and roughly 100 bytes less stack.
  //
  // The POSITION is load bearing, not incidental. After the MIDI drain, so a
  // request made by this pass's SysEx is applied in this pass; before the
  // transport section (4) and the mode handlers (6), so the sequencer never
  // advances onto a sibling, and the panel never edits one, whose layout is
  // still stale. Drained to empty rather than one per pass for the same reason.
  //
  // Known window, stated rather than hidden: a host that writes a layout and
  // reads the SIBLING back inside a single midi_rx_poll drain sees the old
  // value. Separate round trips, which is what every tool and the suite do, do
  // not. Nothing is lost either way; the mirror still lands.
  while (eng.ServiceLayoutSync()) {}

#ifdef SUPEROS_COMBINED
  if (midi_take_fw_switch() == FW_D650) {
    save_dirty(eng);
    combined_switch_firmware(FW_D650);
  }
#endif

  // SysEx 0x4A: reboot into the resident SysEx bootloader for a firmware
  // update over USB-C or DIN, no panel access needed. Flush pending saves,
  // detach USB cleanly so the bootloader's own device re-enumerates, park
  // BOOT_MAGIC in GPIOR0 (survives the jump; cleared by any real reset) and
  // enter the boot section.
  if (midi_take_boot_request()) {
    save_dirty(eng);
    if (s_settings_dirty) { save_settings(g_settings); s_settings_dirty = false; }
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

  // SysEx 0x71: commit a staged replacement bootloader. Deferred to here so
  // saves are flushed first and the copy does not run from inside the MIDI RX
  // path: it halts the CPU for tens of milliseconds. bootupd_commit() verifies
  // the staged image against the host's CRC before it touches anything live.
  {
    uint16_t want_crc;
    if (midi_take_bootupd_commit(want_crc)) {
      save_dirty(eng);
      if (s_settings_dirty) { save_settings(g_settings); s_settings_dirty = false; }
      midi_send_bootupd_status(bootupd_commit(want_crc));
    }
  }

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
    { const uint8_t sreg = SREG; cli(); s_clk_running = true;
      // G10 phase alignment: a Start from the host IS the downbeat, so the
      // indicator phase is zeroed with the sequencer's rather than carrying
      // whatever the previous passage left. Only when slaved: on the internal
      // clock the oscillator owns the phase and zeroing it would step the LED.
#ifndef G10_BREAK
      if (ext_sync) s_half_tick = 0;
#endif
      SREG = sreg; }
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
  //
  // G10: the INDICATOR's phase source follows the clock source, which is the
  // half of this that was missing. Nothing in the MIDI-clock-IN path advanced
  // s_half_tick, so while slaved tempo_blink() read a phase driven by the
  // TEMPO knob. Note the defect was only ever visible while STOPPED: running,
  // the LED is eng.BeatBlink(), which is fed by eng.ClockTick() below and was
  // already correct on either clock.
  //
  // Handing the phase back is as much of the fix as taking it. The oscillator
  // tracker's estimates are stale after a slaved passage, so they are reset the
  // way Stop resets them and the tracker re-locks from scratch.
  static bool s_clk_ext_prev = false;
  if (ext_sync != s_clk_ext_prev) {
    const uint8_t sreg = SREG; cli();
    s_clk_ext = ext_sync;
    if (!ext_sync) {
      s_per_stop = 0; s_stop_seen = 0; s_k_half = 0; s_clk_last_us = micros();
    }
    SREG = sreg;
    s_clk_ext_prev = ext_sync;
  }

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
  // Feed the engine the measured 24-PPQN period, whichever clock is driving
  // (internal, rear sync, or MIDI): ratchet retrigger spacing derives from it.
  // A burst of MIDI pulses drained in one pass averages across the burst;
  // out-of-range intervals (first tick after silence) are rejected inside
  // SetTickPeriod.
  static uint32_t s_tick_us_prev = 0;
  if (ticks) {
    const uint32_t tus = micros();
    if (s_tick_us_prev) {
      const uint32_t per = (uint32_t)(tus - s_tick_us_prev) / ticks;
      eng.SetTickPeriod(per);
      // G10: s_per_run is the MUSICAL reference the stopped-blink ratio is
      // measured against, and clk_track_edge divides the oscillator's pulse
      // rate by it when the panel clock takes over again. Keep it in step with
      // whichever clock is actually driving, with the same range gate the
      // tracker applies, or the first seconds after unplugging a slaved host
      // blink at the old tempo's ratio.
      if (ext_sync && per >= 5000UL && per <= 150000UL) {
        const uint8_t sreg = SREG; cli(); s_per_run = per; SREG = sreg;
      }
    }
    s_tick_us_prev = tus;
    if (ext_sync) {
      // Two half-ticks per musical tick: exactly what clk_track_edge's running
      // branch applies per 24-PPQN tick. An incoming 0xF8 IS the musical rate,
      // so unlike the oscillator it needs no ratio estimate at all.
#ifndef G10_BREAK
      const uint8_t sreg = SREG; cli();
      half_tick_advance((uint16_t)(2 * ticks));
      SREG = sreg;
#endif
    }
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

  // 5b. Config menu. ENTRY, claim-133: the tool map's STEP 16 is the ONLY
  // SuperOS way in. The CLEAR-held + TAP chord (claim-116) is retired from
  // this side at the player's ask -- "clear + tap doesnt bring up the menu
  // config, that is only for d650c mode" -- because with the tool map on the
  // bare press (claim-132), CLEAR is the most-used key on the panel and a
  // chord on it kept firing the menu mid-performance. The D650C side keeps
  // the chord (emu_avr.cpp menu_gesture): its panel has no tool map, so the
  // chord is its only route.
  //
  // EXIT is unchanged: PATTERN CLEAR is the only way out (the player's
  // rule). TAP is "back" inside the menu and must never leave it, which is
  // handled by TAP simply having no exit path in handle_config_menu.
  // s_menu_hold keeps the mode handlers frozen until CLEAR is released so
  // the closing press cannot fall through into a mode handler.
  if (s_menu && clr.tap) {
    clr.tap = false;
    s_menu = false;
    s_menu_hold = true;
  }
  if (s_menu_hold && !clearB.held()) s_menu_hold = false;

  // Flush menu-made settings to flash once the menu has closed and the
  // sequencer is stopped (a flash write stalls the CPU for a few ms, which
  // would slip the clock mid-playback). This is what makes panel config
  // changes survive a power cycle.
  // claim-118: a master-polymeter change made under the clock owes the store a
  // patch. Settle it here, where every other deferred write already waits for
  // the transport to stop.
  if (!eng.running && eng.MasterPolyOwed() && eng.ServiceMasterPoly()) {
    save_dirty(eng);
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) midi_send_pattern_dump(p);
  }

  if (s_settings_dirty && !s_menu && !eng.running) {
    save_settings(g_settings);
    s_settings_dirty = false;
  }

  if (s_menu || s_menu_hold) {
    if (s_menu) handle_config_menu();
    midi_tx_service(eng);
    // Never write flash while the sequencer is running. Short-circuit on
    // purpose: the request is NOT consumed while running, so it fires the
    // moment the transport stops rather than being dropped.
    if (!eng.running && midi_take_save_request(eng))  save_dirty(eng);
    if (!eng.running && midi_take_settings_save(eng)) save_settings(g_settings);
    if (midi_take_panelmap_save(eng)) save_panel_map(g_panel_map);
    eng.SetIndicators(eng.live_variation(), modeVal == MODE_2ND_PART);
    frame = build_menu_frame();
    return;
  }

  // 5c. Tool selection. CLEAR down shows the tool map INSTANTLY (claim-132);
  // a step press while it is down latches that tool, releasing with no step
  // pressed exits the current tool. Tools live in the write modes and, since
  // claim-132, in MANUAL PLAY ("in pattern write mode or manual play it
  // should bring up the tool selection when i press the clear button"), where
  // CLEAR previously did nothing at all. While CLEAR is down the mode's own
  // handler is frozen, so MANUAL PLAY's slot selects never race a latch.
  const bool tools_ok = mode_is_write(mode) || mode == MODE_MANUAL_PLAY;
  if (tools_ok && clr.map_open() && clearB.held()) {
    for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s)
      if (stepB[s].rising()) {
        clr.chorded  = true;
        clr.consumed = true;
        s_pre_ref = preVal;
        // STEP 16 is the CONFIG MENU, not a tool that latches: it opens the
        // menu and leaves s_tool clear, so the menu's own CLEAR exit is the
        // only thing that has to unwind it.
        if ((uint8_t)(s + 1) == TOOL_MENU) { s_tool = TOOL_NONE; s_menu = true; }
        else if (!((TOOLS_IMPLEMENTED >> s) & 1)) s_tool = TOOL_NONE;  // empty slot
        else                                 s_tool = (uint8_t)(s + 1);
      }
  }
  if (clearB.falling()) {
    s_clear_up_ms = now_ms;      // start the stray-step swallow window
    if (clr.hold_seen && !clr.chorded) s_tool = TOOL_NONE;
  }
  if (clr.tap && s_tool != TOOL_NONE) { s_tool = TOOL_NONE; clr.tap = false; }

  // PRE SCALE, folded into LENGTH (claim-116): with LENGTH latched, HOLD
  // PATTERN CLEAR and move the PRE SCALE dial to set the pattern's prescale.
  //
  // CHECKED HERE, NOT IN handle_tool, and that is forced rather than stylistic:
  // holding CLEAR past CLEAR_HOLD_MS (350 ms) opens the tool map, and an open
  // map FREEZES the tool handlers. A gesture whose whole premise is a CLEAR
  // hold therefore cannot be seen from inside the handler it belongs to. It
  // keeps its own dial reference for the same reason -- handle_tool's s_pre_ref
  // stops being updated the moment the map opens.
  //
  // It reacts to a CHANGE made during the hold, never to where the dial is
  // resting, which is the rule every other PRE SCALE reader on this machine
  // follows: the stock procedure is "set PRE SCALE, then press CLEAR", and
  // reacting to rest position is what once let an ordinary CLEAR tap restamp a
  // pattern written at a different detent.
  {
    static uint8_t s_len_pre_ref = 0xFF;
    if (tools_ok && s_tool == TOOL_LENGTH && clearB.held()) {
      if (s_len_pre_ref == 0xFF) {
        s_len_pre_ref = preVal;              // arm on the dial's entry position
      } else if (preVal != s_len_pre_ref) {
        s_len_pre_ref = preVal;
        clr.chorded = true;   // this hold was an edit, not a pending CLEAR tap
        if (s_ilen_phase) {
          // VOICE phase (the display is blinking): this dial move sets the
          // SELECTED voice's own PRE SCALE, which is the player's rule --
          // "if the led is bright we change the pre-scale for the whole
          // pattern and if it is blinking then we set the pre scale for that
          // instrument, same thing as setting pattern length" (claim-124).
          //
          // Dialling the value the voice already holds clears the override and
          // hands it back to the pattern. That is the same "press what it
          // already is to clear it" gesture LENGTH and DIR use, which is the
          // only way to get back with a dial that has no off position.
          eng.SetInstPre(inst, eng.GetInstPre(inst) == preVal ? 0xFF : preVal);
          midi_send_pattern_dump(eng.cur_pat);
        } else {
          eng.CommitPrescale(preVal);
          s_pre_committed = preVal;
          midi_send_layout_update(eng.cur_pat);
        }
      }
    } else {
      s_len_pre_ref = 0xFF;
    }
  }

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
  // mode_is_write and not tools_ok: MANUAL PLAY joined the tool modes in
  // claim-132, but its bare CLEAR tap stays inert -- the stock prescale
  // commit is a WRITE-mode gesture and silently stamping the dial from a
  // performance mode is the exact accident this guard exists to stop.
  if (clr.tap && mode_is_write(mode) && s_tool == TOOL_NONE) {
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
  if (s_tool != s_tool_prev) { s_prob_sel = 0xFF; s_ratchet_sel = 0xFF; s_ilen_phase = false; }
  s_tool_prev = s_tool;

  // 7. Web-editor MIDI link, outgoing side.
  midi_tx_service(eng);
  // As in the menu path: a save asked for mid-pattern waits for the stop.
  // A flash page write halts the CPU for milliseconds, which is a dropped step
  // if it lands while the sequencer is counting.
  if (!eng.running && midi_take_save_request(eng))  save_dirty(eng);
  if (!eng.running && midi_take_settings_save(eng)) save_settings(g_settings);
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
  eng.EndPulseIfDue();   // claim-128: last chance before the pass's own tail
  eng.SetIndicators(eng.live_variation(), part2_lamp);
  if (tools_ok && clr.map_open() && clearB.held()) frame = build_map_frame();
  else if (tools_ok && s_tool != TOOL_NONE)        frame = build_tool_frame(s_tool, inst);
  else                                             frame = build_frame(mode, inst);
  {
    extern uint16_t g_frame_override; extern uint8_t g_frame_override_en;
    if (g_frame_override_en) frame = g_frame_override;   // SysEx 0x3E bench test
  }
}
