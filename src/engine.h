// SuperOS-808 — sequencer engine
//
// Clocked by the 808's own tempo clock (24 PPQN on the PA status lines, the same
// analog clock the stock CPU used, and the same line the rear sync jack drives
// when SYNC IN/OUT is set to IN). Run state is the START/STOP toggle flip-flop
// level on the status lines.
//
// Each step boundary fires the voices whose bit is set in the playing pattern:
// instrument-data lines high on PD/PE/PF + a COMMON TRIG pulse on PI2, with AC
// (PF3) high when the step is accented. Per the service manual p.4 the voice
// gate ANDs its data bit with COMMON TRIG, and AC sets the pulse AMPLITUDE, so
// accent is one shared level per trigger window, not a per-voice attribute.
//
// The 808's own performance model lives here and follows the owner's manual:
//   - a measure is the pattern's sections played back to back (1st PART then
//     2nd PART, plus SuperOS's sections 3-4), OM Fig. 8
//   - BASIC VARIATION A / B / AB, where AB alternates measures always starting
//     on A, and a variation change takes effect "on the first beat of the
//     following measure" (OM p.12)
//   - INTRO/FILL IN: TAP arms an intro when stopped, or inserts a one-measure
//     fill at the end of the current measure when running; AUTO FILL IN inserts
//     one every 2/4/8/12/16 measures, and an intro is not counted in that
//     measure tally (OM pp.11-12)
//   - rhythm tracks store pattern SLOTS, not variations, so the live variation
//     switch applies on playback (OM p.19)
#pragma once
#include <Arduino.h>
#include "pins.h"
#include "pattern.h"
#include "lamp_probe.h"

// Trigger timing, per service manual p.4 Figure 6.
//
// The stock CPU emits a ~10 us pulse on PI2 and the monostable IC6 stretches it
// to the ~1 ms the sound generators actually need ("Since the step pulse width
// of 10us is too narrow to trigger a sound generator, it is widened to approx.
// 1ms ... accomplished by the monostable IC6"). The instrument data is held
// stable ACROSS that whole window so the voice gates AND it with the stretched
// trigger.
//
// So there are two spans, not one:
//   TRIG_STROBE_US  how long PI2 is high — short, like the stock CPU's
//   TRIG_DATA_US    how long the instrument-data lines are held — must cover
//                   IC6's ~1 ms output
// Driving PI2 high for the full data window (which is what this did at first)
// is 150x longer than the real CPU and leaves the trigger asserted across the
// whole gate window instead of edge-triggering it.
static constexpr uint16_t TRIG_STROBE_US = 10;
static constexpr uint16_t TRIG_DATA_US   = 1500;

// MIDI-IN drum hits are gathered before firing. A DAW sends the notes of one
// "chord" as separate Note Ons, but the 808 has a SINGLE shared common-trigger
// edge and a SINGLE shared accent level per hit. Firing one pulse per note races
// the voices across pulse windows and lets a later note pull the shared accent
// down. Instead we accumulate the voices/accent and fire ONE pulse once no new
// note has arrived for this long, so every stacked voice latches on the same
// edge with the same accent, exactly like a sequencer step.
//
// The window must exceed the gap between two of a chord's notes as the main loop
// SEES them: the RX FIFO is drained once per ~1 ms pass, so a chord's notes
// arrive roughly one per pass. 5 ms rides out that jitter while staying far
// below the spacing of even 64th notes (>20 ms), so it never merges genuinely
// separate hits. Cost is a fixed ~5 ms latency on MIDI-driven hits, a constant
// offset rather than jitter.
static constexpr uint16_t TRIG_GATHER_US = 5000;

// Instrument-data pin per Instrument enum value (service manual p.3):
//   PD0-3 = CH/OH/CY/CB, PE0-3 = CP/RS/HT/MT, PF0-3 = LT/SD/BD/AC
static const uint8_t INSTRUMENT_PIN[NUM_INSTRUMENTS] = {
  AC_PIN,   // INST_AC — PF3, pin 19 (trigger amplitude, not a voice)
  PF2_PIN,  // INST_BD — PF2, pin 18
  PF1_PIN,  // INST_SD — PF1, pin 17
  PF0_PIN,  // INST_LT — PF0, pin 16   (low tom / low conga, SW8)
  PE3_PIN,  // INST_MT — PE3, pin 15   (mid tom / mid conga, SW9)
  PE2_PIN,  // INST_HT — PE2, pin 14   (high tom / high conga, SW10)
  PE1_PIN,  // INST_RS — PE1, pin 13   (rim shot / claves, SW11)
  PE0_PIN,  // INST_CP — PE0, pin 12   (hand clap / maracas, SW12)
  PD3_PIN,  // INST_CB — PD3, pin 11
  PD2_PIN,  // INST_CY — PD2, pin 10
  PD1_PIN,  // INST_OH — PD1, pin  9
  PD0_PIN,  // INST_CH — PD0, pin  8
};

// PANEL INDICATOR LAMPS — one CPU line per PAIR, not one per lamp
// -----------------------------------------------------------------
// Traced off the p.8 schematic (bottom centre, Q2-Q5 / D76-D79) and confirmed
// by the p.4 text: "the LED driver transistors (Q2-Q5) for BASIC VARIATION, 1ST
// and 2ND are directly connected to the bus of PD and PE". Only TWO CPU lines
// reach that block, and each drives an INVERTER PAIR:
//
//   socket 13 = PE1 (RS data) -> R9 10K -> Q4 -> D76
//                                Q4 collector -> R16 47K -> Q5 -> D77
//   socket 12 = PE0 (CP data) -> R10 10K -> Q3 -> D78
//                                Q3 collector -> R11 47K -> Q2 -> D79
//
// So the line HIGH lights the first lamp of its pair and the line LOW lights the
// second — one of the two is ALWAYS lit, there is no both-off state. Which panel
// legend each lamp carries is CONFIRMED ON THE MACHINE (2026-08-04), because the
// schematic's legend boxes do not sit under the transistors you would expect:
// the part pair reads the opposite way round to the variation pair.
//
//   PE1 HIGH = VARIATION B    PE1 LOW = VARIATION A
//   PE0 HIGH = 2ND PART       PE0 LOW = 1ST PART
//
// PD0 and PD1 drive NO lamp. The earlier code drove PD0/PD1 as if they were the
// A and B lamps and drove PE1 from the 1ST/2ND PART state, which is why nothing
// on the panel ever followed the BASIC VARIATION dial: the A/B pair was being
// driven by the MODE dial instead. It also held PD0/PD1 high for a lamp that
// does not exist — measured on hardware 2026-08-03, that alone made the closed
// hi-hat fire on EVERY trigger (OH followed via PD1).
//
// Each base has an RC to ground (R9/C5 = 10K/10uF, R10/C4 = 10K/33uF), so the
// lamp follows the AVERAGE level over ~100 ms. Holding the line at the lamp
// value between triggers is exactly what the stock CPU does; p.4 also warns the
// lamps "may sometimes light even when unnecessary signals are applied", which
// is that averaging seeing RAM addresses on the same bus.
//
// SAFETY RULE, unchanged: never touch these lines while a trigger pulse is in
// flight. fire_pulse() writes every instrument pin explicitly from the mask, so
// a lamp line is driven LOW at trigger time unless that voice is genuinely
// firing; the danger is only in RE-RAISING one during the window afterwards,
// because IC6 stretches our 10 us strobe into ~1 ms of open voice gate. Since
// pulse_on_ stays true for the whole TRIG_DATA_US = 1500 us data window, which
// outlasts IC6, `if (pulse_on_) return` is a sufficient guard — and triggers
// come from the main loop, not an interrupt, so there is no race with it. The
// lamp level is restored by the next SetIndicators() call, one main-loop pass
// (~1 ms) later, which is far inside the lamps' own RC.
static const uint8_t LAMP_VAR_PIN  = PE1_PIN;   // socket 13, shares the RS data line
static const uint8_t LAMP_PART_PIN = PE0_PIN;   // socket 12, shares the CP data line

// MIDI note per instrument. General MIDI drum map where the 808 voice has an
// obvious GM counterpart; the toms follow GM's low/mid/high.
static const uint8_t INSTRUMENT_NOTE[NUM_INSTRUMENTS] = {
  0,    // INST_AC — accent has no note of its own (velocity carries it)
  36,   // BD  Bass Drum 1
  38,   // SD  Acoustic Snare
  41,   // LT  Low Floor Tom
  45,   // MT  Low Tom
  48,   // HT  Hi-Mid Tom
  37,   // RS  Side Stick
  39,   // CP  Hand Clap
  56,   // CB  Cowbell
  49,   // CY  Crash Cymbal 1
  46,   // OH  Open Hi-Hat
  42,   // CH  Closed Hi-Hat
};

// Pattern chain depth. FOUR, the player's spec (claim-115): "max chain for
// patterns is 4". It was 16, which nothing enforced a reason for.
//
// One constant caps every route, and that is the point of changing it here
// rather than at the gestures: the panel's range gesture already stops at
// CHAIN_MAX (main.cpp handle_manual_play) and SysEx 0x1A already clamps to it
// (midi.cpp:481), so both follow without a second rule to keep in step. Holding
// STEP 3 and pressing STEP 10 now chains STEP 3,4,5,6 rather than eight.
//
// It also buys 24 bytes of static RAM back, since chain[] and queued[] are both
// CHAIN_MAX wide, which goes straight into the stack budget.
static constexpr uint8_t CHAIN_MAX = 4;

// SysEx 0x2E RANDOMIZE lane: every lane at once. Ratified fleet-wide, and it is
// the ONLY lane value portable across the four machines, because lane 0 means
// AC here, ACCENT on the 606, a note on the 101 and the WHOLE PATTERN on the
// 303. A cross-machine schedule can use 0x7F and nothing else.
static constexpr uint8_t LANE_ALL = 0x7F;

// Play direction (DIRECTION tool). Runtime only, not persisted.
enum PlayDir : uint8_t { DIR_FWD = 0, DIR_BCK = 1, DIR_RND = 2, DIR_PP = 3 };

// Pattern storage is store-resident: only the active pattern lives in RAM, so 32
// patterns do not cost 32 x sizeof(Pattern) of SRAM. These hooks read/write one
// pattern to the block store; registered at boot via SetPatIO. Reads are cheap;
// a FLASH write halts the CPU for milliseconds, so the engine only writes while
// stopped (or defers to the next stop).
using PatReadFn  = bool (*)(uint8_t idx, Pattern &out);
using PatWriteFn = bool (*)(uint8_t idx, const Pattern &in);

// The same store, addressed by FIELD rather than by record: peek and patch a
// few bytes at a known offset (PATTERN_LAYOUT_OFF, PATTERN_POLY_OFF), and clear
// a slot outright.
//
// These exist because a record-shaped read-modify-write cannot be afforded above
// a flash write on this machine. flash_pat_write's own chain is 333 B and
// everything above it shares one 663-byte stack, so an operation that holds a
// 228-byte Pattern to edit two bytes of it overflows wherever it sits: that is
// claim-080, and it is why relocating those buffers never worked. The record
// buffer stays inside the flash layer, where exactly one exists.
//
// Three callers, all the same shape: the A<->B layout mirror (5 bytes), the
// master-poly switch (2 bytes, times 32 patterns), and PATTERN CLEAR (a
// constant record the caller does not need to hold).
using PatPeekFn  = bool (*)(uint8_t idx, uint8_t off, uint8_t *dst, uint8_t len);
using PatPatchFn = bool (*)(uint8_t idx, uint8_t off, const uint8_t *src, uint8_t len);
using PatClearFn = bool (*)(uint8_t idx);

class Engine {
 public:
  Track track[NUM_TRACKS];      // tracks stay RAM-resident (12 x 65 B)

  // ---- playback state ----
  bool    running     = false;
  bool    track_play  = false;  // started with the MODE dial in PLAY
  bool    track_write = false;  // started with the MODE dial in COMPOSE
  uint8_t cur_pat     = 0;      // absolute pattern index 0..31 (playing / editing)
  int8_t  step        = -1;     // sounding step 0..63, -1 = before the first
  uint8_t cur_track   = 0;
  uint8_t track_pos   = 0;

  uint8_t pending_prescale = 0xFF;  // PRE SCALE armed for the next commit

  // ---- panel performance state ----
  uint8_t sel_slot   = 0;        // the STEP button that is selected (0..15)
  uint8_t variation  = VAR_A;    // BASIC VARIATION dial
  bool    if_var_b   = false;    // I/F VARIATION dial (no AB position on the 808)
  uint8_t fill_every = FILL_MANUAL;
  uint8_t fill_slot  = NUM_BASIC;   // which INTRO/FILL IN (slot 12..15) is armed
  bool    intro_armed = false;   // TAP pressed while stopped: play the fill first
  bool    fill_request = false;  // TAP pressed while running: fill the next measure
  uint16_t measure   = 0;        // measures of BASIC rhythm played this run

  // ---- pattern chain (a SuperOS addition to MANUAL PLAY) ----
  uint8_t chain[CHAIN_MAX];
  uint8_t chain_len = 0;
  uint8_t chain_pos = 0;
  uint8_t queued[CHAIN_MAX];
  uint8_t queue_len = 0;

  uint16_t dirty_trk = 0;       // bit per track (12 tracks needs 16 bits)
  bool     step_advanced = false;

  // ---- SuperOS performance state (runtime; not persisted) ----
  uint8_t  play_dir   = DIR_FWD;
  // Per-voice play direction (claim-119). 0xFF = follow the master.
  //
  // ENGINE STATE, not a Pattern field, and that is a deliberate match rather
  // than a shortcut: the MASTER direction is engine state too, so storing the
  // per-voice override in the record while the thing it overrides lives in RAM
  // would be two different lifetimes for one control. It also costs no format
  // change, and PATTERN_BYTES is the key the store loads records by, so growing
  // the record makes every pattern already on the machine stop loading.
  uint8_t  idir[NUM_INSTRUMENTS];
  // Per-voice PRE SCALE (claim-124). 0xFF = follow the pattern's own.
  uint8_t  ipre[NUM_INSTRUMENTS];
  // LENGTH tool: 0 = every pattern plays its own section layout, 1..64 = force
  // this many steps on ALL patterns, which is what lets a chain of differently
  // written patterns run to one bar. Runtime only and never persisted, exactly
  // as on the 606 — it is a performance override, not part of a pattern.
  uint8_t  global_len = 0;
  uint16_t mute_mask  = 0;      // bit per Instrument, 1 = muted (bit0 = accent)
  // RESLICE: the window in steps, 0 = off. ONE field, because a flag beside a
  // window is two encodings of one fact and they drift apart at the edges: the
  // old ResliceOff() cleared the flag and left the window set, and 0x45
  // publishes the window on its own byte, so a host polling runtime state read
  // a window that was not in effect. It also made `on && len == 0` sayable,
  // which main.cpp's LED frame turns into led_bit(255).
  uint8_t  reslice_len = 0;
  uint8_t  arp[16];
  uint8_t  arp_len = 0;

  void SetPatIO(PatReadFn r, PatWriteFn w, PatPeekFn pk = nullptr,
                PatPatchFn pt = nullptr, PatClearFn pc = nullptr) {
    pat_read_ = r; pat_write_ = w;
    pat_peek_ = pk; pat_patch_ = pt; pat_clear_ = pc;
  }

  void Init() {
    memset(idir, 0xFF, sizeof(idir));    // every voice follows the master
    memset(ipre, 0xFF, sizeof(ipre));    // ...at the pattern's own prescale
    for (uint8_t i = 0; i < NUM_TRACKS; ++i) track[i].Clear();
    cache_.Clear();
    cache_idx_ = 0xFF;
    copy_buf_.Clear();
    // Every data line idles LOW so no voice can latch against a stray trigger.
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
      pinMode(INSTRUMENT_PIN[i], OUTPUT);
      digitalWrite(INSTRUMENT_PIN[i], LOW);
    }
    pinMode(PI2_PIN, OUTPUT);
    digitalWrite(PI2_PIN, LOW);
  }

  // ---- panel indicator LEDs -------------------------------------------------
  // Deferred while a trigger pulse is in flight: those lines carry instrument
  // data during the pulse and re-raising one inside IC6's stretched gate fires
  // the voice. See the note above the pin table. Called every main-loop pass, so
  // a skipped update is re-applied within ~1 ms of the pulse clearing — far below
  // the lamps' own RC integration.
  // var_showing is VAR_A or VAR_B — VAR_AB is resolved to whichever half is
  // sounding by live_variation(), so the pair alternates per measure the way the
  // stock machine's does. Anything that is not VAR_B shows A.
  void SetIndicators(uint8_t var_showing, bool part2) {
    ind_var_ = var_showing; ind_part2_ = part2;
    if (pulse_on_) return;
    drive_lamps_();
  }

  // Put both shared lines back to their lamp level. Split out of
  // SetIndicators so that Service() can call it AT THE INSTANT it releases the
  // data lines (claim-111). It used to be reachable only from SetIndicators, at
  // the far end of the main-loop pass, so every trigger left the pair at the
  // instrument-data level for the whole tail of that pass on top of the data
  // window itself -- and since PE1 LOW lights VARIATION A, a machine showing B
  // blinked A on every hit. The tail grows with panel and USB work, so the glow
  // got brighter the busier the host was.
  //
  // Safe at that point by the guard's own reasoning: `if (pulse_on_) return`
  // treats the end of the TRIG_DATA_US window as the moment re-raising these
  // lines can no longer reach IC6's stretched gate. Service() clearing
  // pulse_on_ IS that moment, so this is the first legal instant, not an early
  // one. What CANNOT be fixed here is the window itself: PE1 is the rim-shot
  // data line, it has to sit LOW across the gate or the rim shot fires on every
  // step, and VARIATION A is lit for exactly that long. The pair has no
  // both-off state.
  void drive_lamps_() {
    const uint8_t want = (uint8_t)(ind_var_ == VAR_B ? HIGH : LOW);
    digitalWrite(LAMP_VAR_PIN,  want);
    digitalWrite(LAMP_PART_PIN, ind_part2_ ? HIGH : LOW);
#ifdef SUPEROS_LAMP_PROBE
    lamp_probe_edge(g_lamp_sos, want, want);   // back to the lamp's own level
#endif
  }
  // End the trigger data window the moment TRIG_DATA_US has elapsed, and put
  // the indicator pair back with it. Safe to call as often as you like: a no-op
  // unless a pulse is in flight and due.
  //
  // CALLED FROM THE SCAN, NOT JUST FROM Service() (claim-128). claim-111 fixed
  // WHAT restores the lamp; this fixes WHEN, and the player's report is why:
  // "the a led still flickers when playing so that did not fix the problem."
  //
  // Measured against the 1981 program on the host (tools/d650/lamp_trace.sh):
  // it holds PE1 low for 1197..1206 us per trigger -- a NINE MICROSECOND
  // spread -- for a duty of 99.24% high and 1.4% RC ripple. SuperOS held it
  // low for TRIG_DATA_US plus however long the rest of the loop pass took,
  // which swings with USB traffic, flash work and what the display is drawing.
  //
  // THAT VARIANCE IS THE FLICKER. The average was never far off the original's;
  // a steady 1% glow reads as a faintly lit lamp, while a 1% glow that
  // modulates from pass to pass reads as flicker. The emulator does not have
  // the problem because its lamp_service runs every 200 us, so its window ends
  // on time rather than at the next convenient moment.
  void EndPulseIfDue() {
    if (!pulse_on_ || (uint16_t)(micros() - pulse_t0_) < TRIG_DATA_US) return;
    digitalWrite(PI2_PIN, LOW);
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
      digitalWrite(INSTRUMENT_PIN[i], LOW);
    pulse_on_ = false;
    drive_lamps_();          // closes the excursion the probe is timing   // two of those pins are the indicator pair: claim-111
  }

  // Which variation is SOUNDING right now — which is not the dial.
  //
  // Taken from cur_pat rather than from `variation`, because cur_pat is what
  // resolve_pattern_() re-derives at the measure boundary: a variation change
  // takes effect "on the first beat of the following measure" (OM p.12), and a
  // lamp driven from the switch lights up to a measure before the B pattern is
  // audible. Reading the pattern actually addressed also covers AB alternation
  // and an intro/fill measure (which follows I/F VARIATION, not BASIC) for
  // free, since resolve_pattern_() already handles both.
  uint8_t live_variation() const { return pat_var(cur_pat) ? VAR_B : VAR_A; }

  // The 16-step section sounding right now, 0xFF when stopped. Sections 0/2 are
  // 1st PARTs and 1/3 are 2nd PARTs, so the panel's 1ST/2ND PART lamp is
  // (section & 1) while running — it shows the part being PLAYED, the way the
  // stock machine's does, rather than the part the MODE dial is pointing at.
  uint8_t playing_section() const {
    return (running && step >= 0) ? (uint8_t)((uint8_t)step >> 4) : 0xFF;
  }

  // True while the chase light belongs on the playhead's step: the FIRST
  // tempo-clock tick of the step, and no longer. tick_ is 1 for exactly that
  // span (ClockTick sets it on the boundary and counts up from there).
  //
  // MEASURED OFF THE STOCK PROGRAM IMAGE, not chosen — tools/d650/chase_probe.c
  // runs the µPD650C-085 program against this same emulator core on a desktop
  // and watches the step-LED drive. The playhead LED comes on for one 24-PPQN
  // tick at the top of each step and then goes dark for the rest of it.
  //
  // Which is a different thing from a blink rate, and the two look identical
  // until you make the clock irregular. Under a deliberately jittered clock
  // (tick periods cycling 53 / 83 / 113 ms while the step stayed at 666 ms) the
  // flash followed the individual TICK, spread 68 ms. A blink at a fixed rate,
  // or a fixed fraction of the step, would both have sat at ~83 ms.
  //
  // So the duty falls out of PRE SCALE instead of being a free parameter:
  // 1/8 of the step at PRE SCALE 1 (8 ticks per step), 1/4 at 2, 1/6 at 3,
  // 1/3 at 4. The flash is a constant musical length at any tempo.
  bool chase_lit() const { return running && step >= 0 && tick_ == 1; }

  // BASIC VARIATION dial. While running a change only arms: on_wrap() calls
  // resolve_pattern_() at the measure boundary, which is where OM p.12 puts it.
  // Stopped, it applies immediately — that is what makes the dial choose which
  // memory the write modes edit, and it keeps the lamp honest either way.
  void SetVariation(uint8_t v) {
    if (v == variation) return;
    variation = v;
    if (!running) { resolve_pattern_(); latch_prescale(); }
  }

  // ---- pattern cache --------------------------------------------------------
  // Exchange two records without a third one on the stack. `t = a; a = b;
  // b = t` would put a whole 228 B Pattern on the deepest path the sequencer
  // has, in a build whose measured headroom is a few hundred bytes.
  static void swap_records_(Pattern &a, Pattern &b) {
    uint8_t *pa = (uint8_t *)&a, *pb = (uint8_t *)&b;
    for (uint16_t i = 0; i < sizeof(Pattern); ++i) {
      const uint8_t t = pa[i]; pa[i] = pb[i]; pb[i] = t;
    }
  }

  Pattern &cur() {
    if (cache_idx_ != cur_pat) {
      if (!running) {
        flush_();
      } else if (cache_dirty_ && cache_idx_ != 0xFF) {
        // claim-103. THE PARK USED TO BE AN UNCONDITIONAL OVERWRITE, and with
        // BASIC VARIATION on AB the sequencer alternates between TWO patterns,
        // so every flip discarded the record parked at the previous flip. The
        // player's symptom is the OTHER half reading back as whatever the
        // store last held, which if it was never stored is empty: reported on
        // hardware as "write to a while it is playing and b gets deleted".
        //
        // When the record being parked and the record being loaded are both
        // live and unstored, this is an EXCHANGE, not an overwrite, and one
        // slot is enough to do it. That is the AB case exactly: the pattern
        // being flipped to is the one parked at the previous flip.
        //
        // No flash write is added. `pend_` exists because a store halts the
        // CPU for milliseconds and cannot run under the clock, and a swap
        // keeps that property: nothing reaches the store here.
        if (pend_valid_ && pend_idx_ == cur_pat) {
          swap_records_(cache_, pend_pat_);
          pend_idx_    = cache_idx_;   // the outgoing record, still unstored
          cache_idx_   = cur_pat;      // the incoming one, also still unstored
          cache_dirty_ = true;
          return cache_;               // pend_valid_ stays true
        }
        // Three or more distinct dirty patterns in flight (a chain, not AB)
        // still cannot fit in one slot. The newest is kept and the parked one
        // is lost, which is the pre-existing behaviour, now reached only in
        // the case that genuinely overflows rather than on every AB flip.
        pend_pat_   = cache_;
        pend_idx_   = cache_idx_;
        pend_valid_ = true;
      }
      cache_idx_   = cur_pat;
      cache_dirty_ = false;
      if (pend_valid_ && pend_idx_ == cache_idx_) {
        cache_ = pend_pat_;
        pend_valid_ = false;
        cache_dirty_ = true;
      } else if (!pat_read_ || !pat_read_(cache_idx_, cache_)) {
        cache_.Clear();
      }
      // Direction travels WITH the pattern (claim-123), so adopt it whenever
      // the addressed pattern changes. play_dir and idir[] stay as the live
      // copies every read path already uses; this is the one place they are
      // refilled, because this is the one place the record underneath them
      // changes.
      adopt_dir_();
    }
    return cache_;
  }

  // Live direction <- the record. Kept next to cur() rather than inlined into
  // it so the pattern-push and SysEx paths can call it too.
  void adopt_dir_() {
    play_dir = (uint8_t)(cache_.dir & 3);
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
      idir[i] = cache_.inst_dir_get(i);
      ipre[i] = cache_.inst_pre_get(i);
    }
  }

  // One tick of the voices that run on their OWN PRE SCALE (claim-124).
  //
  // WHY THIS EXISTS AT ALL. PRESCALE_TICKS is {8, 4, 6, 3}, so a voice can be
  // FASTER than the pattern, and a faster voice has to sound BETWEEN the
  // master's steps. That is why per-voice prescale is not the one-line change
  // per-voice direction was: direction only picks a different index at the same
  // instant, prescale picks a different instant.
  //
  // The 808 has ONE common trigger, so "between the master's steps" means an
  // extra pulse, which is exactly what the ratchet machinery already does. This
  // reuses fire_pulse the same way.
  //
  // Such a voice runs its OWN position counter and is SKIPPED by fire_step, so
  // it is never sounded twice. Its own counter is also what polymeter free-run
  // would have given it, so poly_pos_ is not consulted for it either.
  void tick_voices_() {
    uint16_t m = 0;
    bool     acc = false;
    const Pattern &p = cache_;
    const uint8_t len = eff_len_();
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
      if (ipre[i] == 0xFF) continue;
      if (vtick_[i]) { --vtick_[i]; continue; }
      vtick_[i] = (uint8_t)(PRESCALE_TICKS[ipre[i] & 3] - 1);
      if (++vpos_[i] >= len) vpos_[i] = 0;
      const uint8_t d = (idir[i] != 0xFF) ? idir[i] : play_dir;
      uint8_t si = pos_to_abs_(dir_pos_(d, vpos_[i], len));
      const uint8_t li = p.ilen[i];
      if (li) si = (uint8_t)(si % li);
      if (i == INST_AC) continue;               // the accent track has no pulse
      if (p.step_get(i, si)) {
        m |= (uint16_t)1 << i;
        // The shared accent level, read at THIS voice's own step: an off-grid
        // pulse with no accent reads as a voice that suddenly went quiet.
        if (p.step_get(INST_AC, si)) acc = true;
      }
    }
    m &= (uint16_t)~mute_mask;
    if (m) fire_pulse(m, acc && !(mute_mask & 1));
  }

  void ReadPattern(uint8_t idx, Pattern &out) {
    if (idx == cache_idx_)               { out = cache_;    return; }
    if (pend_valid_ && idx == pend_idx_) { out = pend_pat_; return; }
    if (!pat_read_ || !pat_read_(idx, out)) out.Clear();
  }
  void WritePattern(uint8_t idx, const Pattern &in) {
    if (idx == cache_idx_) { cache_ = in; cache_dirty_ = true; return; }
    if (!running && pat_write_) {
      // Drop a deferred record for this SAME slot before writing through
      // (claim-097). While pend_ is valid it is authoritative -- resident_()
      // returns it ahead of the store -- so leaving it behind means ReadPattern
      // answers with the pre-edit record and the next Flush() writes that back
      // over the one just stored. Only this slot's shadow goes: a deferred
      // write to a different slot is still owed to the store.
      if (pend_valid_ && pend_idx_ == idx) pend_valid_ = false;
      pat_write_(idx, in);
      return;
    }
    pend_pat_ = in; pend_idx_ = idx; pend_valid_ = true;
  }

  void Flush() {
    flush_();
    if (pend_valid_) {
      if (pat_write_ && pend_idx_ != cache_idx_) pat_write_(pend_idx_, pend_pat_);
      pend_valid_ = false;
    }
  }

  // Apply ONE pending A<->B layout mirror; returns false when the queue is
  // empty. superos_loop drains it once per pass, after the MIDI drain and
  // before the transport section, so the sequencer never advances onto a
  // sibling whose layout is still stale. See mirror_layout_to_sibling_ for why
  // the request is deferred at all, and claim-080 for what it cost not to be.
  //
  // NO Pattern anywhere in here, and that is the point rather than a detail.
  // The obvious implementation reads the sibling's record, edits five bytes and
  // writes it back, and that is what the old mirror did. It cannot be afforded
  // at ANY depth on this machine: flash_pat_write's own chain is 333 B and
  // superos_loop's frame is 256, so a 228-byte record in between overflows the
  // 663-byte stack wherever it sits. Five bytes in, five bytes out, and the one
  // record buffer that has to exist stays inside the flash layer.
  //
  // noinline so the frame cannot migrate into superos_loop, which is the root of
  // every chain here.
  bool __attribute__((noinline)) ServiceLayoutSync() {
    if (!layout_sync_req_) return false;
    // Walk a running bit rather than recomputing (1UL << dst) each pass: the AVR
    // has no barrel shifter, so a variable 32-bit shift is a loop, and doing one
    // per iteration cost more flash than this whole function.
    uint32_t bit = 1;
    uint8_t  dst = 0;
    while (!(layout_sync_req_ & bit)) { bit <<= 1; ++dst; }
    layout_sync_req_ &= ~bit;
    const uint8_t src = sibling_(dst);

    uint8_t lay[PATTERN_LAYOUT_LEN];
    if (!peek_layout_(src, lay)) return true;   // source unreadable: drop it
    patch_layout_(dst, lay);
    return true;
  }

  uint8_t active_prescale() const { return active_pre_; }

  // ---- transport ------------------------------------------------------------
  void Start(bool in_track_play, bool in_track_write) {
    running     = true;
    track_play  = in_track_play;
    track_write = in_track_write;
    step        = -1;
    tick_       = 0;
    pos_        = 0;
    first_      = true;
    pp_back_    = false;
    reslice_len = 0;
    measure     = 0;
    ab_phase_   = 0;
    memset(rat_rem_, 0, sizeof(rat_rem_));
    arp_pos_ = 0;
    memset(poly_pos_, 0, sizeof(poly_pos_));
    rng_ ^= (uint32_t)micros() | 1u;

    if (track_play) {
      track_pos = 0;
      if (track[cur_track].len > 0) sel_slot = track[cur_track].pat(0);
    } else if (track_write) {
      // COMPOSE records in real time from measure 1; the operator clears the
      // track first (OM p.19).
      track_pos = 0;
      track[cur_track].Clear();
      mark_trk_dirty(cur_track);
    } else {
      chain_pos = 0;
      if (chain_len > 0) sel_slot = pat_slot(chain[0]);
    }

    // An armed INTRO plays first, once, before the basic rhythm (OM p.11).
    fill_now_ = intro_armed;
    intro_armed = false;
    fill_request = false;
    resolve_pattern_();
    latch_prescale();
    if (track_write) record_measure_();
  }

  void Stop() {
    running     = false;
    track_play  = false;
    track_write = false;
    step        = -1;
    tick_       = 0;
    queue_len   = 0;
    reslice_len = 0;
    fill_now_   = false;
    fill_request = false;
    memset(rat_rem_, 0, sizeof(rat_rem_));
    arp_len  = 0;
    arp_pos_ = 0;
  }

  // Quarter-note beat indicator, phase-locked to the pattern.
  bool BeatBlink() const {
    if (!running || step < 0) return false;
    const uint16_t t = (uint16_t)pos_ * PRESCALE_TICKS[active_pre_] + tick_;
    return (t % 24) < 12;
  }

  // One 24-PPQN tempo-clock tick. Returns true on a step boundary.
  bool ClockTick() {
    if (!running) return false;
    tick_voices_();                 // voices on their own PRE SCALE, every tick
    if (tick_ == 0) {
      advance_step();
      tick_ = 1;
      return true;
    }
    if (++tick_ >= step_len_()) tick_ = 0;
    return false;
  }

  // Measured 24-PPQN tick period, fed each pass by the main loop from
  // whichever clock is driving (internal, rear sync or MIDI). Ratchet retrigger
  // spacing is computed from it, so repeats stay even at any tempo and under
  // any clock source. Smoothed 3:1 here; out-of-range readings (a start after
  // silence, a glitched edge) are dropped rather than folded in.
  void SetTickPeriod(uint32_t us) {
    if (us < 2000UL || us > 200000UL) return;
    tick_us_ = (tick_us_ * 3 + us) >> 2;
  }

  // Call every main-loop pass: ends a pending trigger pulse, clears
  // step_advanced, and fires any ratchet retriggers that have come due.
  void Service() {
    step_advanced = false;
    EndPulseIfDue();
    // Ratchet retriggers are MICROSECOND-scheduled, not tick-scheduled. On
    // ticks the spacing base/(hits) truncates: a 4x ratchet on a 6-tick 16th
    // landed its hits on ticks 0,1,2,3 — a rushed burst and then silence —
    // and several other PRE SCALE x count combinations were similarly
    // lopsided. Deadlines are set in fire_step from the measured tick period;
    // the ~1 ms service cadence bounds the jitter, far below audibility at
    // ratchet spacings (>= ~15 ms).
    if (running) {
      uint16_t due = 0;
      const uint32_t now = micros();
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        if (rat_rem_[i] && (int32_t)(now - rat_next_us_[i]) >= 0) {
          due |= (uint16_t)1 << i;
          rat_rem_[i]--;
          rat_next_us_[i] += rat_sp_us_[i];
        }
      }
      if (due) fire_pulse(due, rat_accent_);
    }
    if (gather_active_ && !pulse_on_ &&
        (uint16_t)(micros() - gather_last_) >= TRIG_GATHER_US) {
      const uint16_t m = gather_mask_;
      const bool     a = gather_accent_;
      gather_mask_   = 0;
      gather_accent_ = false;
      gather_active_ = false;
      fire_pulse(m, a);
    }
  }

  // ---- editing --------------------------------------------------------------
  void ToggleStep(uint8_t inst, uint8_t s) { cur().step_tog(inst, s); mark_pat_dirty(cur_pat); }
  void SetStep(uint8_t inst, uint8_t s)    { cur().step_set(inst, s); mark_pat_dirty(cur_pat); }
  void ClearStep(uint8_t inst, uint8_t s)  { cur().step_clr(inst, s); mark_pat_dirty(cur_pat); }

  // Set or clear one step in ANY pattern, playing or not. This is what the
  // editor's live cell edits ride on (SysEx 0x13): pushing a whole 261-byte
  // pattern per click would take ~90 ms of MIDI wire time and could not keep up
  // with a hand drawing a hi-hat line.
  //
  // `scratch` is caller-owned and is touched ONLY when idx is not the playing
  // slot. It used to be a `Pattern` local, and that 228-byte frame made this the
  // deepest mainline chain on the machine: 633 B against 663, which the USB
  // endpoint ISR (33 B + 2) overflows -- while this very handler is running, on
  // a path reached FROM USB data (claim-080). Same shape and same fix as
  // claim-016, which took the identical temporary out of the dump and push
  // paths: the buffer belongs in the shared arena, not on the stack and not in
  // a static, which would trade the overflow for the same RAM.
  //
  // Note the branch. The playing slot returns early and never needs a buffer,
  // so the cost lands only on edits to a NON-selected slot. That asymmetry is
  // why an overflow here would present as intermittent, which is exactly how
  // claim-016 read before it was found.
  void SetPatternStep(uint8_t idx, uint8_t inst, uint8_t s, bool on,
                      Pattern &scratch) {
    if (idx == cur_pat) {
      if (on) SetStep(inst, s); else ClearStep(inst, s);
      return;
    }
    ReadPattern(idx, scratch);
    if (on) scratch.step_set(inst, s); else scratch.step_clr(inst, s);
    WritePattern(idx, scratch);
  }

  // Set one section's step count. Mirrored onto the slot's OTHER variation:
  // "Setting a PRE SCALE and Step number in one mode will always make it the
  // same in the other" (OM p.16). The two PARTS stay independent, which is what
  // lets a 16 + 4 = 20-step measure exist (OM Fig. 8).
  void SetSectionLength(uint8_t sec, uint8_t len) {
    if (sec >= NUM_SECTIONS) return;
    if (len > NUM_STEP_BTNS) len = NUM_STEP_BTNS;
    cur().seclen[sec] = len;
    mark_pat_dirty(cur_pat);
    mirror_layout_to_sibling_();
  }
  uint8_t SectionLength(uint8_t sec) {
    return sec < NUM_SECTIONS ? cur().seclen[sec] : 0;
  }

  // Commit the PRE SCALE dial into the pattern. On the stock machine this is
  // what pressing CLEAR does while writing (OM p.13); it is mirrored A<->B for
  // the same reason as the step counts.
  void ArmPrescale(uint8_t pre) {
    pending_prescale = pre & 3;
    if (!running) latch_prescale();
  }
  void CommitPrescale(uint8_t pre) {
    cur().prescale = pre & 3;
    mark_pat_dirty(cur_pat);
    active_pre_ = pre & 3;
    mirror_layout_to_sibling_();
  }

  void SetInstLength(uint8_t inst, uint8_t len) {
    if (len > MAX_STEPS) len = MAX_STEPS;
    if (inst < NUM_INSTRUMENTS) { cur().ilen[inst] = len; mark_pat_dirty(cur_pat); }
  }
  void SetInstPoly(uint8_t inst, bool on) {
    if (inst >= NUM_INSTRUMENTS) return;
    const uint16_t bit = (uint16_t)1 << inst;
    if (on) cur().poly |= bit; else cur().poly &= (uint16_t)~bit;
    poly_pos_[inst] = 0;
    mark_pat_dirty(cur_pat);
  }
  bool GetInstPoly(uint8_t inst) { return inst < NUM_INSTRUMENTS && ((cur().poly >> inst) & 1); }

  // The step this instrument is currently sounding, or 0xFF when stopped.
  uint8_t inst_playhead(uint8_t inst) const {
    if (!running || step < 0 || inst >= NUM_INSTRUMENTS) return 0xFF;
    const uint8_t li = cache_.ilen[inst];
    if (!li) return (uint8_t)step;
    if ((cache_.poly >> inst) & 1) return (uint8_t)(poly_pos_[inst] % li);
    return (uint8_t)((uint8_t)step % li);
  }

  // Can this instrument's own loop EVER reach this step of the record?
  //
  // fire_step reads the record at `si = (poly ? poly_pos_ : step) % ilen`, so
  // with a loop length set, the only indices any voice ever reads are
  // 0..ilen-1, whether or not it is running its own polymeter counter. Steps at
  // or past that length are DEAD: still in the record, never sounded.
  //
  // The write display has to agree with that, and did not (claim-112). It bound
  // the row by SectionLength alone, so a voice set to length 5 still lit hits on
  // steps 6..16 that the machine would never play, and the panel claimed a
  // rhythm you could not hear. This is the display's predicate, deliberately
  // derived from the same `% ilen` that decides what sounds, rather than
  // restated next to it where the two could drift apart.
  //
  // The record is NOT touched. The player's requirement is that the hidden
  // steps survive ("they should still be saved in the pattern we just wont be
  // able to see them"), so shortening a voice and lengthening it again brings
  // the old steps back, exactly as it does for a section.
  bool step_in_inst_loop(uint8_t inst, uint8_t s) {
    if (inst >= NUM_INSTRUMENTS) return true;   // not ours to hide
    const uint8_t li = cur().ilen[inst];
    return !li || s < li;
  }

  // Per-voice direction. 0xFF clears the override and the voice follows the
  // master again, which is how the panel turns one off.
  void SetInstDir(uint8_t inst, uint8_t dir) {
    if (inst >= NUM_INSTRUMENTS) return;
    const uint8_t d = (dir > DIR_PP && dir != 0xFF) ? 0xFF : dir;
    idir[inst] = d;
    cur().inst_dir_set(inst, d);          // saved with the pattern (claim-123)
    mark_pat_dirty(cur_pat);
  }

  // The pattern's own direction. Goes through here rather than assigning
  // play_dir, so the record and the live copy cannot disagree.
  void SetInstPre(uint8_t inst, uint8_t v) {
    if (inst >= NUM_INSTRUMENTS) return;
    const uint8_t n = (v > 3 && v != 0xFF) ? 0xFF : v;
    ipre[inst] = n;
    cur().inst_pre_set(inst, n);
    mark_pat_dirty(cur_pat);
    vtick_[inst] = 0; vpos_[inst] = 0;   // start on the pattern, not mid-stride
  }
  uint8_t GetInstPre(uint8_t inst) const {
    return inst < NUM_INSTRUMENTS ? ipre[inst] : 0xFF;
  }
  bool InstPreOverridden(uint8_t inst) const {
    return inst < NUM_INSTRUMENTS && ipre[inst] != 0xFF;
  }

  void SetPlayDir(uint8_t dir) {
    play_dir = (uint8_t)(dir & 3);
    cur().dir = play_dir;
    mark_pat_dirty(cur_pat);
  }
  uint8_t GetInstDir(uint8_t inst) const {
    return inst < NUM_INSTRUMENTS ? idir[inst] : 0xFF;
  }
  bool InstDirOverridden(uint8_t inst) const {
    return inst < NUM_INSTRUMENTS && idir[inst] != 0xFF;
  }
  void ClearInstDirs() { memset(idir, 0xFF, sizeof(idir)); }

  bool master_poly = false;
  // Two bytes of each of 32 patterns. This used to read, edit and write back a
  // whole 228-byte record per pattern, which put the deepest chain on the
  // machine above a flash write (claim-080: 570 B before its callers were even
  // counted). A field patch touches the same two bytes and holds two.
  // claim-118: this works WHILE PLAYING now, and the split is why it can.
  //
  // It used to be refused under the clock, for a real reason and not a timid
  // one: it field-patches all 32 patterns, and a store write halts the CPU for
  // milliseconds. That is the same fact pend_ exists for. Running it under the
  // clock would drop ticks, which is exactly the "stay in time" the player
  // asked not to lose.
  //
  // So the two halves are separated. The MUSICAL change -- master_poly, the
  // live records, and the per-voice counters -- is pure RAM and happens at
  // once, so the machine responds on the press. The STORE change is OWED and
  // settled by ServiceMasterPoly() the moment the transport stops. Nothing
  // reaches flash under the clock, so no tick is lost and the reset lands in
  // time.
  //
  // While running only RESIDENT records are touched. patch_poly_ would write
  // through to the store for any pattern that is not the cache or the deferred
  // record, which is the halt being avoided.
  void ApplyMasterPoly(bool poly) {
    master_poly = poly;
    const uint16_t v = poly ? 0x0FFF : 0x0000;
    if (running) {
      if (cache_idx_ != 0xFF && cache_.poly != v) { cache_.poly = v; cache_dirty_ = true; }
      if (pend_valid_) pend_pat_.poly = v;
      master_poly_owed_ = true;
    } else {
      for (uint8_t p = 0; p < NUM_PATTERNS; ++p) patch_poly_(p, v);
      master_poly_owed_ = false;
    }
    // WHEN the voices realign, which took three attempts to land and is worth
    // recording as all three.
    //
    //   at the bar      -- rejected: up to a whole measure of lag, which on a
    //                      box you play by hand reads as the key not working.
    //   instantly       -- rejected: it lands in the middle of a step, so a
    //                      voice is cut part-way through and it is not clean.
    //   the NEXT STEP   -- this. The smallest grid the machine has, so there is
    //                      no audible lag, and it is still a boundary, so
    //                      nothing is chopped mid-step.
    //
    // Stopped, there is no step boundary coming and nothing to be graceful
    // about, so it applies at once.
    if (running) poly_resync_ = true;
    else         memset(poly_pos_, 0, sizeof(poly_pos_));
  }

  // Settle a master-polymeter change made under the clock. Returns true if it
  // actually wrote, so the caller knows to re-dump. Refuses while running for
  // the same reason ApplyMasterPoly defers: this is the part that halts.
  bool ServiceMasterPoly() {
    if (!master_poly_owed_ || running) return false;
    const uint16_t v = master_poly ? 0x0FFF : 0x0000;
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) patch_poly_(p, v);
    master_poly_owed_ = false;
    return true;
  }
  bool MasterPolyOwed() const { return master_poly_owed_; }

  // PATTERN CLEAR. In the AB variation position the machine erases BOTH memories
  // at once: "When the AB position is set during the clearing operation, both A
  // and B memories will be erased at the same time" (OM p.16).
  void ClearPatternSlot(uint8_t slot, uint8_t var_mode) {
    if (var_mode == VAR_AB) {
      clear_one_(pat_index(0, slot));
      clear_one_(pat_index(1, slot));
    } else {
      clear_one_(pat_index(var_mode == VAR_B ? 1 : 0, slot));
    }
  }
  void ClearPattern(uint8_t p) { clear_one_(p); }

  void CopyPatternToBuf()    { copy_buf_ = cur(); }
  void PastePatternFromBuf() { cur() = copy_buf_; mark_pat_dirty(cur_pat); }
  void SetSwing(uint8_t sw)  { cur().swing = sw & 15; mark_pat_dirty(cur_pat); }

  void RotateInst(uint8_t inst, uint8_t sec, bool right) {
    uint16_t v = cur().steps[inst][sec];
    v = right ? (uint16_t)((v >> 1) | (v << 15)) : (uint16_t)((v << 1) | (v >> 15));
    cur().steps[inst][sec] = v;
    permute_attrs_(inst, sec, false, right);
    mark_pat_dirty(cur_pat);
  }
  void ReverseInst(uint8_t inst, uint8_t sec) {
    const uint16_t v = cur().steps[inst][sec];
    uint16_t r = 0;
    for (uint8_t i = 0; i < 16; ++i) if (v & (1u << i)) r |= (uint16_t)1 << (15 - i);
    cur().steps[inst][sec] = r;
    permute_attrs_(inst, sec, true, false);
    mark_pat_dirty(cur_pat);
  }
  void RandomizeInst(uint8_t inst, uint8_t sec, uint8_t density) {
    // The four panel detents expressed on the WIRE scale, so the panel tool and
    // the 0x2E contract run the ONE generator in rand_section_ rather than two
    // copies of the same loop. 40/90/140/200 out of 256 is 20/45/70/100 out of
    // 128: the same probability to the bit. The drawn BITS differ (the wire
    // compares the low 7 of the byte, this used to compare all 8), so a given
    // PRNG stream now hits different steps than it did before c00f0b4. Nothing
    // pins that, and a generator whose exact stream is load-bearing would be a
    // defect in itself.
    //
    // len is 16, not the section length: the panel tool has always replaced the
    // whole section, and steps past seclen keep a value that becomes visible if
    // the section is later lengthened. Preserved deliberately, not inherited.
    static const uint8_t thr[4] = { 20, 45, 70, 100 };
    rand_section_(cur(), inst, sec, thr[density & 3], NUM_STEP_BTNS);
    mark_pat_dirty(cur_pat);
  }

  // SysEx 0x2E RANDOMIZE: the ratified fleet contract (claim-014, `343fbb9`).
  // Returns the 0x14 ack status, 0 ok / 2 out of range; bad length is 1 and is
  // the caller's, since only the caller can see the message.
  //
  // The range rule lives HERE, in one place, the way the 303's HostRandomize
  // does, so the wire handler cannot drift from the panel path.
  //
  //   lane 0..11  ONE instrument, and lane 0 is AC on this machine per the 0x16
  //               numbering (AC=0, BD=1 ... CH=11).
  //   lane 0x7F   every lane.
  //   lane 12..0x7E  REJECTED, and NOTHING changes. A rejected lane that
  //               silently widens to the whole pattern is the 606 defect the
  //               contract forbids and it is the load-bearing check. Both range
  //               tests run before any mutation, so a rejected payload cannot
  //               leave a half-randomized pattern behind.
  //
  // density 0..127 is the chance a step is a hit: 0 clears the lane, 127 fills
  // it. This is the WIRE contract, which is the 303's (COMMON rule 5), and it
  // is deliberately not the panel DENSITY tool's four detents.
  //
  // seed 0 draws from the machine's own PRNG. A nonzero seed is deterministic
  // AND leaves the entropy stream untouched, so a G4 capture repeats and taking
  // one does not perturb the next unseeded randomize.
  //
  // `scratch` is caller-owned and touched ONLY when `pat` is not the playing
  // slot, the same shape and the same reason as SetPatternStep: a 228-byte
  // record does not belong on this machine's stack above a flash write.
  uint8_t HostRandomize(uint8_t pat, uint8_t lane, uint8_t density,
                        uint16_t seed, Pattern &scratch) {
    if (pat >= NUM_PATTERNS) return 2;
    if (lane >= NUM_INSTRUMENTS && lane != LANE_ALL) return 2;
    if (pat == cur_pat) {
      randomize_into_(cur(), lane, density, seed);
      mark_pat_dirty(cur_pat);
    } else {
      ReadPattern(pat, scratch);
      randomize_into_(scratch, lane, density, seed);
      WritePattern(pat, scratch);
    }
    return 0;
  }

  // win 0 is OFF, not a clamped 1: the caller that has no window and the caller
  // that wants the tool off are the same caller, and main.cpp used to branch
  // between ResliceOn and ResliceOff to say it.
  void ResliceOn(uint8_t win) {
    if (win > NUM_STEP_BTNS) win = NUM_STEP_BTNS;
    if (!reslice_len && win) { reslice_start_ = pos_; reslice_off_ = 0; }
    reslice_len = win;
  }
  void ResliceOff() { reslice_len = 0; }
  bool    ResliceActive() const { return reslice_len != 0; }
  uint8_t ResliceWindow() const { return reslice_len; }

  void ArpAdd(uint8_t inst) { if (arp_len < 16) arp[arp_len++] = inst; }
  void ArpClear() { arp_len = 0; arp_pos_ = 0; }

  void TriggerNow(uint16_t mask, bool accent) {
    mask &= (uint16_t)~1u;
    if (!mask) return;
    gather_mask_   |= mask;
    gather_accent_ |= accent;
    gather_active_  = true;
    gather_last_    = micros();
  }

  // ---- INTRO / FILL IN ------------------------------------------------------
  // TAP while STOPPED arms the selected INTRO/FILL IN to play first, as an
  // intro; pressing it again cancels. TAP while RUNNING queues a one-measure
  // fill that takes over at the end of the current measure (OM pp.11-12).
  void TapFill() {
    if (running) fill_request = true;
    else         intro_armed = !intro_armed;
  }
  void SelectFillSlot(uint8_t slot) {
    if (slot >= NUM_BASIC && slot < NUM_SLOTS) fill_slot = slot;
  }

  // Point the editor/writer at a slot, INCLUDING an intro/fill slot. SelectSlot
  // deliberately refuses those (pressing an INTRO/FILL button in a play mode
  // arms the fill, it does not change the basic rhythm), but PATTERN CLEAR and
  // the write modes must be able to address all 16.
  void SetEditSlot(uint8_t slot) {
    if (slot >= NUM_SLOTS) return;
    sel_slot = slot;
    if (slot_is_fill(slot)) fill_slot = slot;
    fill_now_ = false;
    resolve_pattern_();
    latch_prescale();
  }
  bool FillPlaying() const { return fill_now_; }

  // The MANUAL PLAY / PLAY step-LED row: which slots are SPOKEN FOR, and which
  // one is sounding. Lives here rather than in build_frame so it can be tested
  // on the host at all -- main.cpp does not link into the host harness, and a
  // test that re-implemented this would be asserting against a copy.
  //
  // claim-115. The chain and the queue were NEVER DRAWN. build_frame lit
  // sel_slot and the fill and nothing else, so chaining four patterns lit one
  // LED and queueing the next pattern lit none at all: the panel gave the
  // player no way to see what they had just asked for. Reported as "i cant see
  // the patterns light up red when they are changed" and "pressing the next
  // pattern play it is blank".
  //
  // The rule is the stock machine's priority display (OM p.11, "the rhythm that
  // will play first FLASHES, the other selected one stays lit") carried to the
  // whole set: everything spoken for is LIT, and the one actually sounding
  // BLINKS out of that set. That is the player's own description, "lit red
  // until it plays then it blinks".
  //
  // The blink slot is CLEARED before it is re-set. It is normally also a chain
  // member (chain_pos points at it), so leaving its steady bit in would hold it
  // permanently on and the blink would never be visible -- which is the whole
  // symptom this fixes, reintroduced one line later.
  uint16_t play_row(bool blink_on) const {
    uint16_t f = 0;
    const bool    fill_first = intro_armed || FillPlaying();
    const uint8_t flashing   = fill_first ? fill_slot : sel_slot;
    const uint8_t steady     = fill_first ? sel_slot
                                          : (intro_armed ? fill_slot : 0xFF);
    if (steady < NUM_SLOTS) f |= (uint16_t)1 << steady;
    for (uint8_t i = 0; i < chain_len && i < CHAIN_MAX; ++i)
      f |= (uint16_t)1 << (chain[i] & (NUM_SLOTS - 1));
    for (uint8_t i = 0; i < queue_len && i < CHAIN_MAX; ++i)
      f |= (uint16_t)1 << (queued[i] & (NUM_SLOTS - 1));
    if (flashing < NUM_SLOTS) {
      f &= (uint16_t)~((uint16_t)1 << flashing);
      if (blink_on) f |= (uint16_t)1 << flashing;
    }
    return f;
  }

  // ---- pattern selection ----------------------------------------------------
  // The 808 finishes the measure it is playing before switching, "however, if
  // the new Rhythm is selected on the first beat of the measure the new Rhythm
  // will begin on the second beat of that same measure" (OM p.11). pos_ == 0 is
  // that first beat.
  void SelectSlot(uint8_t slot) {
    if (slot >= NUM_SLOTS) return;
    if (slot_is_fill(slot)) { SelectFillSlot(slot); return; }
    if (!running) {
      sel_slot  = slot;
      chain_len = 0;
      chain_pos = 0;
      queue_len = 0;
      resolve_pattern_();
      latch_prescale();
    } else if (pos_ == 0) {
      sel_slot  = slot;
      chain_len = 0;
      queue_len = 0;
      resolve_pattern_();
      latch_prescale();
      if (track_write) record_measure_();   // rewrite this measure's entry
    } else {
      queued[0] = slot;
      queue_len = 1;
    }
  }
  void SelectChain(const uint8_t *slots, uint8_t n) {
    if (n > CHAIN_MAX) n = CHAIN_MAX;
    if (!running) {
      memcpy(chain, slots, n);
      chain_len = n;
      chain_pos = 0;
      sel_slot  = chain[0];
      resolve_pattern_();
      latch_prescale();
    } else {
      memcpy(queued, slots, n);
      queue_len = n;
    }
  }

  // ---- rhythm-track editing -------------------------------------------------
  void TrackTruncate(uint8_t new_len) {
    Track &t = track[cur_track];
    if (new_len < t.len) { t.len = new_len; mark_trk_dirty(cur_track); }
  }
  void TrackClear() {
    track[cur_track].Clear();
    mark_trk_dirty(cur_track);
  }

  void mark_pat_dirty(uint8_t p) { if (p == cache_idx_) cache_dirty_ = true; }
  void mark_trk_dirty(uint8_t t) { dirty_trk |= (uint16_t)1 << t; }

  uint16_t fired() const { return fired_; }
  bool fired_accent() const { return fired_accent_; }
  bool PulseActive() const { return pulse_on_; }
  uint8_t position() const { return pos_; }

 private:
  uint8_t  tick_       = 0;
  uint8_t  pos_        = 0;     // position within the measure, 0..total_len-1
  bool     first_      = true;
  bool     pp_back_    = false;
  // claim-042. The note bits live in steps[inst][sec], but ratchet and
  // probability are SPARSE maps keyed by ABSOLUTE step, so a permutation that
  // moves only the bits strands them: the attribute stays on a step the hit has
  // left. It is silent at the time (engine.h reads ratchet_get only for
  // instruments that actually fire) and surfaces later, when some unrelated
  // edit puts a hit back on that step and a ratchet the user never set for it
  // starts firing. A303's claim-031 is the same invariant broken a different
  // way. One helper for both maps because their layout is identical:
  // 0xFF = free slot, high nibble of val = instrument.
  static void permute_map_(uint8_t *step, const uint8_t *val, uint8_t n,
                           uint8_t inst, uint8_t base, bool reverse, bool right) {
    for (uint8_t k = 0; k < n; ++k) {
      if (step[k] == 0xFF || (uint8_t)(val[k] >> 4) != inst) continue;
      const uint8_t loc = (uint8_t)(step[k] - base);
      if (loc >= 16) continue;      // unsigned wrap: not in this section
      const uint8_t nl = reverse ? (uint8_t)(15 - loc)
                                 : (uint8_t)((right ? loc + 15 : loc + 1) & 15);
      step[k] = (uint8_t)(base + nl);
    }
  }
  // Free every slot of `inst` in this section whose step carries no hit in
  // `live`. Same claim-042 stranding, reached by replacement instead of by
  // permutation.
  static void prune_map_(uint8_t *step, const uint8_t *val, uint8_t n,
                         uint8_t inst, uint8_t base, uint16_t live) {
    for (uint8_t k = 0; k < n; ++k) {
      if (step[k] == 0xFF || (uint8_t)(val[k] >> 4) != inst) continue;
      const uint8_t loc = (uint8_t)(step[k] - base);
      if (loc >= 16) continue;
      if (!(live & ((uint16_t)1 << loc))) step[k] = 0xFF;
    }
  }
  // The 0x2E generator. Only LIVE steps are touched: a section with seclen 0 is
  // not part of the measure and is left exactly as it was, which is the same
  // scope the 303 uses (it walks `s.length`). Steps past a section's length keep
  // whatever they held.
  //
  // noinline so the loop body cannot migrate into handle_sysex, which is the
  // root of the deepest chain on the loop root.
  void __attribute__((noinline))
  randomize_into_(Pattern &p, uint8_t lane, uint8_t density, uint16_t seed) {
    const uint32_t save = rng_;
    if (seed) rng_ = 0x2E000000UL | (uint32_t)seed;
    const uint8_t lo = (lane == LANE_ALL) ? 0 : lane;
    const uint8_t hi = (lane == LANE_ALL) ? (uint8_t)(NUM_INSTRUMENTS - 1) : lane;
    for (uint8_t i = lo; i <= hi; ++i) {
      for (uint8_t sec = 0; sec < NUM_SECTIONS; ++sec) {
        uint8_t len = p.seclen[sec];
        if (!len) continue;
        if (len > NUM_STEP_BTNS) len = NUM_STEP_BTNS;
        rand_section_(p, i, sec, density, len);
      }
    }
    // A seeded run must not consume the machine's entropy, or the NEXT unseeded
    // randomize becomes a function of how many captures were taken.
    if (seed) rng_ = save;
  }

  // ONE generator, shared by the panel DENSITY tool and the 0x2E wire contract.
  // It was two copies of this loop and they cost 760 B of flash in a build with
  // 170 B of headroom above the 0xD700 app ceiling (measured at c00f0b4, where
  // the shipping env stopped linking). density is the WIRE scale 0..127.
  //
  // noinline for the reason every other Pattern-touching helper here is: it must
  // not migrate into handle_sysex, the root of the deepest chain on the loop
  // root, and it must not be emitted twice.
  void __attribute__((noinline))
  rand_section_(Pattern &p, uint8_t inst, uint8_t sec, uint8_t density,
                uint8_t len) {
    uint16_t v = p.steps[inst][sec];
    uint16_t m = 1;
    for (uint8_t k = 0; k < len; ++k, m = (uint16_t)(m << 1)) {
      // ONE draw, in its own statement. `|` does not order its operands, so two
      // draws in one expression make a seeded result a property of the compiler
      // rather than of the seed -- the 303's claim-055, which was a live bug and
      // not a style note.
      const uint8_t r = (uint8_t)(rng_u8() & 0x7F);
      if ((density >= 127) || (r < density)) v |= m;
      else                                   v &= (uint16_t)~m;
    }
    p.steps[inst][sec] = v;
    // Not a permutation: it REPLACES the section, so every attribute in it
    // described a hit that no longer exists and they all go (claim-042, and the
    // 303's claim-082). `live = 0`, not v, and the difference is the whole
    // point: keeping an attribute because randomize happened to re-hit that step
    // would hand a new random hit a ratchet the user set for a different one.
    // The G12 suite caught exactly that.
    prune_attrs_(p, inst, sec, 0);
  }

  // Takes the record explicitly: HostRandomize reaches a slot that is not the
  // playing one, and cur() would silently prune the WRONG pattern's attributes.
  void prune_attrs_(Pattern &p, uint8_t inst, uint8_t sec, uint16_t live) {
    const uint8_t base = (uint8_t)(sec * 16);
    prune_map_(p.rstep, p.rval, RATCHET_SLOTS, inst, base, live);
    prune_map_(p.pstep, p.pval, PROB_SLOTS,    inst, base, live);
  }
  void permute_attrs_(uint8_t inst, uint8_t sec, bool reverse, bool right) {
    const uint8_t base = (uint8_t)(sec * 16);
    Pattern &p = cur();
    permute_map_(p.rstep, p.rval, RATCHET_SLOTS, inst, base, reverse, right);
    permute_map_(p.pstep, p.pval, PROB_SLOTS,    inst, base, reverse, right);
  }

  uint32_t rng_        = 0x808a55edUL;
  uint8_t  active_pre_ = 2;
  bool     fill_now_   = false; // this measure is playing an intro/fill
  bool     master_poly_owed_ = false;  // claim-118: store patch owed to a stop
  bool     poly_resync_      = false;  // claim-118: realign on the next step
  uint8_t  ab_phase_   = 0;     // AB variation: 0 = A measure, 1 = B measure
  uint8_t  ind_var_    = VAR_A;
  bool     ind_part2_  = false;

  uint8_t rng_u8() { rng_ = rng_ * 1664525UL + 1013904223UL; return (uint8_t)(rng_ >> 24); }

  // Musical position -> sounding position for one direction. Split out of
  // advance_step so a VOICE can ask the same question the pattern does
  // (claim-119); the master path is byte-for-byte what it was.
  //
  // DIR_RND draws per call ON PURPOSE: two voices both set to random should not
  // walk in lockstep, which is what sharing one draw would do. DIR_PP reads the
  // shared pp_back_, so a ping-pong voice turns around with the pattern rather
  // than keeping a private phase nothing on the panel could show.
  uint8_t dir_pos_(uint8_t dir, uint8_t p, uint8_t len) {
    switch (dir) {
      case DIR_BCK: return (uint8_t)(len - 1 - p);
      case DIR_PP:  return pp_back_ ? (uint8_t)(len - 1 - p) : p;
      case DIR_RND: return (uint8_t)(rng_u8() % len);
      default:      return p;
    }
  }

  // A cleared pattern is a CONSTANT, so the caller has no reason to hold one.
  // Building it on the stack to hand to the store cost 228 B above a flash
  // write; the store writes the same bytes from its own record buffer
  // (serialize_cleared_pattern, pinned byte-for-byte against
  // serialize_pattern(Clear()) in tools/hosttest/record_fields.cpp).
  void clear_one_(uint8_t p) {
    Pattern *r = resident_(p);
    if (r) {
      r->Clear();
      if (r == &cache_) cache_dirty_ = true;
      return;
    }
    if (pat_clear_) pat_clear_(p);
  }

  // ASK for the layout fields (section lengths + prescale) to be copied onto the
  // slot's other variation, which the machine keeps identical between A and B.
  // ServiceLayoutSync does the copy, from the mainline, off this call chain.
  //
  // It used to do the copy here, reading the sibling's whole 228-byte record
  // into a local to touch five bytes of it. That put a 235-byte frame on the
  // SysEx path (handle_sysex -> SelectSlot -> latch_prescale -> here) and was
  // the whole of claim-080's remaining overflow: 629 B peak against a 621 B
  // usable budget, 663 minus the worst interrupt frame.
  //
  // A bitmask, NOT a saved copy of the five values. Two DIFFERENT siblings can
  // be requested in one pass (two 0x1D messages in one midi_rx_poll drain) and
  // a single saved request loses one; a bitmask accumulates and cannot. It also
  // needs no values saved, because the source is always the request's PARTNER,
  // read at service time, and the partner is normally cur_pat, which is already
  // in cache_.
  //
  // Clearing cur_pat's own bit is what makes service ORDER not matter: the
  // pattern just edited is the authority for its slot, so any earlier request to
  // overwrite IT is stale by definition. Without this, editing A then B leaves
  // both bits set and the result depends on which is popped first.
  // A pattern index is var * NUM_SLOTS + slot and NUM_SLOTS is a power of two,
  // so the OTHER variation of the same slot is one XOR rather than a divide, a
  // modulo and a multiply. Asserted, because it silently addresses the wrong
  // pattern if the slot count ever stops being a power of two.
  static_assert((NUM_SLOTS & (NUM_SLOTS - 1)) == 0, "sibling_() assumes NUM_SLOTS is 2^n");
  static uint8_t sibling_(uint8_t idx) { return (uint8_t)(idx ^ NUM_SLOTS); }

  // One variable 32-bit shift, not two. The AVR has no barrel shifter so each is
  // a loop, and the sibling's bit is a fixed distance from cur_pat's: NUM_SLOTS
  // places up or down, which is a byte swap on a 32-bit mask.
  void mirror_layout_to_sibling_() {
    const uint32_t mine = (uint32_t)1 << cur_pat;
    const uint32_t sib  = (cur_pat < NUM_SLOTS) ? (mine << NUM_SLOTS) : (mine >> NUM_SLOTS);
    layout_sync_req_ = (layout_sync_req_ & ~mine) | sib;
  }

  // A pattern that is RESIDENT (the cache, or the one pending write) is answered
  // from RAM and never reaches the store. That is the normal case for a field
  // operation's source -- the mirror's source is cur_pat, which IS the cache --
  // so the common path costs no buffer at either end.
  Pattern *resident_(uint8_t idx) {
    if (idx == cache_idx_)                    return &cache_;
    if (pend_valid_ && idx == pend_idx_)      return &pend_pat_;
    return nullptr;
  }

  // Typed accessors for the two field windows, not a generic byte-offset
  // dispatcher: the offsets only have to be spoken to the STORE, where the
  // record really is a byte array. A resident pattern is a struct and is touched
  // as one. (The generic version compiled to 1.1 KB more flash and pushed
  // combined-nat-test's app into the flash service's staging window.)
  bool peek_layout_(uint8_t idx, uint8_t *lay) {
    const Pattern *p = resident_(idx);
    if (!p) {
      if (!pat_peek_ || !pat_peek_(idx, PATTERN_LAYOUT_OFF, lay, PATTERN_LAYOUT_LEN))
        return false;
      // The clamp deserialize_pattern applies, for the same reason: a section
      // length out of range would be mirrored onto the sibling verbatim.
      for (uint8_t s = 0; s < NUM_SECTIONS; ++s)
        if (lay[s] > NUM_STEP_BTNS) lay[s] = NUM_STEP_BTNS;
      return true;
    }
    memcpy(lay, p->seclen, PATTERN_LAYOUT_LEN);   // contiguous, asserted in pattern.h
    return true;
  }
  void patch_layout_(uint8_t idx, const uint8_t *lay) {
    Pattern *p = resident_(idx);
    if (!p) {
      // Not resident. With no store there is no record to patch, so the request
      // is dropped rather than fabricating an empty pattern to carry five bytes
      // -- which is what the record-shaped version did, and it wrote a CLEARED
      // sibling over a slot the store had never seen.
      if (pat_patch_) pat_patch_(idx, PATTERN_LAYOUT_OFF, lay, PATTERN_LAYOUT_LEN);
      return;
    }
    if (memcmp(p->seclen, lay, PATTERN_LAYOUT_LEN) == 0) return;   // already agrees
    memcpy(p->seclen, lay, PATTERN_LAYOUT_LEN);
    if (p == &cache_) cache_dirty_ = true;
  }
  void patch_poly_(uint8_t idx, uint16_t v) {
    Pattern *p = resident_(idx);
    if (!p) {
      const uint8_t w[PATTERN_POLY_LEN] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
      if (pat_patch_) pat_patch_(idx, PATTERN_POLY_OFF, w, PATTERN_POLY_LEN);
      return;
    }
    if (p->poly == v) return;
    p->poly = v;
    if (p == &cache_) cache_dirty_ = true;
  }

  // Point cur_pat at the pattern the panel currently addresses.
  void resolve_pattern_() {
    if (fill_now_) {
      cur_pat = pat_index(if_var_b ? 1 : 0, fill_slot);
      return;
    }
    uint8_t v;
    if (variation == VAR_AB)     v = ab_phase_;
    else if (variation == VAR_B) v = 1;
    else                         v = 0;
    cur_pat = pat_index(v, sel_slot);
  }

  void record_measure_() {
    Track &t = track[cur_track];
    if (t.len >= TRACK_MAX_PATS) {
      // "Once you have used up the memory available in one position the TR-808
      // automatically switches to the next track" (OM p.4).
      if (cur_track + 1 < NUM_TRACKS) {
        cur_track = (uint8_t)(cur_track + 1);
        track[cur_track].Clear();
        mark_trk_dirty(cur_track);
      } else {
        return;   // last track full: stop recording rather than wrap
      }
    }
    Track &d = track[cur_track];
    d.set_pat(d.len++, sel_slot);
    mark_trk_dirty(cur_track);
  }

  uint8_t eff_len_() {
    if (global_len) return global_len > MAX_STEPS ? MAX_STEPS : global_len;
    return cur().total_len();
  }

  // Playback position -> absolute step index.
  //
  // Normally the measure is the pattern's sections concatenated, so this walks
  // the section lengths (OM Fig. 8). While the GLOBAL LENGTH override is on it
  // does not: the override says every pattern is N steps, and a single number
  // cannot describe a part split, so the measure becomes a flat run of N steps
  // from step 0 and the part layout is bypassed for as long as it is set.
  //
  // For the ordinary case those are the same thing — a stock pattern's 1st PART
  // is 16 steps and the sections sit on 16-step boundaries, so position n is
  // step n either way. They only diverge when a 1st PART is SHORTER than 16,
  // where the layout skips the unused tail of the section and the override
  // plays straight through it. That is the override doing its job rather than a
  // rounding error: it is the 606's flat-pattern model, which is what makes the
  // step data past a short part reachable without editing the part lengths.
  // Clearing the override restores the machine's own layout untouched.
  //
  // Both branches saturate rather than propagate an error, and they saturate
  // the same way they always did: the override clamps a silly position to the
  // last step, and the layout walk answers step 0 for a position no section
  // covers. What changed with claim-051 is only that pos_to_step() now SAYS it
  // failed instead of returning a value indistinguishable from step 0, so this
  // is the one place that decides what to play and the next caller inherits a
  // detectable error rather than a silent wrong step. Sounding behaviour is
  // unchanged.
  uint8_t pos_to_abs_(uint8_t pos) {
    if (!global_len) {
      const uint8_t s = cur().pos_to_step(pos);
      return s == POS_INVALID ? 0 : s;
    }
    return pos >= MAX_STEPS ? (uint8_t)(MAX_STEPS - 1) : pos;
  }

  // Ticks the current step lasts. SWING lengthens on-beat (even pos_) steps and
  // shortens off-beat ones by the same amount, pushing the off-beat later while
  // keeping each pair's total — and so the tempo — constant.
  uint8_t step_len_() const {
    const uint8_t base = PRESCALE_TICKS[active_pre_];
    const uint8_t sw   = cache_.swing;
    if (!sw) return base;
    const uint8_t d = (uint8_t)((uint16_t)base * sw / 32);
    if (pos_ & 1) return (uint8_t)(base > d + 1 ? base - d : 2);
    return (uint8_t)(base + d);
  }

  bool     pulse_on_     = false;
  uint32_t pulse_t0_     = 0;
  uint16_t pulse_mask_   = 0;
  bool     pulse_accent_ = false;
  uint16_t gather_mask_   = 0;
  bool     gather_accent_ = false;
  bool     gather_active_ = false;
  uint32_t gather_last_   = 0;
  uint16_t fired_        = 0;
  bool     fired_accent_ = false;
  Pattern  copy_buf_;

  Pattern    cache_;
  uint8_t    cache_idx_   = 0xFF;
  bool       cache_dirty_ = false;
  PatReadFn  pat_read_    = nullptr;
  PatWriteFn pat_write_   = nullptr;
  PatPeekFn  pat_peek_  = nullptr;
  PatPatchFn pat_patch_ = nullptr;
  PatClearFn pat_clear_ = nullptr;
  bool       pend_valid_  = false;
  uint8_t    pend_idx_    = 0;
  Pattern    pend_pat_;

  // One bit per pattern: "this pattern's layout is stale, copy its partner's
  // onto it". Set by mirror_layout_to_sibling_, drained by ServiceLayoutSync.
  // NUM_PATTERNS is 32, so the mask is exactly a uint32_t; the static_assert
  // keeps that honest if the slot count ever moves.
  uint32_t   layout_sync_req_ = 0;
  static_assert(NUM_PATTERNS <= 32, "layout_sync_req_ is a 32-bit mask");

  void flush_() {
    if (cache_dirty_ && pat_write_ && cache_idx_ != 0xFF) pat_write_(cache_idx_, cache_);
    cache_dirty_ = false;
  }

  uint8_t  rat_rem_[NUM_INSTRUMENTS]      = {0};
  uint32_t rat_sp_us_[NUM_INSTRUMENTS]    = {0};
  uint32_t rat_next_us_[NUM_INSTRUMENTS]  = {0};
  bool     rat_accent_    = false;
  uint32_t tick_us_       = 20833;   // 24-PPQN period, 120 BPM default
  uint8_t  reslice_start_ = 0;
  uint8_t  reslice_off_   = 0;
  uint8_t  arp_pos_       = 0;
  uint8_t  poly_pos_[NUM_INSTRUMENTS] = {0};
  uint8_t  vtick_[NUM_INSTRUMENTS]    = {0};   // claim-124: ticks to this
  uint8_t  vpos_[NUM_INSTRUMENTS]     = {0};   // voice's next step, and where

  void latch_prescale() {
    if (pending_prescale != 0xFF) {
      cur().prescale = pending_prescale;
      mark_pat_dirty(cur_pat);
      pending_prescale = 0xFF;
      mirror_layout_to_sibling_();
    }
    active_pre_ = cur().prescale & 3;
  }

  void advance_step() {
    // The armed polymeter realign (claim-118), taken HERE because this is the
    // step boundary: it lands before fire_step, so the very next step to sound
    // is already aligned, and every voice comes back together on the same edge
    // rather than wherever the press happened to fall inside a step.
    if (poly_resync_) {
      memset(poly_pos_, 0, sizeof(poly_pos_));
      poly_resync_ = false;
    }
    uint8_t len = eff_len_();
    if (first_) {
      first_ = false;
      pos_   = 0;
    } else if (++pos_ >= len) {
      on_wrap();
      pos_ = 0;
      if (play_dir == DIR_PP) pp_back_ = !pp_back_;
    }
    len = eff_len_();
    const uint8_t p = pos_ >= len ? (uint8_t)(len - 1) : pos_;

    // Map the musical position to the sounding position by play direction, then
    // through the section layout to an absolute step index.
    uint8_t q = dir_pos_(play_dir, p, len);
    if (reslice_len) {
      reslice_off_ = (uint8_t)((reslice_off_ + 1) % reslice_len);
      q = (uint8_t)((reslice_start_ + reslice_off_) % len);
    }

    const uint8_t s = pos_to_abs_(q);
    step = (int8_t)s;
    fire_step(s);
    step_advanced = true;

    {
      const Pattern &pp = cur();
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        const uint8_t li = pp.ilen[i] ? pp.ilen[i] : MAX_STEPS;
        if (++poly_pos_[i] >= li) poly_pos_[i] = 0;
      }
    }
  }

  void on_wrap() {
    const bool was_fill = fill_now_;

    // An intro/fill plays exactly once and then hands back to the basic rhythm.
    // Intros are not counted in the AUTO FILL IN measure tally (OM p.12), and
    // neither is a fill measure itself.
    if (!was_fill) ++measure;

    if (track_play) {
      const Track &t = track[cur_track];
      if (t.len > 0 && !was_fill) {
        if (++track_pos >= t.len) track_pos = 0;
        sel_slot = t.pat(track_pos);
      }
    } else if (queue_len > 0 && (chain_len == 0 || chain_pos + 1 >= chain_len)) {
      memcpy(chain, queued, queue_len);
      chain_len = (queue_len > 1) ? queue_len : 0;
      chain_pos = 0;
      sel_slot  = chain_len ? chain[0] : queued[0];
      queue_len = 0;
    } else if (chain_len > 0 && !was_fill) {
      if (++chain_pos >= chain_len) chain_pos = 0;
      sel_slot = chain[chain_pos];
    }

    // AB alternation advances one measure at a time, always starting on A
    // (OM p.12). A fill measure does not advance the phase.
    if (!was_fill && variation == VAR_AB) ab_phase_ ^= 1;

    // AUTO FILL IN: insert a fill every N measures, or on demand from TAP.
    const uint8_t every = FILL_MEASURES[fill_every < NUM_FILL_POS ? fill_every : 0];
    fill_now_ = fill_request || (!was_fill && every && (measure % every) == 0);
    fill_request = false;

    resolve_pattern_();
    latch_prescale();

    if (track_write && !fill_now_) record_measure_();
  }

  void fire_step(uint8_t s) {
    const Pattern &p = cur();
    memset(rat_rem_, 0, sizeof(rat_rem_));

    uint16_t mask;
    bool     accent = false;
    uint8_t  sis[NUM_INSTRUMENTS];
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) sis[i] = s;

    if (arp_len) {                        // ARP replaces the pattern
      const uint8_t a = arp[arp_pos_];
      arp_pos_ = (uint8_t)((arp_pos_ + 1) % arp_len);
      mask   = (a >= INST_BD && a < NUM_INSTRUMENTS) ? (uint16_t)(1u << a) : 0;
      accent = false;
    } else {
      mask = 0;
      // A voice with a direction override resolves its OWN sounding step from
      // the shared musical position, so it reads the pattern backwards (or at
      // random) while the rest of the machine runs forwards. Everything after
      // that -- polymeter length, probability, ratchets -- is unchanged and
      // operates on whatever step this voice landed on (claim-119).
      const uint8_t len_ = eff_len_();
      const uint8_t mp_  = pos_ >= len_ ? (uint8_t)(len_ - 1) : pos_;
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        // A voice on its own PRE SCALE sounds from tick_voices_ and must not
        // also be sounded here, or every one of its hits would double (claim-124).
        if (ipre[i] != 0xFF) { sis[i] = s; continue; }
        const uint8_t li = p.ilen[i];
        uint8_t base = s;
        if (idir[i] != 0xFF && idir[i] != play_dir && !reslice_len)
          base = pos_to_abs_(dir_pos_(idir[i], mp_, len_));
        uint8_t si = base;
        if (li) si = (uint8_t)((((p.poly >> i) & 1) ? poly_pos_[i] : base) % li);
        sis[i] = si;
        if (i == INST_AC) accent = p.step_get(i, si);
        else if (p.step_get(i, si)) mask |= (uint16_t)1 << i;
      }
      for (uint8_t k = 0; k < PROB_SLOTS; ++k) {
        if (p.pstep[k] == 0xFF) continue;
        const uint8_t i = (uint8_t)(p.pval[k] >> 4);
        const uint8_t v = (uint8_t)(p.pval[k] & 15);
        if (i >= NUM_INSTRUMENTS || p.pstep[k] != sis[i] || !v) continue;
        const bool hit = (i == INST_AC) ? accent : ((mask >> i) & 1);
        if (hit && rng_u8() >= (uint8_t)(v << 4)) {
          if (i == INST_AC) accent = false;
          else mask &= (uint16_t)~((uint16_t)1 << i);
        }
      }
    }

    mask &= (uint16_t)~mute_mask;
    if (mute_mask & 1) accent = false;

    fired_        = mask;
    fired_accent_ = accent;

    if (!arp_len && mask) {
      // Spacing = the step's real duration divided evenly among the hits,
      // from the measured tick period, so repeats sit in time under the
      // internal clock, rear sync and MIDI alike (see the note in Service).
      const uint32_t step_us = (uint32_t)step_len_() * tick_us_;
      const uint32_t now = micros();
      rat_accent_ = accent;
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        if (!((mask >> i) & 1)) continue;
        const uint8_t e = p.ratchet_get(i, sis[i]);
        if (!e) continue;
        const uint32_t sp = step_us / (uint32_t)(e + 1);
        rat_rem_[i]     = e;
        rat_sp_us_[i]   = sp;
        rat_next_us_[i] = now + sp;
      }
    }

    if (!mask && !accent) return;
    fire_pulse(mask, accent);
  }

  // Drive every data line to its value (indicator LEDs may idle a line HIGH, so
  // non-firing voices must be forced LOW, not left alone), AC raised only on
  // accented hits, then the common-trigger pulse.
  //
  // Accent is a SINGLE shared level: it sets the amplitude of the COMMON TRIG
  // pulse, so every voice firing in the window is accented together. A DAW chord
  // arrives as separate Note Ons landing inside one pulse window; restarting the
  // pulse per note would let an un-accented note pull the shared accent (and the
  // earlier note's data line) back down. Instead, merge into the live pulse and
  // keep pulse_t0_ so a chord cannot stretch it.
  void fire_pulse(uint16_t mask, bool accent) {
    if (pulse_on_) {
      pulse_mask_   |= mask;
      pulse_accent_ |= accent;
    } else {
      pulse_mask_   = mask;
      pulse_accent_ = accent;
      pulse_t0_     = micros();
      pulse_on_     = true;
    }
    for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
      digitalWrite(INSTRUMENT_PIN[i], (pulse_mask_ & ((uint16_t)1 << i)) ? HIGH : LOW);
#ifdef SUPEROS_LAMP_PROBE
    // The data window drives PE1 from the step mask, which is the level that
    // fights the lamp whenever the rim shot is not in it.
    lamp_probe_edge(g_lamp_sos,
                    (uint8_t)((pulse_mask_ >> INST_RS) & 1 ? HIGH : LOW),
                    (uint8_t)(ind_var_ == VAR_B ? HIGH : LOW));
#endif
    digitalWrite(AC_PIN, pulse_accent_ ? HIGH : LOW);
    // Short strobe: IC6 turns this into the ~1 ms the voices need. The data
    // lines stay put until Service() clears them TRIG_DATA_US later, so they
    // span IC6's output exactly as Figure 6 shows.
    digitalWrite(PI2_PIN, HIGH);
    delayMicroseconds(TRIG_STROBE_US);
    digitalWrite(PI2_PIN, LOW);
  }
};
