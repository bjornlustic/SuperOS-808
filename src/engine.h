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

static constexpr uint8_t CHAIN_MAX = 16;

// Play direction (DIRECTION tool). Runtime only, not persisted.
enum PlayDir : uint8_t { DIR_FWD = 0, DIR_BCK = 1, DIR_RND = 2, DIR_PP = 3 };

// Pattern storage is store-resident: only the active pattern lives in RAM, so 32
// patterns do not cost 32 x sizeof(Pattern) of SRAM. These hooks read/write one
// pattern to the block store; registered at boot via SetPatIO. Reads are cheap;
// a FLASH write halts the CPU for milliseconds, so the engine only writes while
// stopped (or defers to the next stop).
using PatReadFn  = bool (*)(uint8_t idx, Pattern &out);
using PatWriteFn = bool (*)(uint8_t idx, const Pattern &in);

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
  uint16_t mute_mask  = 0;      // bit per Instrument, 1 = muted (bit0 = accent)
  bool     reslice_on  = false;
  uint8_t  reslice_len = 0;
  uint8_t  arp[16];
  uint8_t  arp_len = 0;

  void SetPatIO(PatReadFn r, PatWriteFn w) { pat_read_ = r; pat_write_ = w; }

  void Init() {
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
    digitalWrite(LAMP_VAR_PIN,  var_showing == VAR_B ? HIGH : LOW);
    digitalWrite(LAMP_PART_PIN, part2 ? HIGH : LOW);
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
  Pattern &cur() {
    if (cache_idx_ != cur_pat) {
      if (!running) {
        flush_();
      } else if (cache_dirty_ && cache_idx_ != 0xFF) {
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
    }
    return cache_;
  }

  void ReadPattern(uint8_t idx, Pattern &out) {
    if (idx == cache_idx_)               { out = cache_;    return; }
    if (pend_valid_ && idx == pend_idx_) { out = pend_pat_; return; }
    if (!pat_read_ || !pat_read_(idx, out)) out.Clear();
  }
  void WritePattern(uint8_t idx, const Pattern &in) {
    if (idx == cache_idx_) { cache_ = in; cache_dirty_ = true; return; }
    if (!running && pat_write_) { pat_write_(idx, in); return; }
    pend_pat_ = in; pend_idx_ = idx; pend_valid_ = true;
  }

  void Flush() {
    flush_();
    if (pend_valid_) {
      if (pat_write_ && pend_idx_ != cache_idx_) pat_write_(pend_idx_, pend_pat_);
      pend_valid_ = false;
    }
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
    reslice_on  = false;
    measure     = 0;
    ab_phase_   = 0;
    memset(rat_rem_, 0, sizeof(rat_rem_));
    arp_pos_ = 0;
    memset(poly_pos_, 0, sizeof(poly_pos_));
    rng_ ^= (uint32_t)micros() | 1u;

    if (track_play) {
      track_pos = 0;
      if (track[cur_track].len > 0) sel_slot = track[cur_track].pat[0];
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
    track_write = false;
    step        = -1;
    tick_       = 0;
    queue_len   = 0;
    reslice_on  = false;
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
    if (tick_ == 0) {
      advance_step();
      tick_ = 1;
      return true;
    }
    uint16_t due = 0;
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
      if (rat_rem_[i] && tick_ == rat_next_[i]) {
        due |= (uint16_t)1 << i;
        rat_rem_[i]--;
        rat_next_[i] = (uint8_t)(rat_next_[i] + rat_sp_[i]);
      }
    }
    if (due) fire_pulse(due, rat_accent_);
    if (++tick_ >= step_len_()) tick_ = 0;
    return false;
  }

  // Call every main-loop pass: ends a pending trigger pulse, clears step_advanced.
  void Service() {
    step_advanced = false;
    if (pulse_on_ && (uint16_t)(micros() - pulse_t0_) >= TRIG_DATA_US) {
      digitalWrite(PI2_PIN, LOW);
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
        digitalWrite(INSTRUMENT_PIN[i], LOW);
      pulse_on_ = false;
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
  void SetPatternStep(uint8_t idx, uint8_t inst, uint8_t s, bool on) {
    if (idx == cur_pat) {
      if (on) SetStep(inst, s); else ClearStep(inst, s);
      return;
    }
    Pattern p;
    ReadPattern(idx, p);
    if (on) p.step_set(inst, s); else p.step_clr(inst, s);
    WritePattern(idx, p);
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

  bool master_poly = false;
  void ApplyMasterPoly(bool poly) {
    master_poly = poly;
    const uint16_t v = poly ? 0x0FFF : 0x0000;
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      Pattern t;
      ReadPattern(p, t);
      if (t.poly == v) continue;
      t.poly = v;
      WritePattern(p, t);
    }
    memset(poly_pos_, 0, sizeof(poly_pos_));
  }

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
    mark_pat_dirty(cur_pat);
  }
  void ReverseInst(uint8_t inst, uint8_t sec) {
    const uint16_t v = cur().steps[inst][sec];
    uint16_t r = 0;
    for (uint8_t i = 0; i < 16; ++i) if (v & (1u << i)) r |= (uint16_t)1 << (15 - i);
    cur().steps[inst][sec] = r;
    mark_pat_dirty(cur_pat);
  }
  void RandomizeInst(uint8_t inst, uint8_t sec, uint8_t density) {
    static const uint8_t thr[4] = { 40, 90, 140, 200 };
    uint16_t v = 0;
    for (uint8_t i = 0; i < 16; ++i) if (rng_u8() < thr[density & 3]) v |= (uint16_t)1 << i;
    cur().steps[inst][sec] = v;
    mark_pat_dirty(cur_pat);
  }

  void ResliceOn(uint8_t win) {
    if (win < 1) win = 1;
    if (win > NUM_STEP_BTNS) win = NUM_STEP_BTNS;
    if (!reslice_on) { reslice_start_ = pos_; reslice_off_ = 0; }
    reslice_on = true; reslice_len = win;
  }
  void ResliceOff() { reslice_on = false; }

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
  uint32_t rng_        = 0x808a55edUL;
  uint8_t  active_pre_ = 2;
  bool     fill_now_   = false; // this measure is playing an intro/fill
  uint8_t  ab_phase_   = 0;     // AB variation: 0 = A measure, 1 = B measure
  uint8_t  ind_var_    = VAR_A;
  bool     ind_part2_  = false;

  uint8_t rng_u8() { rng_ = rng_ * 1664525UL + 1013904223UL; return (uint8_t)(rng_ >> 24); }

  void clear_one_(uint8_t p) {
    if (p == cache_idx_) { cache_.Clear(); cache_dirty_ = true; }
    else { Pattern t; t.Clear(); WritePattern(p, t); }
  }

  // Copy the layout fields (section lengths + prescale) onto the slot's other
  // variation, which the machine keeps identical between A and B.
  void mirror_layout_to_sibling_() {
    const uint8_t sib = pat_index(pat_var(cur_pat) ^ 1, pat_slot(cur_pat));
    Pattern t;
    ReadPattern(sib, t);
    bool changed = false;
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s)
      if (t.seclen[s] != cache_.seclen[s]) { t.seclen[s] = cache_.seclen[s]; changed = true; }
    if (t.prescale != cache_.prescale) { t.prescale = cache_.prescale; changed = true; }
    if (changed) WritePattern(sib, t);
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
    d.pat[d.len++] = sel_slot;
    mark_trk_dirty(cur_track);
  }

  uint8_t eff_len_() { return cur().total_len(); }

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
  bool       pend_valid_  = false;
  uint8_t    pend_idx_    = 0;
  Pattern    pend_pat_;

  void flush_() {
    if (cache_dirty_ && pat_write_ && cache_idx_ != 0xFF) pat_write_(cache_idx_, cache_);
    cache_dirty_ = false;
  }

  uint8_t  rat_rem_[NUM_INSTRUMENTS]  = {0};
  uint8_t  rat_sp_[NUM_INSTRUMENTS]   = {0};
  uint8_t  rat_next_[NUM_INSTRUMENTS] = {0};
  bool     rat_accent_    = false;
  uint8_t  reslice_start_ = 0;
  uint8_t  reslice_off_   = 0;
  uint8_t  arp_pos_       = 0;
  uint8_t  poly_pos_[NUM_INSTRUMENTS] = {0};

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
    uint8_t q;
    switch (play_dir) {
      case DIR_BCK: q = (uint8_t)(len - 1 - p); break;
      case DIR_PP:  q = pp_back_ ? (uint8_t)(len - 1 - p) : p; break;
      case DIR_RND: q = (uint8_t)(rng_u8() % len); break;
      default:      q = p; break;
    }
    if (reslice_on && reslice_len) {
      reslice_off_ = (uint8_t)((reslice_off_ + 1) % reslice_len);
      q = (uint8_t)((reslice_start_ + reslice_off_) % len);
    }

    const uint8_t s = cur().pos_to_step(q);
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
        sel_slot = t.pat[track_pos];
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
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        const uint8_t li = p.ilen[i];
        uint8_t si = s;
        if (li) si = (uint8_t)((((p.poly >> i) & 1) ? poly_pos_[i] : s) % li);
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
      const uint8_t base = step_len_();
      rat_accent_ = accent;
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        if (!((mask >> i) & 1)) continue;
        const uint8_t e = p.ratchet_get(i, sis[i]);
        if (!e) continue;
        uint8_t sp = (uint8_t)(base / (e + 1));
        if (sp < 1) sp = 1;
        rat_rem_[i]  = e;
        rat_sp_[i]   = sp;
        rat_next_[i] = sp;
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
    digitalWrite(AC_PIN, pulse_accent_ ? HIGH : LOW);
    // Short strobe: IC6 turns this into the ~1 ms the voices need. The data
    // lines stay put until Service() clears them TRIG_DATA_US later, so they
    // span IC6's output exactly as Figure 6 shows.
    digitalWrite(PI2_PIN, HIGH);
    delayMicroseconds(TRIG_STROBE_US);
    digitalWrite(PI2_PIN, LOW);
  }
};
