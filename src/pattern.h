// SuperOS-808 — pattern & rhythm-track data model
//
// The model is the TR-808's own, read off the owner's manual (pp. 3-5, 11-19).
//
// PATTERNS
//   16 STEP buttons x 2 variation modes (A / B) = 32 rhythm patterns, each one
//   measure long. The buttons split into two labelled groups:
//     buttons  1-12  BASIC RHYTHM   1-12
//     buttons 13-16  INTRO/FILL IN  1-4
//   Which variation a button addresses comes from BASIC VARIATION for the basic
//   group and from I/F VARIATION for the intro/fill group. So:
//       absolute index = variation * 16 + slot        (slot 0..15, var 0=A 1=B)
//
// PARTS (the 808's step-count model)
//   A measure is programmed as a 1st PART and a 2nd PART of up to 16 steps each,
//   selected on the MODE dial, and they always play back concatenated as one
//   measure — "the 1st and 2nd Parts cannot be played separately" (OM p.18).
//   The two parts may have DIFFERENT step counts (16+4 = 20 steps, 10+10 = 20,
//   etc. — OM Fig. 8), which is how odd meters are written. A cleared pattern is
//   1st Part = 16 steps, 2nd Part = 0.
//
//   SuperOS keeps that exactly and extends it: seclen[] carries four sections
//   instead of two, so a pattern can run to 64 steps. Sections 3 and 4 default
//   to 0 steps, so a stock-programmed pattern behaves identically; they are only
//   reachable through the SECTION tool.
//
// SHARED BETWEEN A AND B
//   "Setting a PRE SCALE and Step number in one mode will always make it the
//   same in the other" (OM p.16). Section lengths and prescale are therefore
//   mirrored across the A/B pair of a slot whenever either is written — see
//   Engine::SetSectionLength / ArmPrescale.
//
// ACCENT is global per step: an accented step raises AC (PF3), which sets the
// amplitude of that step's shared COMMON TRIG pulse for every voice firing in
// it. That is a property of the hardware, not a design choice.
#pragma once
#include <Arduino.h>
#include <string.h>
#include "controls.h"   // Instrument enum (INST_AC = 0 .. INST_CH = 11)

static constexpr uint8_t NUM_STEP_BTNS  = 16;   // physical step buttons / LEDs
static constexpr uint8_t NUM_SECTIONS   = 4;    // 16-step sections (stock uses 2)
static constexpr uint8_t MAX_STEPS      = NUM_STEP_BTNS * NUM_SECTIONS;  // 64
static constexpr uint8_t NUM_SLOTS      = 16;   // step buttons 1-16
static constexpr uint8_t NUM_BASIC      = 12;   // buttons 1-12  = BASIC RHYTHM
static constexpr uint8_t NUM_FILLS      = 4;    // buttons 13-16 = INTRO/FILL IN
static constexpr uint8_t NUM_PATTERNS   = NUM_SLOTS * 2;   // A + B = 32
static constexpr uint8_t NUM_TRACKS     = 12;   // 12 rhythm tracks
static constexpr uint8_t TRACK_MAX_PATS = 64;   // 64 measures each (768 total)
static constexpr uint8_t PROB_SLOTS     = 40;
static constexpr uint8_t RATCHET_SLOTS  = 16;

// Slot 12..15 are the INTRO/FILL IN buttons.
static inline bool slot_is_fill(uint8_t slot) { return slot >= NUM_BASIC; }

// Absolute pattern index from panel coordinates.
static inline uint8_t pat_index(uint8_t var, uint8_t slot) {
  return (uint8_t)((var & 1) * NUM_SLOTS + (slot & 15));
}
static inline uint8_t pat_var(uint8_t idx)  { return (uint8_t)(idx / NUM_SLOTS); }
static inline uint8_t pat_slot(uint8_t idx) { return (uint8_t)(idx % NUM_SLOTS); }

// PRE SCALE position (0..3 = panel "1".."4") -> tempo-clock ticks per step.
// The 808 defines the quarter note as 24 clocks, and PRE SCALE sets "how many
// steps there will be for each beat" (OM p.15). Values are pinned by the
// manual's own worked examples:
//   PRE SCALE 1: OM p.16 programs a 12-step measure in 4/4  -> 3 steps/beat
//   PRE SCALE 3: OM p.13/p.15, 16 steps in 4/4, 12 in 3/4   -> 4 steps/beat
//   PRE SCALE 4: OM p.17, "thirty second notes ... to
//                complete a 4/4 measure" = 32 steps         -> 8 steps/beat
//   PRE SCALE 2: the remaining TR subdivision, 16th triplets -> 6 steps/beat
// so ticks per step = 24 / steps-per-beat:
//
// INDEXED BY DETENT, NOT BY THE PANEL CODE. Which raw encoder code each detent
// produces is PanelMap::pre_code in controls.h, and that is the table to fix if
// the dial plays the wrong subdivision. Swapping entries here instead gives
// identical panel behaviour while silently breaking the two OM anchors above,
// so it looks like a fix and is not one (see the note on pre_code).
static const uint8_t PRESCALE_TICKS[4] = { 8, 4, 6, 3 };
// Steps per beat, same order — used by the editor and the MIDI clock maths.
static const uint8_t PRESCALE_STEPS_PER_BEAT[4] = { 3, 6, 4, 8 };

// A pattern: one 16-bit word per instrument per 16-step section.
// Step index s (0..63): section = s >> 4, bit = s & 15.
struct Pattern {
  uint16_t steps[NUM_INSTRUMENTS][NUM_SECTIONS];  // [inst][section], bit = step
  // Steps used in each section. seclen[0] = 1st PART, seclen[1] = 2nd PART;
  // [2] and [3] are the SuperOS extension and default to 0. Playback walks the
  // sections in order, seclen[n] steps each, so the measure is their sum.
  uint8_t  seclen[NUM_SECTIONS];
  uint8_t  prescale;                // 0..3 (PRE SCALE dial value)
  uint8_t  swing;                   // 0 = off .. 15 = max (SuperOS)
  // RATCHET: sparse (instrument, step) -> extra-hit count 1..3.
  uint8_t  rstep[RATCHET_SLOTS];    // step 0..63; 0xFF = slot free
  uint8_t  rval[RATCHET_SLOTS];     // (instrument << 4) | count
  // PROBABILITY: sparse (instrument, step) -> play chance in 16ths 1..15.
  uint8_t  pstep[PROB_SLOTS];
  uint8_t  pval[PROB_SLOTS];
  uint8_t  ilen[NUM_INSTRUMENTS];   // per-instrument loop length, 0 = follow
  uint16_t poly;                    // bit per instrument: 1 = free-running

  // A cleared pattern is the machine's own default: 1st Part 16 steps, 2nd Part
  // 0 (OM p.18), PRE SCALE left at whatever the dial commits next.
  void Clear() {
    memset(steps, 0, sizeof(steps));
    seclen[0] = NUM_STEP_BTNS;
    seclen[1] = seclen[2] = seclen[3] = 0;
    prescale = 2;                   // panel "3" = 16ths, the common default
    swing = 0;
    memset(rstep, 0xFF, sizeof(rstep));
    memset(rval, 0, sizeof(rval));
    memset(pstep, 0xFF, sizeof(pstep));
    memset(pval, 0, sizeof(pval));
    memset(ilen, 0, sizeof(ilen));
    poly = 0;
  }
  bool Empty() const {
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
      for (uint8_t s = 0; s < NUM_SECTIONS; ++s) if (steps[i][s]) return false;
    return true;
  }

  // Total steps in the measure (sum of the section lengths), at least 1.
  uint8_t total_len() const {
    uint16_t t = 0;
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) t += seclen[s];
    if (t == 0) return 1;
    return (uint8_t)(t > MAX_STEPS ? MAX_STEPS : t);
  }

  // Map a playback position (0..total_len-1) to an absolute step index. The
  // measure is the sections concatenated, so position 17 of a 16+4 pattern is
  // section 1 step 1 = absolute step 17 — see OM Fig. 8.
  uint8_t pos_to_step(uint8_t pos) const {
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
      const uint8_t l = seclen[s];
      if (pos < l) return (uint8_t)(s * NUM_STEP_BTNS + pos);
      pos = (uint8_t)(pos - l);
    }
    return 0;
  }
  // Playback position of the first step of a section (for the chase display).
  uint8_t section_start(uint8_t sec) const {
    uint8_t p = 0;
    for (uint8_t s = 0; s < sec && s < NUM_SECTIONS; ++s) p = (uint8_t)(p + seclen[s]);
    return p;
  }

  bool step_get(uint8_t inst, uint8_t s) const { return (steps[inst][s >> 4] >> (s & 15)) & 1; }
  void step_set(uint8_t inst, uint8_t s) { steps[inst][s >> 4] |=  (uint16_t)1 << (s & 15); }
  void step_clr(uint8_t inst, uint8_t s) { steps[inst][s >> 4] &= ~((uint16_t)1 << (s & 15)); }
  void step_tog(uint8_t inst, uint8_t s) { steps[inst][s >> 4] ^=  (uint16_t)1 << (s & 15); }

  uint8_t ratchet_get(uint8_t inst, uint8_t s) const {
    for (uint8_t k = 0; k < RATCHET_SLOTS; ++k)
      if (rstep[k] == s && (rval[k] >> 4) == inst) return (uint8_t)(rval[k] & 15);
    return 0;
  }
  bool ratchet_set(uint8_t inst, uint8_t s, uint8_t v) {
    if (v > 3) v = 3;
    uint8_t free_k = 0xFF;
    for (uint8_t k = 0; k < RATCHET_SLOTS; ++k) {
      if (rstep[k] == s && (rval[k] >> 4) == inst) {
        if (v) rval[k] = (uint8_t)((inst << 4) | v); else rstep[k] = 0xFF;
        return true;
      }
      if (rstep[k] == 0xFF && free_k == 0xFF) free_k = k;
    }
    if (!v) return true;
    if (free_k == 0xFF) return false;
    rstep[free_k] = s;
    rval[free_k]  = (uint8_t)((inst << 4) | v);
    return true;
  }
  void ratchet_clear_inst(uint8_t inst) {
    for (uint8_t k = 0; k < RATCHET_SLOTS; ++k)
      if (rstep[k] != 0xFF && (rval[k] >> 4) == inst) rstep[k] = 0xFF;
  }

  uint8_t prob_get(uint8_t inst, uint8_t s) const {
    for (uint8_t k = 0; k < PROB_SLOTS; ++k)
      if (pstep[k] == s && (pval[k] >> 4) == inst) return (uint8_t)(pval[k] & 15);
    return 0;
  }
  bool prob_set(uint8_t inst, uint8_t s, uint8_t v) {
    uint8_t free_k = 0xFF;
    for (uint8_t k = 0; k < PROB_SLOTS; ++k) {
      if (pstep[k] == s && (pval[k] >> 4) == inst) {
        if (v) pval[k] = (uint8_t)((inst << 4) | v); else pstep[k] = 0xFF;
        return true;
      }
      if (pstep[k] == 0xFF && free_k == 0xFF) free_k = k;
    }
    if (!v) return true;
    if (free_k == 0xFF) return false;
    pstep[free_k] = s;
    pval[free_k]  = (uint8_t)((inst << 4) | v);
    return true;
  }
  void prob_clear_inst(uint8_t inst) {
    for (uint8_t k = 0; k < PROB_SLOTS; ++k)
      if (pstep[k] != 0xFF && (pval[k] >> 4) == inst) pstep[k] = 0xFF;
  }
};

// steps (12*4*2 = 96) + seclen (4) + prescale + swing (2) + rstep/rval (32)
// + pstep/pval (80) + ilen (12) + poly (2) = 228, inside the block store's
// 243-byte record cap.
//
// Changing this cleanly invalidates older saves: the store's read() returns the
// stored length and load only deserializes when it equals PATTERN_BYTES, so
// mismatched old records simply don't load — no version byte needed.
static constexpr uint8_t PATTERN_BYTES =
    NUM_INSTRUMENTS * NUM_SECTIONS * 2 + NUM_SECTIONS + 2 + RATCHET_SLOTS * 2 +
    PROB_SLOTS * 2 + NUM_INSTRUMENTS + 2;  // 228

inline void serialize_pattern(const Pattern &p, uint8_t *buf) {
  uint8_t o = 0;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
      buf[o++] = (uint8_t)(p.steps[i][s] & 0xFF);
      buf[o++] = (uint8_t)(p.steps[i][s] >> 8);
    }
  for (uint8_t s = 0; s < NUM_SECTIONS; ++s) buf[o++] = p.seclen[s];
  buf[o++] = p.prescale;
  buf[o++] = p.swing;
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) buf[o++] = p.rstep[i];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) buf[o++] = p.rval[i];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) buf[o++] = p.pstep[i];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) buf[o++] = p.pval[i];
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) buf[o++] = p.ilen[i];
  buf[o++] = (uint8_t)(p.poly & 0xFF);
  buf[o++] = (uint8_t)(p.poly >> 8);
}

inline void deserialize_pattern(Pattern &p, const uint8_t *buf) {
  uint8_t o = 0;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
      p.steps[i][s] = (uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8);
      o += 2;
    }
  for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
    p.seclen[s] = buf[o++];
    if (p.seclen[s] > NUM_STEP_BTNS) p.seclen[s] = NUM_STEP_BTNS;
  }
  p.prescale = buf[o++];
  p.swing    = buf[o++];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) p.rstep[i] = buf[o++];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) p.rval[i]  = buf[o++];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) {
    // Sanitize: a used slot needs a valid step, a real instrument and a non-zero
    // count; anything else (e.g. an editor-zeroed blob) frees it.
    if (p.rstep[i] == 0xFF) continue;
    if (p.rstep[i] >= MAX_STEPS || (p.rval[i] >> 4) >= NUM_INSTRUMENTS || !(p.rval[i] & 15))
      p.rstep[i] = 0xFF;
  }
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) p.pstep[i] = buf[o++];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) p.pval[i]  = buf[o++];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) {
    if (p.pstep[i] == 0xFF) continue;
    if (p.pstep[i] >= MAX_STEPS || (p.pval[i] >> 4) >= NUM_INSTRUMENTS || !(p.pval[i] & 15))
      p.pstep[i] = 0xFF;
  }
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
    p.ilen[i] = buf[o++];
    if (p.ilen[i] > MAX_STEPS) p.ilen[i] = 0;
  }
  p.poly = (uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8);
  o += 2;
  if (p.prescale > 3) p.prescale = 2;
  if (p.swing > 15) p.swing = 0;
  if (p.total_len() == 1 && p.seclen[0] == 0) p.seclen[0] = NUM_STEP_BTNS;
}

// A rhythm track: the pattern SLOT played in each successive measure.
//
// Deliberately slots (0..15), not absolute pattern indices: "the Variation
// function is a PLAY function only ... when Composing Rhythm Tracks the TR-808
// will not memorize the setting of either Variation switch" (OM p.19). The
// variation is applied live at playback from the panel switches, so a track
// recorded in A can be replayed wholesale in B.
struct Track {
  uint8_t len;                   // measures recorded (0 = empty track)
  uint8_t pat[TRACK_MAX_PATS];   // slot 0..15 per measure

  void Clear() { len = 0; memset(pat, 0, sizeof(pat)); }
};

static constexpr uint8_t TRACK_BYTES = 1 + TRACK_MAX_PATS;  // 65

inline void serialize_track(const Track &t, uint8_t *buf) {
  buf[0] = t.len;
  memcpy(buf + 1, t.pat, TRACK_MAX_PATS);
}

inline void deserialize_track(Track &t, const uint8_t *buf) {
  t.len = buf[0] > TRACK_MAX_PATS ? TRACK_MAX_PATS : buf[0];
  memcpy(t.pat, buf + 1, TRACK_MAX_PATS);
  for (uint8_t i = 0; i < TRACK_MAX_PATS; ++i)
    if (t.pat[i] >= NUM_SLOTS) t.pat[i] = 0;
}
