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

// Returned by pos_to_step() for a position the section layout does not cover.
// 0xFF is not a reachable step index (steps are 0..63, MAX_STEPS = 64), so the
// error is distinguishable from every valid answer. It used to be 0, which is a
// real playable step, and no caller could tell the two apart (claim-051).
static const uint8_t POS_INVALID = 0xFF;

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
  // Play direction, saved WITH the pattern (claim-123).
  //   dir     the pattern's own direction, DIR_* (0 = forwards)
  //   idir[]  per voice, a NIBBLE each: 0 = follow `dir`, else DIR_* + 1
  // Nibbles because twelve voices at a byte apiece is twelve bytes of a record
  // capped at 243, and this record has to leave room for what comes after it.
  // Track is packed the same way and for the same reason.
  uint8_t  dir;
  uint8_t  idir[(NUM_INSTRUMENTS + 1) / 2];
  // Per-voice PRE SCALE (claim-124), a nibble each: 0 = follow the pattern's
  // own prescale, else PRE SCALE value + 1. Same packing as idir, and it takes
  // the record to 241 of a 243 cap -- there is no room after this one.
  uint8_t  ipre[(NUM_INSTRUMENTS + 1) / 2];

  // A cleared pattern is the machine's own default: 1st Part 16 steps, 2nd Part
  // 0 (OM p.18), PRE SCALE left at whatever the dial commits next.
  void Clear() {
    memset(steps, 0, sizeof(steps));
    seclen[0] = NUM_STEP_BTNS;
    seclen[1] = seclen[2] = seclen[3] = 0;
    prescale = 2;                   // panel "3" = 16ths, the common default
    dir      = 0;                   // DIR_FWD
    memset(idir, 0, sizeof(idir));  // every voice follows the pattern
    memset(ipre, 0, sizeof(ipre));  // ...and the pattern's prescale
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
  // section 1 step 1 = absolute step 17, see OM Fig. 8.
  //
  // Returns POS_INVALID for a position the layout does not cover. There is
  // exactly one state in which a position BELOW total_len() can do that, and it
  // is worth naming because it is not obvious: total_len() reports 1 when every
  // seclen is 0 (it refuses to return a zero length), so position 0 is legal
  // while no section contains it. Three separate guards keep that state off the
  // machine: Clear() sets seclen[0] = 16, the panel's SetSectionLength is fed
  // b + 1 and cannot pass 0 (main.cpp:527, :717), and deserialize_pattern
  // repairs an all-zero layout on every store and SysEx read. peek_layout_ is
  // the one guard that clamps HIGH only (engine.h:1024), and it is safe solely
  // because no route produces the low value for it to pass through.
  uint8_t pos_to_step(uint8_t pos) const {
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
      const uint8_t l = seclen[s];
      if (pos < l) return (uint8_t)(s * NUM_STEP_BTNS + pos);
      pos = (uint8_t)(pos - l);
    }
    return POS_INVALID;
  }
  // Playback position of the first step of a section (for the chase display).
  uint8_t section_start(uint8_t sec) const {
    uint8_t p = 0;
    for (uint8_t s = 0; s < sec && s < NUM_SECTIONS; ++s) p = (uint8_t)(p + seclen[s]);
    return p;
  }

  // 0xFF = follow the pattern's own direction. Stored as nibble 0.
  uint8_t inst_dir_get(uint8_t inst) const {
    if (inst >= NUM_INSTRUMENTS) return 0xFF;
    const uint8_t n = (inst & 1) ? (uint8_t)(idir[inst >> 1] >> 4)
                                 : (uint8_t)(idir[inst >> 1] & 0x0F);
    return n ? (uint8_t)(n - 1) : 0xFF;
  }
  uint8_t inst_pre_get(uint8_t inst) const {
    if (inst >= NUM_INSTRUMENTS) return 0xFF;
    const uint8_t n = (inst & 1) ? (uint8_t)(ipre[inst >> 1] >> 4)
                                 : (uint8_t)(ipre[inst >> 1] & 0x0F);
    return n ? (uint8_t)(n - 1) : 0xFF;
  }
  void inst_pre_set(uint8_t inst, uint8_t v) {
    if (inst >= NUM_INSTRUMENTS) return;
    const uint8_t n = (v == 0xFF || v > 3) ? 0 : (uint8_t)(v + 1);
    uint8_t &b = ipre[inst >> 1];
    if (inst & 1) b = (uint8_t)((b & 0x0F) | (uint8_t)(n << 4));
    else          b = (uint8_t)((b & 0xF0) | n);
  }

  void inst_dir_set(uint8_t inst, uint8_t d) {
    if (inst >= NUM_INSTRUMENTS) return;
    const uint8_t n = (d == 0xFF || d > 3) ? 0 : (uint8_t)(d + 1);
    uint8_t &b = idir[inst >> 1];
    if (inst & 1) b = (uint8_t)((b & 0x0F) | (uint8_t)(n << 4));
    else          b = (uint8_t)((b & 0xF0) | n);
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
  // v is a chance in 16ths, 1..15; 0 frees the slot, which is "always plays".
  //
  // CLAMPED, because the value shares a byte with the instrument index and an
  // out-of-range chance does not overflow harmlessly: v = 16 makes
  // (inst << 4) | v read back as instrument inst+1 with chance 0. The panel
  // tool used to hand exactly that in from step 16, so the press quietly
  // rewrote the slot onto the next instrument, did nothing audible, and burned
  // one of the PROB_SLOTS. Step 16 now means "always" at the call site, and the
  // clamp keeps any other caller (a pushed pattern, the editor) honest.
  bool prob_set(uint8_t inst, uint8_t s, uint8_t v) {
    if (v > 15) v = 15;
    if (inst >= NUM_INSTRUMENTS) return false;
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
// The record as it stood before claim-123 added direction. Kept because it is
// the LOAD KEY of every pattern written before that: flash_pat_read accepts it,
// zero-fills the new tail and carries the record forward. Zero is exactly the
// right default for both new fields (DIR_FWD, and "every voice follows the
// pattern"), which is why the upgrade needs no conversion pass and no version
// byte -- an old record is a byte-exact PREFIX of a new one.
static constexpr uint8_t PATTERN_BYTES_V1 =
    NUM_INSTRUMENTS * NUM_SECTIONS * 2 + NUM_SECTIONS + 2 + RATCHET_SLOTS * 2 +
    PROB_SLOTS * 2 + NUM_INSTRUMENTS + 2;  // 228

static constexpr uint8_t PATTERN_BYTES =
    PATTERN_BYTES_V1 + 1 + 2 * ((NUM_INSTRUMENTS + 1) / 2);  // 241, cap is 243
static_assert(PATTERN_BYTES <= 243, "record exceeds the block store's cap");

// FIELD WINDOWS inside a serialized record, for the operations that touch a few
// bytes of a pattern they otherwise do not care about: the A<->B layout mirror
// (seclen[0..3] then prescale) and the master-poly switch (poly).
//
// They exist because a record-sized buffer does not fit above a flash write on
// this machine. flash_pat_write's own chain is 333 B and everything above it
// shares one 663-byte stack, so an operation that reads a 228-byte Pattern,
// edits two bytes of it and writes it back overflows wherever it sits -- which
// is claim-080, and why relocating those buffers never worked while narrowing
// the operation does.
//
// Kept next to the serializer that defines them, and pinned to it by
// tools/hosttest/record_fields.cpp: that test serializes a pattern, reads these
// windows straight out of the bytes, and fails if they disagree with the struct.
// An offset that drifts silently would patch the wrong field of every stored
// pattern, so it is not left to a comment.
static constexpr uint8_t PATTERN_LAYOUT_OFF = NUM_INSTRUMENTS * NUM_SECTIONS * 2;
static constexpr uint8_t PATTERN_LAYOUT_LEN = NUM_SECTIONS + 1;   // seclen + prescale
// PINNED, not derived. It used to be PATTERN_BYTES - 2 because poly was last;
// direction is appended AFTER poly so old records stay a prefix, which means
// poly is no longer last and the old expression would now point into the new
// fields and patch the wrong bytes of every stored pattern.
static constexpr uint8_t PATTERN_POLY_OFF   = PATTERN_BYTES_V1 - 2;
static constexpr uint8_t PATTERN_POLY_LEN   = 2;

// The layout window is contiguous in the STRUCT as well as in the record, which
// is what lets the engine copy it with one memcpy instead of a field-by-field
// loop (400 B of flash for ServiceLayoutSync down to a third of that, and this
// build has ~900 B between the app and the flash service's staging window).
// Asserted rather than assumed: inserting a member between seclen and prescale
// would otherwise mirror `swing` onto the sibling and leave prescale behind.
static_assert(__builtin_offsetof(Pattern, prescale) ==
                  __builtin_offsetof(Pattern, seclen) + NUM_SECTIONS,
              "seclen[] and prescale must stay adjacent: they are one window");

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
  buf[o++] = p.dir;                                    // claim-123, after poly
  for (uint8_t i = 0; i < (NUM_INSTRUMENTS + 1) / 2; ++i) buf[o++] = p.idir[i];
  for (uint8_t i = 0; i < (NUM_INSTRUMENTS + 1) / 2; ++i) buf[o++] = p.ipre[i];
}

// The bytes serialize_pattern would emit for a freshly Clear()ed pattern, built
// straight into the record buffer. PATTERN CLEAR used to construct a Pattern on
// the stack to hand to the store, which put 228 B above a flash write for a
// value that is a constant (claim-080). Byte-for-byte equality with
// serialize_pattern(Pattern::Clear()) is asserted in tools/hosttest/record_fields.cpp,
// not argued for here.
inline void serialize_cleared_pattern(uint8_t *buf) {
  memset(buf, 0, PATTERN_BYTES);
  buf[PATTERN_LAYOUT_OFF] = NUM_STEP_BTNS;              // seclen[0]
  buf[PATTERN_LAYOUT_OFF + NUM_SECTIONS] = 2;           // prescale: panel "3"
  const uint8_t rstep_off = (uint8_t)(PATTERN_LAYOUT_OFF + NUM_SECTIONS + 2);
  memset(buf + rstep_off, 0xFF, RATCHET_SLOTS);         // rstep: all slots free
  const uint8_t pstep_off = (uint8_t)(rstep_off + RATCHET_SLOTS * 2);
  memset(buf + pstep_off, 0xFF, PROB_SLOTS);            // pstep: all slots free
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
  // claim-123, APPENDED AFTER poly so a pre-claim-123 record is a byte-exact
  // prefix. flash_pat_read zero-fills this tail for an old record, and zero is
  // the correct default for both: DIR_FWD, every voice following the pattern.
  p.dir = buf[o++];
  if (p.dir > 3) p.dir = 0;
  for (uint8_t i = 0; i < (NUM_INSTRUMENTS + 1) / 2; ++i) p.idir[i] = buf[o++];
  for (uint8_t i = 0; i < (NUM_INSTRUMENTS + 1) / 2; ++i) p.ipre[i] = buf[o++];
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
// A measure of a track is a STEP BUTTON, 0..15, so it is a nibble and is stored
// as one: two measures per byte. A byte each cost 12 x 64 = 768 B of an 8192 B
// SRAM to carry 384 B of data, and on this machine static RAM is subtracted
// from 8192 to derive the stack budget (claim-080), so the high nibble that
// never held anything was being paid for out of stack headroom.
//
// It also makes the invalid state unrepresentable. deserialize_track used to
// end with `if (t.pat[i] >= NUM_SLOTS) t.pat[i] = 0`, because a stored byte
// could say 200; a nibble cannot say anything but 0..15, so the loop is gone
// and the guarantee is stronger than the loop was (the loop ran on load only,
// the type holds at every write).
//
// The SERIALIZED form is unchanged: one byte per measure, TRACK_BYTES = 65.
// serialize_track is the only path to both the flash record and the 0x24/0x25
// wire format, so neither moves.
struct Track {
  static constexpr uint8_t PACKED_BYTES = (TRACK_MAX_PATS + 1) / 2;

  // `cont` is the stock program's "continues" bit (bit 3 of its length high
  // nibble): COMPOSE ran past 64 measures and rolled into the next track, so
  // playback runs on into it (OM p.19, measured in tools/d650/rtrack_probe).
  // It rides bit 7 of the serialized length byte, which a record written
  // before it existed never sets.
  uint8_t len  : 7;              // measures recorded (0 = empty track)
  uint8_t cont : 1;
  uint8_t pat_[PACKED_BYTES];    // two measures per byte, low nibble first

  uint8_t pat(uint8_t i) const {
    if (i >= TRACK_MAX_PATS) return 0;
    const uint8_t b = pat_[i >> 1];
    return (uint8_t)((i & 1) ? (b >> 4) : (b & 0x0F));
  }
  // Out of line: inlined at every track write it cost 194 B of flash.
  __attribute__((noinline)) void set_pat(uint8_t i, uint8_t v) {
    if (i >= TRACK_MAX_PATS) return;
    uint8_t &b = pat_[i >> 1];
    if (i & 1) b = (uint8_t)((b & 0x0F) | (uint8_t)((v & 0x0F) << 4));
    else       b = (uint8_t)((b & 0xF0) |  (uint8_t)(v & 0x0F));
  }

  void Clear() { len = 0; cont = 0; memset(pat_, 0, sizeof(pat_)); }
};

static constexpr uint8_t TRACK_BYTES = 1 + TRACK_MAX_PATS;  // 65

// One byte per measure on the wire and in flash, whatever the RAM layout is.
inline void serialize_track(const Track &t, uint8_t *buf) {
  buf[0] = (uint8_t)(t.len | (t.cont << 7));
  for (uint8_t i = 0; i < TRACK_MAX_PATS; ++i) buf[1 + i] = t.pat(i);
}

// set_pat masks to a nibble, so a stored byte of 200 lands as slot 8 rather
// than as an out-of-range slot. The old explicit range check is therefore not
// missing here, it is in the type.
inline void deserialize_track(Track &t, const uint8_t *buf) {
  const uint8_t n = buf[0] & 0x7F;
  t.len  = n > TRACK_MAX_PATS ? TRACK_MAX_PATS : n;
  t.cont = buf[0] >> 7;
  for (uint8_t i = 0; i < TRACK_MAX_PATS; ++i) t.set_pat(i, buf[1 + i]);
}
