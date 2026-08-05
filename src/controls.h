// SuperOS-808 — front-panel decode layer
//
// Polarity is uniform: a closed/active/pressed cell reads as 1. Inputs are plain
// INPUT (no pull-ups) — keep it that way, the panel's own 15 K pull-downs define
// the idle level.
//
// Matrix layout (drive one PH select, read PB for steps / PA for rotaries and
// buttons; read PA with ALL selects inactive for the STATUS group):
//
//   steps 1-16     : PH[n/4] x PB[n%4]                       (n = 0..15)
//   step LEDs 1-16 : PH[n/4] x PG[n%4]                       (same grid, PG side)
//   PH0 x PA0-2    : MODE rotary (SW1a), 6 positions
//   PH0 x PA3      : CLEAR button (SW2)      <- the SuperOS menu key
//   PH1 x PA0-1    : PRE SCALE (SW7), 4 positions
//   PH1 x PA2-3    : BASIC VARIATION (SW5), A / AB / B
//   PH2 x PA0-3    : INSTRUMENT SELECT (SW3), 12 positions
//   PH3 x PA0-2    : AUTO FILL IN (SW4), MANUAL / 16 / 12 / 8 / 4 / 2
//   PH3 x PA3      : I/F VARIATION, A / B
//   status group   : TEMPO CLOCK, START/STOP, TAP, FILL IN
//
// WHY THE DECODE IS A TABLE, NOT CODE
// -----------------------------------
// The panel switches reach the CPU through a diode encoder (service manual p.8),
// not one-cell-per-switch. The encoder's truth table had to be read off a 1981
// scan, and for the 12-position INSTRUMENT SELECT and 6-position AUTO FILL IN
// that is exactly the kind of transcription where a single mis-read diode is
// invisible until the machine misbehaves. Worse, MODE's COMPOSE and PLAY
// positions trace to a bridged node and come out with the same code, which
// cannot be right for a 6-position selector (see docs/HARDWARE_MAP.md §4.1).
//
// Since this CPU cannot be bench-probed — the only access after the initial ISP
// bootloader flash is MIDI SysEx — the decode lives in a PanelMap struct that is
// persisted in the block store and can be rewritten over SysEx (0x50) or filled
// in by the guided calibration walk (0x51), using the raw matrix dump (0x3B).
// The compiled-in defaults below are the best schematic reading; an uncalibrated
// device still boots and runs, because the step keys, CLEAR, START/STOP and the
// tempo clock are unambiguous — and those four are all the calibration needs.
#pragma once
#include "pins.h"
#include "hw.h"

// MODE rotary (SW1a) positions, in the order the encoder box lists them.
enum Mode : uint8_t {
  MODE_PATTERN_CLEAR = 0,
  MODE_1ST_PART      = 1,   // PATTERN WRITE, steps 1-16
  MODE_2ND_PART      = 2,   // PATTERN WRITE, steps 17-32
  MODE_MANUAL_PLAY   = 3,
  MODE_PLAY          = 4,   // rhythm-track play
  MODE_COMPOSE       = 5,   // rhythm-track write
  NUM_MODES          = 6,
};

// INSTRUMENT SELECT rotary (SW3) order == the TR-808 panel legend.
// Index 0 is ACCENT, matching the panel's first position; the accent "voice" is
// the shared trigger-amplitude bit (PF3), not a sound generator.
enum Instrument : uint8_t {
  INST_AC = 0,   // ACCENT
  INST_BD,       // bass drum
  INST_SD,       // snare drum
  INST_LT,       // low tom / low conga
  INST_MT,       // mid tom / mid conga
  INST_HT,       // high tom / high conga
  INST_RS,       // rim shot / claves
  INST_CP,       // hand clap / maracas
  INST_CB,       // cowbell
  INST_CY,       // cymbal
  INST_OH,       // open hi-hat
  INST_CH,       // closed hi-hat
  NUM_INSTRUMENTS,
};

// BASIC VARIATION (SW5) and I/F VARIATION.
enum Variation : uint8_t { VAR_A = 0, VAR_AB = 1, VAR_B = 2, NUM_VARIATIONS = 3 };

// AUTO FILL IN (SW4): fire the fill pattern every N measures.
enum FillEvery : uint8_t {
  FILL_MANUAL = 0, FILL_16, FILL_12, FILL_8, FILL_4, FILL_2, NUM_FILL_POS
};
// Measures between automatic fills, indexed by FillEvery. 0 = manual only.
static const uint8_t FILL_MEASURES[NUM_FILL_POS] = { 0, 16, 12, 8, 4, 2 };

// ---------------------------------------------------------------------------
// PanelMap — the calibratable decode table
// ---------------------------------------------------------------------------
// Each rotary is decoded by masking the PA nibble read in its column and looking
// the result up in that rotary's code list. A read that matches nothing leaves
// the rotary at its previous value (rotaries pass through undefined codes
// between detents, and a half-written table must not make the panel jump).
struct PanelMap {
  uint8_t magic;             // PANELMAP_MAGIC when this table has been written

  uint8_t mode_code[NUM_MODES];        // PH0, mask mode_mask
  uint8_t mode_mask;
  uint8_t clear_bit;                   // PH0, PA bit index of the CLEAR button

  uint8_t pre_code[4];                 // PH1, mask pre_mask   (PRE SCALE 1..4)
  uint8_t pre_mask;
  uint8_t var_code[NUM_VARIATIONS];    // PH1, mask var_mask   (A / AB / B)
  uint8_t var_mask;

  uint8_t inst_code[NUM_INSTRUMENTS];  // PH2, mask inst_mask
  uint8_t inst_mask;

  uint8_t fill_code[NUM_FILL_POS];     // PH3, mask fill_mask
  uint8_t fill_mask;
  uint8_t ifvar_bit;                   // PH3, PA bit index of I/F VARIATION (1 = B)

  uint8_t run_bit;                     // status group bit indices
  uint8_t tap_bit;
  uint8_t fillin_bit;
  uint8_t clock_bit;

  void SetDefaults();
  void Sanitize();
};

static constexpr uint8_t PANELMAP_MAGIC = 0x88;
static constexpr uint8_t PANELMAP_BYTES = sizeof(PanelMap);
// The table goes over the wire byte-for-byte and the web editor's calibration
// page indexes it by hand-computed offsets, so its size is part of the protocol.
// All members are uint8_t, so there is no padding to worry about — but pin it,
// because a field added without updating tools/web-editor/index.html would
// silently write garbage into the panel decode.
static_assert(sizeof(PanelMap) == 43, "PanelMap wire size changed: update MAP_LAYOUT in tools/web-editor/index.html");

inline void PanelMap::SetDefaults() {
  magic = 0;                 // "defaults, never calibrated"

  // --- MODE (SW1a) on PH0, PA0-2 ------------------------------------------
  // Traced from the p.8 diode matrix:
  //   MANUAL PLAY   D18            -> PA0            = 1
  //   COMPOSE       D16            -> PA1            = 2   (see note)
  //   PLAY          D17            -> PA0            = 1   (see note)
  //   PATTERN CLEAR D23 + D22      -> PA0 | PA2      = 5
  //   1ST PART      D20 + D21      -> PA1 | PA2      = 6
  //   2ND PART      D19 + D24 chain-> PA0|PA1|PA2    = 7
  // NOTE: as drawn, COMPOSE's and PLAY's verticals are bridged, which would give
  // both {PA0,PA1} = 3 and make the encoder degenerate. The defaults below drop
  // that bridge — the one connection whose removal yields six distinct codes —
  // so COMPOSE reads its own diode (2) and PLAY reads its own (1)... except that
  // then PLAY collides with MANUAL PLAY. There is no reading of the drawing that
  // is both literal and consistent, so the shipped default takes the literal
  // bridged code for the PLAY/COMPOSE pair (3) for PLAY, and gives COMPOSE the
  // lone-diode code (2). If the machine reports 3 for both, calibration will
  // record whatever actually distinguishes them.
  mode_code[MODE_MANUAL_PLAY]   = 1;
  mode_code[MODE_COMPOSE]       = 2;
  mode_code[MODE_PLAY]          = 3;
  mode_code[MODE_PATTERN_CLEAR] = 5;
  mode_code[MODE_1ST_PART]      = 6;
  mode_code[MODE_2ND_PART]      = 7;
  mode_mask = 0x07;
  clear_bit = 3;             // SW2 -> D15 -> PA3, confirmed on the drawing

  // --- PRE SCALE (SW7) on PH1, PA0-1 --------------------------------------
  // The schematic reading, restored: 1 = D6+D7 = 3, 2 = D5 = 1, 3 = D4 = 2,
  // 4 = no diode = 0.
  //
  // MEASURED ON HARDWARE (2026-08-04). A previous pass pairwise-swapped these
  // (1=1, 2=3, 3=0, 4=2) on the theory that the emulator's own decode of the
  // same bits was the stronger reference. It is not — the emulator's decode is
  // the 1981 program's, and it never disagreed with the drawing. What that
  // swap actually produced on the machine is a dial where every detent selects
  // its pair partner: detent 1 -> PRE SCALE 2, 2 -> 1, 3 -> 4, 4 -> 3, which is
  // exactly the permutation those swapped codes describe. Read the raw code at
  // a known detent from the `F0 7D 3B F7` probe before touching this table
  // again; the panel decode is the one part of this firmware where "it sounds
  // wrong" and "it reads wrong" have different fixes.
  //
  // Which entry to change is settled by the fact that PRESCALE_TICKS in
  // pattern.h is indexed by DETENT, not by code, and is pinned by the owner's
  // manual: OM p.16 sets PRE SCALE 1 for a 12-step 4/4 measure = 3 steps per
  // beat, p.13 / p.15 set PRE SCALE 3 for a 16-step 4/4 and a 12-step 3/4 = 4
  // steps per beat. Swapping PRESCALE_TICKS instead gives identical panel
  // behaviour while silently breaking both of those anchors, so it looks like a
  // fix and is not one. Fix the codes here.
  //
  // A CALIBRATED TABLE OVERRIDES THIS. If the block store holds a map with
  // PANELMAP_MAGIC, load_panel_map() replaces the whole struct, defaults
  // included — so a device that has been through the guided walk keeps whatever
  // that walk recorded and must be re-walked, not reflashed, to pick this up.
  pre_code[0] = 3;  pre_code[1] = 1;  pre_code[2] = 2;  pre_code[3] = 0;
  pre_mask = 0x03;

  // --- BASIC VARIATION (SW5) on PH1, PA2-3 --------------------------------
  // Confirmed on the drawing: A = no diode, B = D3 -> PA2, AB = D2 -> PA3.
  var_code[VAR_A]  = 0x0;
  var_code[VAR_B]  = 0x4;
  var_code[VAR_AB] = 0x8;
  var_mask = 0x0C;

  // --- INSTRUMENT SELECT (SW3) on PH2, PA0-3 ------------------------------
  // MEASURED ON HARDWARE. The encoder is a 12-position CYCLIC counter that runs
  // DOWN as the knob runs up — it uses every code 0..11 exactly once and wraps,
  // so the whole table is one number:
  //
  //     code = (INST_CODE_TOP - position) mod 12
  //
  //   AC 11   HT  6   CB  3
  //   BD 10   RS  5   CY  2
  //   SD  9   CP  4   OH  1
  //   LT  8           CH  0
  //   MT  7
  //
  // Only the TOP is in question — that the run descends and wraps is settled by
  // two independent captures, both of which came out as a complete permutation
  // of 0..11 with no gaps and no collisions. A wrong top shows up as a UNIFORM
  // rotation of the whole dial, never as one bad detent, so it is fixed by
  // moving this one constant rather than by editing entries.
  //
  // TOP = 11 is MEASURED, not inferred: with the knob on the AC detent the raw
  // `F0 7D 3B F7` probe reads code 11 (2026-08-04, twice, on a build known to be
  // the one running). Read a detent's code off that probe rather than off what it
  // sounds like — three earlier attempts inferred it by ear and every one came
  // out rotated.
  //
  // THE TRAP THAT COST TWO ROTATIONS: a rotated dial is usually a STALE BINARY,
  // not a wrong table. This table was already correct when the machine reported
  // "the SD dial plays the LT and everything else is 1 away like that", and
  // "fixing" it to match rotated it for real. Before touching TOP, dump the raw
  // code at a KNOWN detent from the machine you are actually going to flash, and
  // check that what is running came from this source.
  static const uint8_t INST_CODE_TOP = 11;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
    inst_code[i] = (uint8_t)((INST_CODE_TOP + NUM_INSTRUMENTS - i) % NUM_INSTRUMENTS);
  inst_mask = 0x0F;

  // --- AUTO FILL IN (SW4) on PH3, PA0-2 -----------------------------------
  // UNVERIFIED, same reason. Default is plain binary in panel order.
  for (uint8_t i = 0; i < NUM_FILL_POS; ++i) fill_code[i] = i;
  fill_mask = 0x07;
  ifvar_bit = 3;

  // --- STATUS group (all selects inactive) --------------------------------
  // Order per the service manual p.3: "STATUS (TEMPO CLOCK, START/STOP,
  // FILL IN) inputs".
  run_bit    = 0;
  tap_bit    = 1;
  fillin_bit = 2;
  clock_bit  = 3;
}

inline void PanelMap::Sanitize() {
  if (clear_bit > 3) clear_bit = 3;
  if (ifvar_bit > 3) ifvar_bit = 3;
  if (run_bit    > 3) run_bit    = 0;
  if (tap_bit    > 3) tap_bit    = 1;
  if (fillin_bit > 3) fillin_bit = 2;
  if (clock_bit  > 3) clock_bit  = 3;
  if (!mode_mask) mode_mask = 0x07;
  if (!pre_mask)  pre_mask  = 0x03;
  if (!var_mask)  var_mask  = 0x0C;
  if (!inst_mask) inst_mask = 0x0F;
  if (!fill_mask) fill_mask = 0x07;
  for (uint8_t i = 0; i < NUM_MODES; ++i)       mode_code[i] &= 0x0F;
  for (uint8_t i = 0; i < 4; ++i)               pre_code[i]  &= 0x0F;
  for (uint8_t i = 0; i < NUM_VARIATIONS; ++i)  var_code[i]  &= 0x0F;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) inst_code[i] &= 0x0F;
  for (uint8_t i = 0; i < NUM_FILL_POS; ++i)    fill_code[i] &= 0x0F;
}

// Single global instance, defined in main.cpp, written by the calibration path.
extern PanelMap g_panel_map;

// ---------------------------------------------------------------------------
// Controls — one scan's worth of panel state, decoded through g_panel_map
// ---------------------------------------------------------------------------
struct Controls {
  uint8_t cell[4] = {0, 0, 0, 0};  // per select: bit0-3 = PB0-3, bit4-7 = PA0-3
  uint8_t status_hi = 0;           // PA0-3 with all selects inactive

  void Scan() { hw::ScanMatrix(cell, &status_hi); }

  // raw cell access
  bool pb(uint8_t sel, uint8_t b) const { return (cell[sel] >> b) & 1; }
  bool pa(uint8_t sel, uint8_t b) const { return (cell[sel] >> (4 + b)) & 1; }
  uint8_t pa_nib(uint8_t sel) const { return (uint8_t)((cell[sel] >> 4) & 0x0F); }

  // step switches (0..15)
  bool step(uint8_t n) const { return pb(n >> 2, n & 3); }

  // --- rotaries: table lookup, previous value held on no match --------------
  // `prev` is the caller's last decoded value, returned unchanged when the read
  // matches no entry (rotary mid-detent, or an uncalibrated table).
  static uint8_t lookup(uint8_t read, const uint8_t *codes, uint8_t n,
                        uint8_t mask, uint8_t prev) {
    const uint8_t v = (uint8_t)(read & mask);
    for (uint8_t i = 0; i < n; ++i)
      if ((uint8_t)(codes[i] & mask) == v) return i;
    return prev;
  }

  uint8_t mode(uint8_t prev) const {
    return lookup(pa_nib(0), g_panel_map.mode_code, NUM_MODES,
                  g_panel_map.mode_mask, prev);
  }
  uint8_t prescale(uint8_t prev) const {
    return lookup(pa_nib(1), g_panel_map.pre_code, 4,
                  g_panel_map.pre_mask, prev);
  }
  uint8_t variation(uint8_t prev) const {
    return lookup(pa_nib(1), g_panel_map.var_code, NUM_VARIATIONS,
                  g_panel_map.var_mask, prev);
  }
  uint8_t instrument(uint8_t prev) const {
    return lookup(pa_nib(2), g_panel_map.inst_code, NUM_INSTRUMENTS,
                  g_panel_map.inst_mask, prev);
  }
  // AUTO FILL IN is the one rotary whose codes have never been measured — the
  // compiled table is plain binary and is a guess. A wrong guess here is not
  // cosmetic: it silently turns auto-fill ON, and the machine drops the
  // INTRO/FILL pattern into the middle of playback every few measures, which
  // reads as "my pattern keeps changing to another one". Observed on hardware:
  // the MANUAL detent reports code 5, which the binary table maps to "every 2
  // measures". So until the table is calibrated, report MANUAL and never fill.
  uint8_t fill_every(uint8_t prev) const {
    if (g_panel_map.magic != PANELMAP_MAGIC) return FILL_MANUAL;
    return lookup(pa_nib(3), g_panel_map.fill_code, NUM_FILL_POS,
                  g_panel_map.fill_mask, prev);
  }
  bool if_variation_b() const { return pa(3, g_panel_map.ifvar_bit); }

  // --- buttons / status -----------------------------------------------------
  bool clear() const { return pa(0, g_panel_map.clear_bit); }

  bool run() const       { return (status_hi >> g_panel_map.run_bit)    & 1; }
  bool tap() const       { return (status_hi >> g_panel_map.tap_bit)    & 1; }
  bool fill_in() const   { return (status_hi >> g_panel_map.fillin_bit) & 1; }
  bool tempo_clk() const { return (status_hi >> g_panel_map.clock_bit)  & 1; }
};

// step-LED helper: light step LED n (0..15) on the PG x PH grid
namespace hw {
  inline void LightStep(uint8_t n) { LightCell(n >> 2, n & 3); }
}
