// SuperOS-808 — device glue between the sequencer and the block store.
// Provides the page program/read hooks, the single store instance, the logical
// block-id map, and pattern / track / settings / panel-map load+save helpers.
//
// Storage is the wear-leveled internal-flash arena (flash_eeprom.h). Its writes
// go through the boot-section SPM service (flash_store.h), installed once per
// board via service-install.syx. If the service is absent everything still runs
// — patterns just do not survive power-off.
#pragma once
#include <Arduino.h>
#include "flash_store.h"
#include "flash_eeprom.h"
#include "pattern.h"
#include "engine.h"
#include "settings.h"
#include "controls.h"

// Logical block ids (must stay < FE_MAX_BLOCKS = 48).
static constexpr uint8_t FB_PATTERN_BASE = 0;                          //  0..31
static constexpr uint8_t FB_TRACK_BASE   = NUM_PATTERNS;               // 32..43
static constexpr uint8_t FB_SETTINGS     = FB_TRACK_BASE + NUM_TRACKS; // 44
static constexpr uint8_t FB_PANELMAP     = FB_SETTINGS + 1;            // 45
static_assert(FB_PANELMAP < FE_MAX_BLOCKS, "block-id space overflowed");

// FB_SETTINGS payload: magic, midi_channel, clock_source, out_mode. The magic
// gates a blank/older block so a fresh device falls back to the struct defaults
// instead of garbage.
static constexpr uint8_t FB_SETTINGS_LEN = 4;
static constexpr uint8_t SETTINGS_MAGIC  = 0x96;

// Page hooks: program via the boot SPM service, read via far program-memory reads.
inline uint8_t fe_dev_program(uint16_t abs_page, const uint8_t *buf) {
  return flash_write_page(abs_page, buf);
}
inline void fe_dev_read(uint16_t abs_page, uint8_t *buf) {
  uint32_t a = (uint32_t)abs_page << 8;
  for (uint16_t i = 0; i < FE_PAGE; ++i) buf[i] = pgm_read_byte_far(a + i);
}

typedef FlashEeprom PersistStore;
PersistStore g_flash;
static bool g_flash_ok = false;

// Mount the arena (formats if blank). False if the SPM service is missing — the
// app then runs RAM-only.
inline bool flash_persist_begin() {
  if (!flash_service_present()) return false;
  g_flash_ok = g_flash.begin(fe_dev_program, fe_dev_read);
  return g_flash_ok;
}

// Store-resident pattern IO hooks (registered with the Engine). Read returns
// false on a blank/older block so the cache falls back to an empty pattern.
inline bool flash_pat_read(uint8_t idx, Pattern &out) {
  if (!g_flash_ok) return false;
  uint8_t buf[FE_MAX_PAYLOAD];
  // Cap at the full buffer, not PATTERN_BYTES: read() clamps to the cap, so a
  // longer record from an OLD format would otherwise clamp to exactly
  // PATTERN_BYTES and load truncated garbage instead of being rejected.
  // Accept the PRE-claim-123 record too and carry it forward. Direction was
  // appended after poly, so an old record is a byte-exact PREFIX of a new one:
  // zero-filling the tail yields DIR_FWD and "every voice follows the pattern",
  // which are exactly the defaults. No conversion pass, no version byte, and
  // nothing already on the machine stops loading -- which is what would have
  // happened if PATTERN_BYTES had simply changed, since it IS the load key.
  {
    const uint8_t n = g_flash.read(FB_PATTERN_BASE + idx, buf, sizeof(buf));
    if (n == PATTERN_BYTES_V1)
      memset(buf + PATTERN_BYTES_V1, 0, PATTERN_BYTES - PATTERN_BYTES_V1);
    else if (n != PATTERN_BYTES)
      return false;
  }
  deserialize_pattern(out, buf);
  return true;
}
// Kept out of flash_pat_write's frame (noinline) so its 243-byte buf never
// coexists with gc()'s frame: flash_pat_write runs gc_if_full() first, at a
// shallower depth where no buffer is reserved, then this appends. Together they
// keep the push chain under the combined build's stack (claim-016).
static __attribute__((noinline)) bool flash_pat_append(uint8_t idx, const Pattern &in) {
  uint8_t buf[FE_MAX_PAYLOAD];
  serialize_pattern(in, buf);
  return g_flash.append_only(FB_PATTERN_BASE + idx, buf, PATTERN_BYTES);
}

inline bool flash_pat_write(uint8_t idx, const Pattern &in) {
  if (!g_flash_ok) return false;
  if (!g_flash.gc_if_full()) return false;   // compact here, before buf exists
  return flash_pat_append(idx, in);
}

// FIELD-level store access: read or patch a few bytes of a stored pattern, and
// clear one, without the caller ever holding a record.
//
// This is what claim-080 came down to. Measured with tools/diag/stack_walk.py
// once it learned to follow `ijmp`: flash_pat_write's own chain is 333 B, and
// every operation above it shares one 663-byte stack, so a caller holding a
// 228-byte Pattern to edit two bytes of it overflows wherever it sits. The three
// callers that did (the A<->B mirror, master poly, PATTERN CLEAR) now hold at
// most five bytes, and the ONE record buffer lives here.
//
// The buffer is shared between the read and the write-back within a single
// frame, which is the other half of it: the read-modify-write through the Engine
// put TWO record-sized buffers on one chain.
// ONE worker for all three, because all three want the same 243-byte record
// buffer and the same read prologue. Three copies of it cost ~450 B of flash,
// and combined-nat-test has only ~900 B between the app and the flash service's
// staging window (makesyx refuses the image past 0xD700), so on this machine
// that is not a tidiness argument.
enum : uint8_t { PAT_OP_PEEK = 0, PAT_OP_PATCH = 1, PAT_OP_CLEAR = 2 };

static __attribute__((noinline)) bool flash_pat_rec_op_(
    uint8_t idx, uint8_t op, uint8_t off, uint8_t *data, uint8_t len) {
  uint8_t buf[FE_MAX_PAYLOAD];
  if (op == PAT_OP_CLEAR) {
    serialize_cleared_pattern(buf);
    return g_flash.append_only(FB_PATTERN_BASE + idx, buf, PATTERN_BYTES);
  }
  {
    const uint8_t n = g_flash.read(FB_PATTERN_BASE + idx, buf, sizeof(buf));
    if (n == PATTERN_BYTES_V1)
      memset(buf + PATTERN_BYTES_V1, 0, PATTERN_BYTES - PATTERN_BYTES_V1);
    else if (n != PATTERN_BYTES)
      return false;
  }
  if (op == PAT_OP_PEEK) {
    memcpy(data, buf + off, len);      // raw bytes; the caller owns any clamp
    return true;
  }
  bool changed = false;
  for (uint8_t i = 0; i < len; ++i)
    if (buf[off + i] != data[i]) { buf[off + i] = data[i]; changed = true; }
  if (!changed) return true;           // identical: no write, so no flash wear
  return g_flash.append_only(FB_PATTERN_BASE + idx, buf, PATTERN_BYTES);
}

// gc_if_full runs HERE, at a shallower depth than the buffer, which is the same
// discipline flash_pat_write follows and the whole of claim-072.
static bool __attribute__((noinline))
flash_pat_op_(uint8_t idx, uint8_t op, uint8_t off, uint8_t *data, uint8_t len) {
  if (!g_flash_ok) return false;
  if ((uint16_t)off + len > PATTERN_BYTES) return false;
  if (op != PAT_OP_PEEK && !g_flash.gc_if_full()) return false;
  return flash_pat_rec_op_(idx, op, off, data, len);
}

inline bool flash_pat_peek(uint8_t idx, uint8_t off, uint8_t *dst, uint8_t len) {
  return flash_pat_op_(idx, PAT_OP_PEEK, off, dst, len);
}
inline bool flash_pat_patch(uint8_t idx, uint8_t off, const uint8_t *src, uint8_t len) {
  return flash_pat_op_(idx, PAT_OP_PATCH, off, const_cast<uint8_t *>(src), len);
}
inline bool flash_pat_clear(uint8_t idx) {
  return flash_pat_op_(idx, PAT_OP_CLEAR, 0, nullptr, 0);
}

// Register the pattern hooks and load tracks. Patterns are loaded lazily by the
// engine cache on first access. Power-on selection is deliberately NOT restored:
// the device always wakes on pattern 1, variation A, like the stock 808.
inline void load_all(Engine &eng) {
  // hooks self-guard on g_flash_ok
  eng.SetPatIO(flash_pat_read, flash_pat_write,
               flash_pat_peek, flash_pat_patch, flash_pat_clear);
  if (!g_flash_ok) return;
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_TRACKS; ++i)
    if (g_flash.read(FB_TRACK_BASE + i, buf, TRACK_BYTES) == TRACK_BYTES)
      deserialize_track(eng.track[i], buf);
}

inline void __attribute__((noinline)) load_settings(Settings &s) {
  if (!g_flash_ok) return;
  uint8_t buf[FB_SETTINGS_LEN];
  if (g_flash.read(FB_SETTINGS, buf, FB_SETTINGS_LEN) == FB_SETTINGS_LEN &&
      buf[0] == SETTINGS_MAGIC) {
    s.midi_channel = buf[1];
    s.clock_source = buf[2];
    s.out_mode     = buf[3];
    s.sanitize();
  }
}

inline void save_settings(const Settings &s) {
  if (!g_flash_ok) return;
  const uint8_t buf[FB_SETTINGS_LEN] = {
    SETTINGS_MAGIC, s.midi_channel, s.clock_source, s.out_mode };
  g_flash.write(FB_SETTINGS, buf, FB_SETTINGS_LEN);
}

// --- panel decode table -----------------------------------------------------
// The rotary decode is calibrated on the machine rather than compiled in (see
// docs/HARDWARE_MAP.md §7). A blank or magic-mismatched block leaves the
// compiled defaults in place, so an uncalibrated device still boots and can be
// driven through the calibration walk.
inline void __attribute__((noinline)) load_panel_map(PanelMap &m) {
  m.SetDefaults();
  if (!g_flash_ok) return;
  uint8_t buf[PANELMAP_BYTES];
  if (g_flash.read(FB_PANELMAP, buf, PANELMAP_BYTES) == PANELMAP_BYTES &&
      buf[0] == PANELMAP_MAGIC) {
    memcpy(&m, buf, PANELMAP_BYTES);
    m.Sanitize();
  }
}

inline void save_panel_map(const PanelMap &m) {
  if (!g_flash_ok) return;
  PanelMap t = m;
  t.magic = PANELMAP_MAGIC;
  t.Sanitize();
  g_flash.write(FB_PANELMAP, (const uint8_t *)&t, PANELMAP_BYTES);
}

// Write everything whose dirty bit is set, then clear the bits. On the flash
// backend each page write halts the CPU for a few ms, so call only while the
// sequencer is stopped.
// Kept out of save_dirty's frame (noinline) for the same reason
// flash_pat_append is kept out of flash_pat_write's: its 243-byte buf must never
// coexist with gc()'s frame. save_dirty runs gc_if_full() first, at a shallower
// depth where no buffer is reserved, then this appends.
//
// claim-072: claim-016 made exactly this split for the pattern-push path and
// missed this one, because the analysis was a call chain added by hand and
// nobody put save_dirty and gc() end to end. save_dirty is called from
// superos_loop, whose own frame is 256 B, so the old
//   superos_loop -> save_dirty(buf 243) -> write -> gc
// reached 708 B against the combined build's 663 B budget.
// buf is TRACK_BYTES (65), not FE_MAX_PAYLOAD (243). The original held the full
// payload buffer for a record that serialize_track fills to exactly TRACK_BYTES
// (`buf[0] = len` then TRACK_MAX_PATS bytes), so 178 of those bytes were never
// written. Only the append path can size down like this: flash_pat_read must
// keep the full buffer, because it caps read() at sizeof(buf) to reject an
// over-long record from an older format instead of silently truncating it.
static __attribute__((noinline)) bool flash_trk_append(uint8_t i, const Track &t) {
  uint8_t buf[TRACK_BYTES];
  serialize_track(t, buf);
  return g_flash.append_only(FB_TRACK_BASE + i, buf, TRACK_BYTES);
}

inline void save_dirty(Engine &eng) {
  if (!g_flash_ok) return;
  eng.Flush();
  for (uint8_t i = 0; i < NUM_TRACKS; ++i) {
    if (!(eng.dirty_trk & ((uint16_t)1 << i))) continue;
    // Compact here, before buf exists. A failed compaction leaves the track
    // dirty rather than dropping it, so the next save retries.
    if (!g_flash.gc_if_full()) continue;
    if (flash_trk_append(i, eng.track[i]))
      eng.dirty_trk &= (uint16_t)~((uint16_t)1 << i);
  }
}
