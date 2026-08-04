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
  if (g_flash.read(FB_PATTERN_BASE + idx, buf, sizeof(buf)) != PATTERN_BYTES) return false;
  deserialize_pattern(out, buf);
  return true;
}
inline bool flash_pat_write(uint8_t idx, const Pattern &in) {
  if (!g_flash_ok) return false;
  uint8_t buf[FE_MAX_PAYLOAD];
  serialize_pattern(in, buf);
  return g_flash.write(FB_PATTERN_BASE + idx, buf, PATTERN_BYTES);
}

// Register the pattern hooks and load tracks. Patterns are loaded lazily by the
// engine cache on first access. Power-on selection is deliberately NOT restored:
// the device always wakes on pattern 1, variation A, like the stock 808.
inline void load_all(Engine &eng) {
  eng.SetPatIO(flash_pat_read, flash_pat_write);   // hooks self-guard on g_flash_ok
  if (!g_flash_ok) return;
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_TRACKS; ++i)
    if (g_flash.read(FB_TRACK_BASE + i, buf, TRACK_BYTES) == TRACK_BYTES)
      deserialize_track(eng.track[i], buf);
}

inline void load_settings(Settings &s) {
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
inline void load_panel_map(PanelMap &m) {
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
inline void save_dirty(Engine &eng) {
  if (!g_flash_ok) return;
  eng.Flush();
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_TRACKS; ++i) {
    if (!(eng.dirty_trk & ((uint16_t)1 << i))) continue;
    serialize_track(eng.track[i], buf);
    if (g_flash.write(FB_TRACK_BASE + i, buf, TRACK_BYTES))
      eng.dirty_trk &= (uint16_t)~((uint16_t)1 << i);
  }
}
