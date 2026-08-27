// combined.h -- combined SuperOS-808 + µPD650C-085 emulator build
// (SUPEROS_COMBINED). Scheme ported from the other SuperOS firmwares.
//
// One image, one firmware active per boot, selected by a byte in the internal
// EEPROM. SuperOS keeps its patterns in the internal-flash block store; the
// emulator keeps its µPD444C pattern store in the internal EEPROM, which
// SuperOS never touches.
//
// STORAGE BUDGET. The 808 has four µPD444C RAMs (4096 nibbles = 2048 bytes
// packed) plus a 2 KB program image: 4 KB before any headers, against an
// internal EEPROM of exactly 4096 bytes. So the program image MUST be compiled
// in (-DD650_IMG_EMBEDDED); the EEPROM then holds just the 2048-byte RAM image
// at 0x020..0x81F, leaving ~2 KB spare, and MIDI image upload is compiled out
// because there is nowhere to put it.
//
// Switching:
//   - SysEx `F0 7D 4D <fw> F7` from either firmware (0 = SuperOS, 1 = emulator)
//   - SuperOS -> emulator: the config menu (double-tap CLEAR, then step 9)
//   - emulator -> SuperOS: double-press CLEAR. The emulated machine has no menu
//     to put this in, so the AVR bridge owns the gesture and keeps the key away
//     from the emulated CPU until it resolves (see emu_avr.cpp).
//   - power-on escape: hold STEP 1 + STEP 16 while switching on -> boots SuperOS
//     regardless of the stored selection. Step keys are used deliberately: they
//     are the one input whose matrix position needs no calibration, and this
//     probe runs before the panel table is loaded.
//
// Internal EEPROM (only the selector lives here, so it can be read before any
// driver init):
//   0x000  firmware select: 0/0xFF = SuperOS, 1 = emulator
//
//   0x001            emulator-area magic (EE_EMU_MAGIC_VAL when initialized)
//   0x00A            open the config menu on the next SuperOS boot (one-shot)
//   0x00B..0x01F     G5 cycle-probe record, 21 B (bench builds only)
//   0x020..0x81F     µPD444C packed pattern store (D650_EXT_BYTES = 2048)
//   0x820..0xFFF     TELEMETRY, bench builds only. See EE_TLM_* below.
//
// EVERY ADDRESS IN THAT MAP IS AN INTEGER CONSTANT BELOW AND EVERY BLOCK IS
// DERIVED FROM THE ONE BEFORE IT. Three overlaps have been found here by
// measurement rather than by reading, two of them in one instrument in one day:
// the cycle-probe record ran past its base into EE_EMU_PATT, three auxiliary
// blocks sat inside EE_IMG_DATA, and the PC profile's ready marker wrote 0x81F,
// which is the LAST BYTE of the pattern store. A telemetry address that lands
// in a data region is silent by construction: it corrupts the thing being
// measured and reads as a fault in it. Hardcoding the next address is what made
// all three possible, so nothing here is hardcoded any more.
#pragma once
#include <stdint.h>

// Addresses are integers; the pointer spellings below are derived from them.
// static_assert on a pointer macro does not compile everywhere -- a cast from
// pointer to integer is not a constant expression under clang, and the whole
// set fails with "static assertion expression is not an integral constant
// expression" (A101 hit this first). avr-gcc accepts it as an extension, so the
// asserts on this tree happened to build; that is luck, not portability, and
// the failure mode is somebody deleting the assert to make the build go green.
static constexpr uint16_t EE_SIZE_BYTES = 4096;   // AT90USB1286 internal EEPROM

static constexpr uint16_t EE_FW_SELECT_ADDR    = 0x000;
static constexpr uint16_t EE_EMU_MAGIC_ADDR    = 0x001;
static constexpr uint16_t EE_IMG_MAGIC_ADDR    = 0x008;
static constexpr uint16_t EE_IMG_SUM_ADDR      = 0x009;
static constexpr uint16_t EE_MENU_ON_BOOT_ADDR = 0x00A;

// The one telemetry block that lives BELOW the pattern store, because it has to
// be readable in a short ISP dump. 21 B at 0x00B ends at 0x01F, exactly abutting
// EE_EMU_PATT; the assert is in emu_avr.cpp beside the record that defines it.
static constexpr uint16_t EE_CYC_PROBE_ADDR  = 0x00B;
static constexpr uint16_t EE_CYC_PROBE_BYTES = 21;

static constexpr uint16_t EE_EMU_PATT_ADDR  = 0x020;
static constexpr uint16_t EE_EMU_PATT_BYTES = 2048;   // == D650_EXT_BYTES
static constexpr uint16_t EE_IMG_DATA_ADDR  = EE_EMU_PATT_ADDR + EE_EMU_PATT_BYTES;

// ---- telemetry, bench builds only ------------------------------------------
// Derived, never hardcoded: change a size and the next block moves with it.
// EE_IMG_DATA (the uploadable program image) starts at the same address, but a
// combined build cannot have one -- 2048 pattern + 2048 image + the 32-byte
// header is 4128 against a 4096-byte EEPROM, which is what the #error below
// says -- and every probe env is a combined build. So this region is free on
// exactly the builds that publish telemetry into it. THAT IS A CONDITION, NOT A
// PROPERTY: if a probe flag is ever enabled on a non-combined build, these
// blocks land on the uploaded image and corrupt it every window.
// Written out, NOT derived, so the assert below has something to catch. If this
// were EE_EMU_PATT_ADDR + EE_EMU_PATT_BYTES the assert would compare a value
// with itself and could never fail, which is the same as not having one.
static constexpr uint16_t EE_TLM_BASE = 0x820;

static constexpr uint16_t EE_TLM_PROF_ADDR  = EE_TLM_BASE;          // PC histogram
static constexpr uint16_t EE_TLM_PROF_BYTES = 32;
static constexpr uint16_t EE_TLM_PROF_RDY_ADDR = EE_TLM_PROF_ADDR + EE_TLM_PROF_BYTES;
static constexpr uint16_t EE_TLM_PROF_RDY_BYTES = 1;

static constexpr uint16_t EE_TLM_BENCH_ADDR  = EE_TLM_PROF_RDY_ADDR + EE_TLM_PROF_RDY_BYTES;
static constexpr uint16_t EE_TLM_BENCH_BYTES = 12;   // microbench r[12]
static constexpr uint16_t EE_TLM_BENCH_Q_ADDR  = EE_TLM_BENCH_ADDR + EE_TLM_BENCH_BYTES;
static constexpr uint16_t EE_TLM_BENCH_Q_BYTES = 4;
static constexpr uint16_t EE_TLM_BENCH_Z_ADDR  = EE_TLM_BENCH_Q_ADDR + EE_TLM_BENCH_Q_BYTES;
static constexpr uint16_t EE_TLM_BENCH_Z_BYTES = 4;

static constexpr uint16_t EE_TLM_SVC_ADDR  = EE_TLM_BENCH_Z_ADDR + EE_TLM_BENCH_Z_BYTES;
static constexpr uint16_t EE_TLM_SVC_BYTES = 12;

static constexpr uint16_t EE_TLM_RST_ADDR  = EE_TLM_SVC_ADDR + EE_TLM_SVC_BYTES;
static constexpr uint16_t EE_TLM_RST_BYTES = 4;
static constexpr uint16_t EE_TLM_PNL_ADDR  = EE_TLM_RST_ADDR + EE_TLM_RST_BYTES;
static constexpr uint16_t EE_TLM_PNL_BYTES = 11;
static constexpr uint16_t EE_TLM_SEEN_ADDR = EE_TLM_PNL_ADDR + EE_TLM_PNL_BYTES;
static constexpr uint16_t EE_TLM_SEEN_BYTES = 10;

static constexpr uint16_t EE_TLM_END = EE_TLM_SEEN_ADDR + EE_TLM_SEEN_BYTES;

// Both PROVEN to fire, not assumed (L9: an assert that has never failed is
// indistinguishable from one whose condition is inverted). EE_TLM_BASE = 0x81F
// fails the first; EE_TLM_SEEN_BYTES = 4000 fails the second.
//
// These cover the bounds. They do NOT cover the class, because an assert can
// only see the addresses it is given and every overlap found here so far came
// from a HARDCODED address at a call site that no assert mentioned.
// tools/diag/ee_map_check.py resolves every write target out of the source and
// is the check that would have caught all three.
static_assert(EE_TLM_BASE >= EE_EMU_PATT_ADDR + EE_EMU_PATT_BYTES,
              "telemetry starts inside the emulator's pattern store: it would "
              "overwrite pattern bytes every window");
static_assert(EE_TLM_END <= EE_SIZE_BYTES,
              "telemetry runs past the end of the internal EEPROM");

#define EE_FW_SELECT   ((uint8_t *)EE_FW_SELECT_ADDR)

// Internal-EEPROM offsets for the emulator area.
#define EE_EMU_MAGIC   ((uint8_t *)EE_EMU_MAGIC_ADDR)
#define EE_IMG_MAGIC   ((uint8_t *)EE_IMG_MAGIC_ADDR)
#define EE_IMG_SUM     ((uint8_t *)EE_IMG_SUM_ADDR)
// Set by the emulator just before it switches back, so SuperOS comes up with
// the config menu already open: the CLEAR double-press means "menu" on both
// sides, and landing in play mode instead made the trip back feel like a
// different gesture from the one that got you there.
#define EE_MENU_ON_BOOT ((uint8_t *)EE_MENU_ON_BOOT_ADDR)
static constexpr uint8_t EE_MENU_ON_BOOT_VAL = 0xA7;

#define EE_EMU_PATT    ((uint8_t *)EE_EMU_PATT_ADDR)
#define EE_IMG_DATA    ((uint8_t *)EE_IMG_DATA_ADDR)

static constexpr uint8_t FW_SUPEROS = 0;   // also 0xFF (virgin EEPROM)
static constexpr uint8_t FW_D650    = 1;

#ifdef SUPEROS_COMBINED
#ifndef D650_IMG_EMBEDDED
#error "Combined builds must embed the program image (-DD650_IMG_EMBEDDED): the uPD444C image plus an uploadable program image is 4 KB and the internal EEPROM is only 4096 bytes. Run tools/d650/make_image_embed.py first."
#endif
// Uploadable-image storage exists only where there is room for it.
#endif

// --- emulator storage backend ----------------------------------------------
// Thin names over the avr-libc EEPROM calls, kept as a seam so the store can be
// swapped without touching the emulator.
#include <avr/eeprom.h>
inline uint8_t d650_st_read(const uint8_t *ee) { return eeprom_read_byte(ee); }
inline void d650_st_write(uint8_t *ee, uint8_t v) { eeprom_update_byte(ee, v); }
inline void d650_st_read_block(void *dst, const void *ee, uint16_t n) { eeprom_read_block(dst, ee, n); }
inline void d650_st_write_block(const void *src, void *ee, uint16_t n) { eeprom_update_block(src, ee, n); }
inline bool d650_st_ready() { return eeprom_is_ready(); }

// Bump this whenever the seeded store layout changes; it forces every device to
// re-seed once (emulator patterns reset on that one boot; SuperOS patterns live
// in the separate block store and are untouched).
// 0x82: the /WE polarity fix (d650_host.h). Under the old polarity the store was
// write-transparent most of the time and the CPU's stale data latch was
// splattered across read addresses, so any image saved before this point is
// corrupt. Bumping forces one clean re-seed.
static constexpr uint8_t EE_EMU_MAGIC_VAL = 0x82;
static constexpr uint8_t EE_IMG_MAGIC_VAL = 0x8D;

// Select the other firmware and reboot through the bootloader's app entry.
void combined_switch_firmware(uint8_t fw);

// Mount the block store and load the calibrated panel decode table into
// g_panel_map. Lives in main.cpp because that translation unit owns the store's
// globals, but BOTH firmwares need it: the emulator reads the status-group bit
// indices (RUN / TAP / FILL IN / TEMPO CLOCK) out of the same table, and with a
// zero-initialised map its clock bit is 0 — the START/STOP line — which makes the
// emulated machine advance one step per RUN press and then stall.
void panel_map_boot_load();
