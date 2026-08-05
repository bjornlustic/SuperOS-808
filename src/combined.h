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
//   0x020..0x81F      µPD444C packed pattern store (D650_EXT_BYTES = 2048)
#pragma once
#include <stdint.h>

#define EE_FW_SELECT   ((uint8_t *)0x000)

// Internal-EEPROM offsets for the emulator area.
#define EE_EMU_MAGIC   ((uint8_t *)0x001)
#define EE_IMG_MAGIC   ((uint8_t *)0x008)
#define EE_IMG_SUM     ((uint8_t *)0x009)
#define EE_EMU_PATT    ((uint8_t *)0x020)
#define EE_IMG_DATA    ((uint8_t *)0x820)

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
