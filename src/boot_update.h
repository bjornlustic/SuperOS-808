// boot_update.h -- replace the resident bootloader over USB-C or DIN.
//
// The bootloader cannot rewrite itself: erasing 0x1F0..0x1F7 destroys the code
// running the erase. So the APP does it, through the boot-section SPM service
// (flash_service.c), and the two halves cover each other:
//
//   app + flash service  ->  rewrites the bootloader   (this file)
//   bootloader           ->  rewrites the flash service (service-install.syx)
//
// Neither ever programs its own pages, so no step can erase the code executing
// it, and either half can always be repaired from the other.
//
// The image is received into a 2 KB staging window in application-region flash
// (0xD800, just below the pattern arena), one page at a time. Nothing live is
// touched during the upload: a dropped cable, a bad checksum or a power cut
// costs only the staging area, and the upload is simply retried. Only when the
// host asks to commit, and only if a CRC16 over the whole staged image matches
// the one the host declared, are the eight pages copied onto the bootloader.
//
// The copy is power-fail safe. Only the bootloader's BODY (0x1F1..0x1F8) is
// ever a destination; page 0x1F0 holds its reset stub and a recovery routine
// and is never written from here, so whatever happens there is always an intact
// entry point to come back to. The service sets an arm flag in flash before the
// first erase and clears it after the last write, so a boot section that could
// be inconsistent always has the flag set; the stub reads it at reset and calls
// the service straight back to finish the copy from staging. Re-entry is
// idempotent, so it survives being interrupted again.
//
// What that costs: page 0x1F0 can only be changed over ISP. It is deliberately
// tiny and dull for exactly that reason.
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "flash_store.h"

static constexpr uint16_t BOOTUPD_PAGES       = 8;                 // 0x1F0..0x1F7
static constexpr uint16_t BOOTUPD_STAGE_FIRST = 0xD8;              // 0xD800
static constexpr uint32_t BOOTUPD_STAGE_BASE  = 0xD800UL;
static constexpr uint16_t BOOTUPD_IMAGE_BYTES = BOOTUPD_PAGES * 256;  // 2048

// Status, reported back over SysEx 0x73.
static constexpr uint8_t BOOTUPD_OK          = 0;
static constexpr uint8_t BOOTUPD_NO_SERVICE  = 1;  // old/absent flash service
static constexpr uint8_t BOOTUPD_BAD_INDEX   = 2;
static constexpr uint8_t BOOTUPD_BAD_LENGTH  = 3;
static constexpr uint8_t BOOTUPD_BAD_CKSUM   = 4;  // per-page XOR mismatch
static constexpr uint8_t BOOTUPD_WRITE_FAIL  = 5;  // service refused the write
static constexpr uint8_t BOOTUPD_BAD_CRC     = 6;  // staged image != host's CRC
static constexpr uint8_t BOOTUPD_BLANK       = 7;  // staging starts erased
static constexpr uint8_t BOOTUPD_COMMIT_FAIL = 8;

// CRC16-CCITT over the whole staging window, read straight out of flash.
// Because it covers all 2048 bytes, it doubles as the "did every page arrive"
// check: a partial upload leaves erased or stale bytes in the gap and cannot
// match. That is why no per-page arrival bitmap is kept, and why an interrupted
// upload needs no cleanup beyond sending the pages again.
inline uint16_t bootupd_staged_crc() {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < BOOTUPD_IMAGE_BYTES; ++i) {
    crc ^= (uint16_t)pgm_read_byte_far(BOOTUPD_STAGE_BASE + i) << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

// Receive one 256-byte page. idx 0..7 maps to bootloader page 0x1F0 + idx, but
// lands in staging: the service will not accept a boot-section page here at all.
inline uint8_t bootupd_stage(uint8_t idx, const uint8_t *page256) {
  if (idx >= BOOTUPD_PAGES) return BOOTUPD_BAD_INDEX;
  if (!flash_boot_service_present()) return BOOTUPD_NO_SERVICE;
  return flash_stage_page((uint16_t)(BOOTUPD_STAGE_FIRST + idx), page256) == FLASH_OK
             ? BOOTUPD_OK
             : BOOTUPD_WRITE_FAIL;
}

// Verify the staged image against the CRC the host computed over the same 2048
// bytes, then copy it onto the bootloader. Returns BOOTUPD_OK only if the new
// bootloader is in place.
inline uint8_t bootupd_commit(uint16_t want_crc) {
  if (!flash_boot_service_present()) return BOOTUPD_NO_SERVICE;

  // An erased first word means nothing was staged (or page 0 never arrived).
  // Cheap, and it catches the commonest operator error before the CRC walk.
  if (pgm_read_word_far(BOOTUPD_STAGE_BASE) == 0xFFFF) return BOOTUPD_BLANK;

  if (bootupd_staged_crc() != want_crc) return BOOTUPD_BAD_CRC;

  return flash_commit_boot() == FLASH_OK ? BOOTUPD_OK : BOOTUPD_COMMIT_FAIL;
}
