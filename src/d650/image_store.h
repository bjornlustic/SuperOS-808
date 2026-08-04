// image_store.h -- user-supplied µPD650C-085 program image: persistence + the
// SysEx receiver (combined build only).
//
// The firmware ships WITHOUT any program image. The user reads out their own
// chip (or uses a transfer they are legally entitled to) and uploads it over DIN
// MIDI in the RE-303/recpu nibble format:
//
//   F0 7D 03 03 7E 7F <info...> F7             metadata block, ignored
//   F0 7D 03 03 7E 03 <blk> <nibbles> F7       data: hi nibble first,
//                                              last decoded byte = sum8 of the
//                                              block's data bytes
//   F0 7D 03 03 7F 00 F7                       end mark
//
// Block sizes are NOT uniform: the observed chunking is 507 + 507 + 507 + 507 +
// 20 data bytes across five blocks. So the receiver must not assume a block
// size — it appends at a running offset and only checks the total at the end
// mark.
//
// There is nowhere to store an uploaded image on this board, so the whole store
// and its receiver are compiled out (D650_IMG_UPLOADABLE is never defined; see
// the storage budget in combined.h). EE_IMG_DATA would run from 0x820 to
// 0x101F, past the end of the 4096-byte internal EEPROM, and the writes would
// wrap onto the pattern store. The image is compiled into flash instead.
#pragma once
#include <avr/wdt.h>
#include "../combined.h"

#define D650_IMG_SIZE 2048

#ifdef D650_IMG_UPLOADABLE

static inline uint16_t img_sum16(const uint8_t *p) {
  uint16_t s = 0;
  for (uint16_t i = 0; i < D650_IMG_SIZE; ++i) s += p[i];
  return s;
}

// Store -> dst. True only if a stored image is present and checksums clean.
static bool img_load(uint8_t *dst) {
  if (d650_st_read(EE_IMG_MAGIC) != EE_IMG_MAGIC_VAL) return false;
  d650_st_read_block(dst, EE_IMG_DATA, D650_IMG_SIZE);
  const uint16_t want = (uint16_t)d650_st_read(EE_IMG_SUM)
                      | ((uint16_t)d650_st_read(EE_IMG_SUM + 1) << 8);
  return img_sum16(dst) == want;
}

// src -> store, in 64-byte chunks. The magic is invalidated first and rewritten
// last, so a torn write can never validate at boot.
static void img_save(const uint8_t *src) {
  d650_st_write(EE_IMG_MAGIC, 0xFF);
  for (uint16_t i = 0; i < D650_IMG_SIZE; i += 64) {
    d650_st_write_block(src + i, EE_IMG_DATA + i, 64);
    wdt_reset();
  }
  const uint16_t s = img_sum16(src);
  d650_st_write(EE_IMG_SUM,     (uint8_t)(s & 0xFF));
  d650_st_write(EE_IMG_SUM + 1, (uint8_t)(s >> 8));
  d650_st_write(EE_IMG_MAGIC, EE_IMG_MAGIC_VAL);
}

// ---------------------------------------------------------------------------
// SysEx receiver
// ---------------------------------------------------------------------------
// Fed one byte at a time from the emulator's MIDI poll. Returns true exactly
// once, on the end mark, when a complete and checksum-clean 2048-byte image has
// been assembled in the buffer passed to Bind().
//
// Verified against a real transfer: five blocks of 507/507/507/507/20 data
// bytes, every trailing sum8 correct, concatenating to exactly 2048 bytes.
class ImgRx {
 public:
  void Reset() { state_ = 0; off_ = 0; have_hi_ = false; }

  // Bytes are decoded straight into the bound destination buffer — no staging buffer, because on this part 600 spare
  // bytes of SRAM is real money. A block whose checksum fails is rolled back by
  // rewinding the write offset to where the block started, so a bad block
  // corrupts nothing.
  bool Byte(uint8_t b) {
    if (b == 0xF0) { state_ = 1; hdr_ = 0; return false; }
    if (b == 0xF7) {
      bool done = false;
      if (state_ == 3) commit_block_();
      else if (state_ == 4) done = (off_ == D650_IMG_SIZE);
      state_ = 0;
      return done;
    }
    if (b & 0x80) { state_ = 0; return false; }   // any other status aborts

    switch (state_) {
      case 1:                                     // header: 7D 03 03 7E/7F
        if (hdr_ == 0 && b != 0x7D) { state_ = 0; break; }
        if (hdr_ == 1 && b != 0x03) { state_ = 0; break; }
        if (hdr_ == 2 && b != 0x03) { state_ = 0; break; }
        if (hdr_ == 3) {
          if (b == 0x7E)      state_ = 2;         // data or info block follows
          else if (b == 0x7F) state_ = 4;         // end mark
          else                state_ = 0;
          hdr_ = 0;
          break;
        }
        ++hdr_;
        break;

      case 2:                                     // block type
        if (b == 0x03) {
          state_ = 3;
          want_blk_ = true;
          have_hi_ = false;
          n_ = 0; sum_ = 0; last_ = 0;
        } else {
          state_ = 5;                             // info/other block: skip it
        }
        break;

      case 3: {                                   // block number, then nibbles
        if (want_blk_) {
          want_blk_ = false;
          if (b == 0) off_ = 0;                   // a fresh block 0 restarts
          blk_start_ = off_;
          break;
        }
        if (!have_hi_) { hi_ = (uint8_t)(b & 0x0F); have_hi_ = true; break; }
        have_hi_ = false;
        const uint8_t v = (uint8_t)((hi_ << 4) | (b & 0x0F));
        // The previous byte is data only once we know another follows; the very
        // last byte of the block is its checksum.
        if (n_) {
          if (off_ < D650_IMG_SIZE) dst_buf_[off_++] = last_;
          sum_ = (uint8_t)(sum_ + last_);
        }
        last_ = v;
        ++n_;
        break;
      }

      default:                                    // 4 = end mark, 5 = skipping
        break;
    }
    return false;
  }

  // The receiver needs the destination pointer during Byte(); capture it once.
  void Bind(uint8_t *dst) { dst_buf_ = dst; }

 private:
  uint8_t *dst_buf_   = nullptr;
  uint16_t off_       = 0;
  uint16_t blk_start_ = 0;
  uint16_t n_         = 0;      // decoded bytes seen in this block
  uint8_t  state_     = 0;
  uint8_t  hdr_       = 0;
  uint8_t  hi_        = 0;
  uint8_t  last_      = 0;      // most recent decoded byte (maybe the checksum)
  uint8_t  sum_       = 0;      // sum of the bytes already committed
  bool     have_hi_   = false;
  bool     want_blk_  = false;

  // `last_` holds the block's trailing checksum; everything before it is data
  // and is already written. Keep it if the sum matches, roll back if not.
  void commit_block_() {
    if (n_ < 2) { off_ = blk_start_; return; }
    if (sum_ != last_) off_ = blk_start_;         // bad block: drop it whole
  }
};

#else   // !D650_IMG_UPLOADABLE — no storage, so no receiver

static inline bool img_load(uint8_t *) { return false; }
class ImgRx {
 public:
  void Bind(uint8_t *) {}
  void Reset() {}
  bool Byte(uint8_t) { return false; }
};

#endif  // D650_IMG_UPLOADABLE
