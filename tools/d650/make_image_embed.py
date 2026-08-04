#!/usr/bin/env python3
"""make_image_embed.py -- generate src/d650/image_embed.h from your own µPD650C.

For PERSONAL builds only. The µPD650C-085 program is copyrighted: the generated
header is gitignored and must never be committed or distributed. This project
ships no program image at all — you supply your own, from a chip you are legally
entitled to read out.

Input: either a raw 2048-byte binary, or a recpu-format .syx transfer (the same
wire format image_store.h receives):

  F0 7D 03 03 7E 7F <info...> F7                           (metadata, skipped)
  F0 7D 03 03 7E 03 <blk> <nibbles hi,lo> F7               (data)
  F0 7D 03 03 7F 00 F7                                     (end marker)

The last decoded byte of each block is the mod-256 sum of the ones before it.
Block sizes are NOT uniform and must not be assumed: the 808's chunking is
507+507+507+507+20. Blocks are
appended at a running offset and the total is checked at the end. Checksums are
verified per block.

Usage: python3 tools/d650/make_image_embed.py <transfer.syx|raw-2048-bytes>
Writes: src/d650/image_embed.h
"""
import sys
from pathlib import Path

IMG_SIZE = 2048
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "src" / "d650" / "image_embed.h"


def decode_syx(data: bytes) -> bytes:
    prg = bytearray()
    i = 0
    nblk = 0
    while i < len(data):
        if data[i] != 0xF0:
            i += 1
            continue
        end = data.find(0xF7, i)
        if end < 0:
            break
        msg = data[i + 1 : end]
        i = end + 1
        if len(msg) < 5 or msg[:3] != bytes((0x7D, 0x03, 0x03)):
            continue
        if msg[3] == 0x7F:                      # end marker
            break
        if msg[3] != 0x7E or msg[4] != 0x03:    # metadata / unknown: skip
            continue
        blk, nibs = msg[5], msg[6:]
        if len(nibs) < 4 or len(nibs) % 2:
            sys.exit(f"error: malformed block {blk} ({len(nibs)} nibbles)")
        if any(n > 0x0F for n in nibs):
            sys.exit(f"error: non-nibble data byte in block {blk}")
        by = bytes((nibs[2 * k] << 4) | nibs[2 * k + 1] for k in range(len(nibs) // 2))
        payload, ck = by[:-1], by[-1]
        s8 = sum(payload) & 0xFF
        if ck != s8:
            sys.exit(f"error: block {blk} checksum mismatch "
                     f"(file {ck:#04x}, computed {s8:#04x})")
        if blk == 0:
            prg = bytearray()                   # a fresh block 0 restarts
        prg += payload
        nblk += 1
    if len(prg) != IMG_SIZE:
        sys.exit(f"error: decoded {len(prg)} bytes from {nblk} blocks, expected {IMG_SIZE}")
    print(f"decoded {nblk} blocks -> {len(prg)} bytes, all checksums OK")
    return bytes(prg)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    src = Path(sys.argv[1])
    data = src.read_bytes()
    if len(data) == IMG_SIZE and 0xF0 not in data[:16]:
        prg = data
        print(f"raw binary dump: {IMG_SIZE} bytes")
    else:
        prg = decode_syx(data)
    sum16 = sum(prg) & 0xFFFF

    lines = [
        "// image_embed.h -- GENERATED, PERSONAL BUILDS ONLY. DO NOT COMMIT.",
        "// The µPD650C-085 program is copyrighted; this file is gitignored.",
        f"// Source: {src.name}, sum16 = 0x{sum16:04X}.",
        "// Regenerate: python3 tools/d650/make_image_embed.py <your transfer>",
        "#pragma once",
        "#include <avr/pgmspace.h>",
        "",
        f"static const uint8_t D650_IMG_EMBED[{IMG_SIZE}] PROGMEM = {{",
    ]
    for off in range(0, IMG_SIZE, 16):
        row = ", ".join(f"0x{b:02X}" for b in prg[off : off + 16])
        lines.append(f"  {row},")
    lines += ["};", ""]
    OUT.write_text("\n".join(lines))
    print(f"wrote {OUT} ({IMG_SIZE} bytes, sum16 0x{sum16:04X})")


if __name__ == "__main__":
    main()
