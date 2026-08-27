# make_boot_syx.py -- post-script for env:bootloader: pack the freshly built
# bootloader into bootloader-update.syx, the file that replaces the RESIDENT
# bootloader over USB-C or DIN with no programmer attached.
#
# This is not the same shape as an app update. An app .syx is consumed by the
# bootloader; this one is consumed by the RUNNING APP (SysEx 0x70/0x71), which
# stages it in application-region flash and only then calls the boot-section
# SPM service to copy it over the bootloader's pages. See src/boot_update.h.
#
#   F0 7D 70 <idx 0-7> <xor_lo7> <xor_hi1> <packed 256 B> F7   x8
#   F0 7D 71 <crc0> <crc1> <crc2> F7                           commit
#
# The commit CRC16-CCITT covers the whole 2048-byte image exactly as it will sit
# in the staging window, 0xFF fill included, so a page that never arrived cannot
# pass. Send it with tools/send_boot_update.py.
Import("env")
import hashlib
import os, sys

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex
from intelhex import IntelHex

PAGE_SIZE = 256
# Page 0x1F0 is the bootloader's PERMANENT page: the reset stub and the routine
# that finishes an interrupted commit. It is deliberately NOT carried here, so
# an update can never damage the code that would rescue it. Changing that page
# is an ISP job. What this file packs is the body, pages 0x1F1..0x1F8.
STUB_BASE = 0x1F000
BOOT_FIRST_PAGE = 0x1F1
BOOT_PAGES = 8
BOOT_BASE = BOOT_FIRST_PAGE * PAGE_SIZE          # 0x1F100
BOOT_END = BOOT_BASE + BOOT_PAGES * PAGE_SIZE    # 0x1F900, exclusive

CMD_STAGE = 0x70
CMD_COMMIT = 0x71

# Fingerprint of the permanent page. That page is not carried by an update, so a
# build whose page 0x1F0 differs from the one already on a device cannot be
# delivered over the wire: the device would keep its old stub and get the new
# body. Nothing in that page should ever move (it holds only constants: the
# stack setup, a call to the recovery routine at 0x1F020 and a jump to the
# body's fixed doorway at 0x1F100), so a change here means someone edited
# boot_entry or boot_recover, or a compiler change re-encoded them.
#
# If this fires: that is not a bug in this script. It means the build must be
# installed over ISP on every device, and only once every device has it should
# this constant be updated to the new value.
PERMANENT_PAGE_SHA256 = "16a6d537e53c7729d75e48bf30e1b73a61c69270253754f1d024bca01bda22f3"

out_path = os.path.join(env["PROJECT_DIR"], "bootloader-update.syx")


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make(target, source, env):
    hex_path = str(source[0])
    ih = IntelHex(hex_path)
    lo, hi = ih.minaddr(), ih.maxaddr()

    # The staged image is copied verbatim onto 0x1F100..0x1F8FF. The build must
    # start at the stub page and its body must fit those eight pages exactly: an
    # image that started elsewhere would land shifted, and one that overran would
    # be truncated into a bootloader that jumps into the middle of itself.
    # Refuse both. This file's whole job is to be safe.
    if lo != STUB_BASE:
        sys.stderr.write(
            "make_boot_syx: FATAL: bootloader starts at 0x%X, not at the stub "
            "page 0x%X. Refusing to build bootloader-update.syx. Check "
            "--section-start=.boot_entry in [env:bootloader].\n" % (lo, STUB_BASE))
        env.Exit(1)
        return
    if hi >= BOOT_END:
        sys.stderr.write(
            "make_boot_syx: FATAL: bootloader body runs to 0x%X, past the eight "
            "pages 0x%X..0x%X this updater replaces. Refusing to build "
            "bootloader-update.syx. The body outgrew 2 KB: either shrink it, or "
            "grow BOOT_PAGES here, BOOTUPD_PAGES in src/boot_update.h and the "
            "staging window in src/flash_service/flash_service.c together.\n"
            % (hi, BOOT_BASE, BOOT_END - 1))
        env.Exit(1)
        return

    stub = bytes(ih.tobinarray(start=STUB_BASE, size=PAGE_SIZE))
    got = hashlib.sha256(stub).hexdigest()
    if got != PERMANENT_PAGE_SHA256:
        sys.stderr.write(
            "make_boot_syx: FATAL: the permanent page 0x%X changed (sha256 %s, "
            "expected %s). An over-the-wire update does NOT carry that page, so "
            "delivering this build would leave every device running its old stub "
            "against a new body. Install this build over ISP everywhere, then "
            "update PERMANENT_PAGE_SHA256 in this file to the new value.\n"
            % (STUB_BASE // PAGE_SIZE, got, PERMANENT_PAGE_SHA256))
        env.Exit(1)
        return

    image = bytes(ih.tobinarray(start=BOOT_BASE, size=BOOT_PAGES * PAGE_SIZE))

    with open(out_path, "wb") as f:
        for idx in range(BOOT_PAGES):
            page = image[idx * PAGE_SIZE:(idx + 1) * PAGE_SIZE]
            x = 0
            for b in page:
                x ^= b
            msg = [0xF0, hex2sysex.MFR_ID, CMD_STAGE, idx, x & 0x7F, (x >> 7) & 0x01]
            msg.extend(hex2sysex.pack_7bit(page))
            msg.append(0xF7)
            f.write(bytes(msg))
        crc = crc16(image)
        f.write(bytes([0xF0, hex2sysex.MFR_ID, CMD_COMMIT,
                       crc & 0x7F, (crc >> 7) & 0x7F, (crc >> 14) & 0x03, 0xF7]))

    print("make_boot_syx: %s -> bootloader-update.syx (%d body pages 0x%X..0x%X, "
          "crc16 0x%04X, %d bytes; permanent page 0x%X not included)"
          % (hex_path, BOOT_PAGES, BOOT_FIRST_PAGE, BOOT_FIRST_PAGE + BOOT_PAGES - 1,
             crc, os.path.getsize(out_path), STUB_BASE // PAGE_SIZE))


syx = env.Command(out_path, "$BUILD_DIR/${PROGNAME}.hex", make)
env.AlwaysBuild(syx)
env.Default(syx)
