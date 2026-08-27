Import("env")
import os, sys, subprocess

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex
import git_dirt

def get_git_rev():
    git_rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip()
    git_status = subprocess.check_output(["git", "status", "-s", "--untracked-files=no"]).decode()
    # `dirty` means "this image was built from a tree nobody can reconstruct",
    # so only files that can reach the binary count. Files that cannot (coord
    # logs, docs, captures) are REPORTED, never silently dropped: seeing the
    # list is what tells you the exemption is doing what you think.
    # gitprov.py in SuperOS-USBC applies the same blocklist to the tree but also
    # votes on this filename, so a crude verdict here overrides its own.
    build, cosmetic = git_dirt.split_dirt(git_status)
    if cosmetic:
        print("makesyx: %d modified file(s) cannot reach the image, not marking dirty: %s"
              % (len(cosmetic), ", ".join(cosmetic)))
    if build:
        print("makesyx: DIRTY, %d modified file(s) CAN reach the image: %s"
              % (len(build), ", ".join(build)))
    suffix = "dirty" if build else ""

    return git_rev + suffix

PAGE_SIZE = 256
# Brick-safety bound for boot-section pages carried in the .syx (same rule as
# make_service_syx.py): pages 0x1F0..0x1F8 hold the RUNNING bootloader, and a
# write there from the bootloader erases the code running the erase. The SPM
# flash service lives above it (0x1FE00..0x1FFFF), which is safe to write from
# the bootloader. Replacing the bootloader itself is the app's job and uses a
# different path entirely (make_boot_syx.py / src/boot_update.h).
SAFE_MIN_PAGE = 0x1F9
FLASH_END_PAGE = 0x1FFFF // PAGE_SIZE

def write_pages(ih, f, first_page, last_page):
    # Page-sparse: an all-0xFF page carries nothing and writing it would erase
    # whatever is there (the commit arm page and the staging window sit inside
    # the app's address span now that .natcode lives above them).
    for page in range(first_page, last_page + 1):
        data = ih.tobinarray(start=page * PAGE_SIZE, size=PAGE_SIZE)
        if all(b == 0xFF for b in data):
            continue
        f.write(hex2sysex.build_sysex(page, data))

# App layout guard. Two windows may hold app bytes: below 0xD700, and the
# .natcode region 0xE000..0x15FFF (translated emulator program). In between sit
# the bootloader-commit arm page (0xD700) and the staging window (0xD800+): app
# bytes at 0xD700 could read as the arm magic and make the reset stub "finish"
# a commit that never happened. 0x17E00+ is the pattern arena.
# Pages the app image may occupy: below the commit-arm page, or in the .natcode
# window (0xE000 up to the arena base, flash_eeprom.h). Anything else lands in
# the staging window or the arena and would be erased under the running system.
def app_page_ok(page):
    return page < 0xD7 or (0xE0 <= page < 0x17E)

# The 808's pattern arena. FLASH_ARENA_BASE in src/flash_store.h:34; passed to
# flash_layout so the ceiling arithmetic sees the real layout.
ARENA_BASE = 0x17E00

# The REAL app ceiling, and it is not the one PlatformIO prints. The arithmetic
# lives in SuperOS-USBC/tools/flash_layout.py (CORE, claim-084) so five machines
# cannot diverge on it; this file only supplies the segments and says whether
# this machine hosts the staging window. The 808 does.
#
# Adopting it fixed an off-by-one in the version this file carried first:
# `ceiling - top` counts the top byte itself as free, so it reported 618 free at
# top 0xD496 where the true figure is 617. One byte, reported optimistically, in
# a check whose whole job is to stop an overrun that has twice been under 200
# bytes on this machine.
#
# REPORT EARLY, REFUSE LATE, and they are not alternatives. The guard below
# REFUSES and is load-bearing: it is what stopped a broken image being flashed.
# This REPORTS, on every `pio run`, because the number's value is knowing the
# budget before spending it. The choke told me only that I had already failed.
# __file__ is NOT defined here: PlatformIO exec()s this script, so the path has
# to come from PROJECT_DIR at call time.
def report_headroom(app, env):
    # The fleet's shared copy wins where it exists (a development checkout with
    # the sibling repo); a public checkout carries a vendored copy in its own
    # tools/, so the ceiling check never goes quiet on a release build.
    shared = os.path.normpath(os.path.join(
        env["PROJECT_DIR"], "..", "..", "SuperOS-USBC", "tools"))
    local = os.path.join(env["PROJECT_DIR"], "tools")
    try:
        if os.path.exists(os.path.join(shared, "flash_layout.py")):
            if shared not in sys.path:
                sys.path.insert(0, shared)
        elif local not in sys.path:
            sys.path.insert(0, local)
        import flash_layout as fl
    except ImportError:
        print("makesyx: flash_layout.py found neither at %s nor in tools/; "
              "headroom NOT reported (this is the check going quiet, not a "
              "pass)" % shared)
        return
    # arena_base is now required and must be the REAL base, not fl.NO_ARENA:
    # A606 measured that "the boot section is the ceiling" holds only where the
    # arena sits outside the application region. The 606's does not (0x0E000
    # through 0x1E000), and the module reported 74842 bytes free against a true
    # 5210. Whether the arena is above every ceiling is the module's conclusion
    # to draw, not this file's to assert. CORE, cc55d7d.
    ceiling = fl.app_ceiling(hosts_staging=True,     # the 808 hosts it
                             arena_base=ARENA_BASE)
    top = fl.app_top(app.segments(), ceiling)
    print(fl.report(top, hosts_staging=True, arena_base=ARENA_BASE,
                    label="makesyx"))

    # THE SECOND CEILING, and until claim-114 nothing reported it. The emulator
    # window (.d650img + .natcode, 0xE000 up) is app-carrying flash too, and the
    # thing directly above it is the PATTERN ARENA. app_page_ok() refuses an
    # image whose pages reach 0x17E, so a collision cannot ship -- but a refusal
    # tells you only that you have already failed, which is the same argument
    # the app-ceiling report above was added for.
    #
    # It matters more now than it did: moving the 2 KB program image up to
    # 0xE000 bought 2048 bytes under the 0xD700 ceiling and spent them out of
    # THIS clearance. Whoever regenerates the translated core is the one who
    # will find out, and they should find out here rather than from a pattern
    # store that quietly stopped holding patterns.
    hi = max((seg[1] for seg in app.segments() if seg[0] >= 0xE000), default=None)
    if hi is not None:
        free = ARENA_BASE - hi
        print("makesyx: emulator window 0x0E000..0x%05X, %d bytes clear of the "
              "pattern arena at 0x%05X" % (hi - 1, free, ARENA_BASE))
        if free < 1024:
            print("makesyx:   WARNING: under 1 KB of clearance; a larger "
                  ".natcode would land in the pattern store")


def make_syx(target, source, env):
    hex_path = str(source[0])
    out_path = str(target[0])
    print(f"makesyx: {hex_path} -> {out_path}")
    from intelhex import IntelHex
    app = IntelHex(hex_path)
    report_headroom(app, env)
    for seg_lo, seg_hi in app.segments():
        for page in range(seg_lo // PAGE_SIZE, (seg_hi - 1) // PAGE_SIZE + 1):
            if not app_page_ok(page):
                sys.stderr.write(
                    "makesyx: FATAL: app data on page 0x%X, inside the "
                    "commit/staging window (0xD700-0xDFFF) or the arena "
                    "(0x17E00+). Fix the layout before shipping this.\n" % page)
                env.Exit(1)
    service_hex = os.path.join(env["PROJECT_DIR"], ".pio", "build",
                               "flash-service", "firmware.hex")
    with open(out_path, "wb") as f:
        # app pages (from 0)
        write_pages(app, f, 0, app.maxaddr() // PAGE_SIZE)
        # SPM flash service pages, so one .syx install also (re)installs the
        # service — no separate service-install.syx step for MIDI users.
        if os.path.exists(service_hex):
            svc = IntelHex(service_hex)
            lo, hi = svc.minaddr() // PAGE_SIZE, svc.maxaddr() // PAGE_SIZE
            if lo < SAFE_MIN_PAGE or hi > FLASH_END_PAGE:
                sys.stderr.write(
                    "makesyx: FATAL: flash-service image spans pages "
                    "0x%X..0x%X, outside the safe boot window (>= 0x%X). "
                    "Refusing: writing the running bootloader would brick "
                    "the device.\n" % (lo, hi, SAFE_MIN_PAGE))
                env.Exit(1)
            write_pages(svc, f, lo, hi)
            print("makesyx: bundled flash service (pages 0x%X..0x%X)" % (lo, hi))
        else:
            print("makesyx: *** NO SPM SERVICE IN THIS .syx ***  the pattern "
                  "store will be DEAD on any device this installs: writes ack "
                  "off the RAM cache and nothing reaches flash (claim-036). "
                  "Build it: pio run -e flash-service. Output is named "
                  "-noservice so the file says so.")
        # cmd 0x02 = execute/jump-to-app
        f.write(bytes([0xF0, hex2sysex.MFR_ID, hex2sysex.CMD_EXECUTE, 0xF7]))
    print("makesyx: wrote %d bytes" % os.path.getsize(out_path))
    make_full_hex(hex_path, out_path[:-4] + "-full.hex", env)

def make_full_hex(app_hex, out_path, env):
    # Merge the complete ISP image: this app + the MIDI bootloader + the SPM
    # flash service. ISP flashing chip-erases the whole part, so a bare app
    # hex silently removes the bootloader and the service; the -full hex is
    # the one to flash with avrdude. Runs on every app build so it can never
    # go stale; build the other two envs first:
    #   pio run -e bootloader -e flash-service -e combined
    from intelhex import IntelHex
    boot = os.path.join(env["PROJECT_DIR"], ".pio", "build", "bootloader", "firmware.hex")
    service = os.path.join(env["PROJECT_DIR"], ".pio", "build", "flash-service", "firmware.hex")
    if not os.path.exists(boot):
        print("makesyx: no bootloader hex, skipping -full.hex (pio run -e bootloader first)")
        return
    def load(path):
        h = IntelHex(path)
        h.start_addr = None   # drop ELF entry records; they differ per image
        return h
    ih = load(app_hex)
    ih.merge(load(boot))
    if os.path.exists(service):
        ih.merge(load(service))
    else:
        print("makesyx: *** NO SPM SERVICE IN THIS -full.hex ***  ISP-flashing "
              "it leaves the pattern store dead (claim-036). "
              "pio run -e flash-service, then rebuild.")
    ih.write_hex_file(out_path)
    print(f"makesyx: wrote {os.path.basename(out_path)} (app + bootloader"
          + (" + flash service)" if os.path.exists(service) else ")"))

git_rev = get_git_rev()
pioenv = env["PIOENV"]
# An image built without the SPM flash service is as dangerous as one built from
# a tree nobody can reconstruct, and for the same reason: you cannot tell by
# looking at it. Installing it leaves the pattern store dead while every
# read-back still succeeds off the RAM cache, so writes ack and nothing
# persists until a reboot exposes it (claim-036, found on the 606). The old
# behaviour printed a WARNING mid-build and named the output exactly like a good
# one. Mark it in the filename instead, the way `dirty` already marks the tree.
service_missing = not os.path.exists(
    os.path.join(env["PROJECT_DIR"], ".pio", "build", "flash-service", "firmware.hex"))
if service_missing:
    git_rev += "-noservice"
progname = f"SuperOS-808_v%s_{pioenv}_{git_rev}" % env.GetProjectOption("custom_project_version")
syx_path = os.path.join(env["PROJECT_DIR"], f"{progname}.syx")

# Command node: always rebuild the .syx regardless of whether the .hex changed.
# AlwaysBuild marks it unconditionally out-of-date every run.
# Default adds it to the targets built by plain `pio run`.
syx = env.Command(syx_path, "$BUILD_DIR/${PROGNAME}.hex", make_syx)
env.AlwaysBuild(syx)
env.Default(syx)
