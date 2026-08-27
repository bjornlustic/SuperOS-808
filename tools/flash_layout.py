#!/usr/bin/env python3
"""The true application ceiling, in one place, because the toolchain's is wrong.

WHY THIS EXISTS
---------------
PlatformIO prints `Flash: 42.x% (used N from 130048)`. **That divides by the
PART, not by the layout.** The AT90USB1286 has 128 KB of flash, but an
application never gets all of it: the boot section, the flash service and (on a
machine hosting the three-entry flash service) the staging window and the commit
arm flag all sit inside the part and none of them is visible to the linker's
percentage.

A808 was bitten by this twice (2026-08-26). A build read **42% used with ~74 KB
apparently spare while the real margin was 170 bytes**, and the only thing that
enforced the true ceiling was `make_syx`, AFTER the link:

    combined-nat        linked 55578   REFUSED, app bytes on page 0xD7
    combined-nat-test   linked 56036   REFUSED
    real ceiling 0x0D700 = 55040

The first bite was recorded in a `platformio.ini` comment and never generalised,
so the second one was free to happen in the SHIPPING env. **claim-074 puts the
303 at 48 bytes of headroom.** This is not an 808 problem.

REPORT EARLY, REFUSE LATE. THEY ARE NOT ALTERNATIVES (A808, 2026-08-26)
------------------------------------------------------------------------
My first instinct was to put this at the send/flash choke and have the machines
drop their copies. **A808 argued that down and they were right.** The choke runs
when you are about to SHIP. The number's whole value is telling you the budget
BEFORE you spend 708 bytes on a feature, not after; A808 got the fatal at the
choke and it told them only that they had already failed. A guard at the choke
prevents SHIPPING a too-large image, which the existing page guard already does.
It does not prevent WRITING one.

So the two jobs are split and both are kept:

    the choke (make_full_hex.py, make_syx)   REFUSES.  Load-bearing; it is what
                                             stopped a broken image being flashed.
    every `pio run` (each makesyx.py)        REPORTS.  Has to be where the
                                             compile is.

WHY THE ARITHMETIC IS HERE AND NOT COPIED INTO FIVE REPOS
---------------------------------------------------------
The numbers below are the layout in `docs/BOOT_MAP.md`, which this repo owns and
which is pinned by `--section-start` flags. Five copies of this arithmetic is
five chances for one to drift from the map, and a repo that missed a correction
would be silently exempt: exactly the reasoning that put `gitprov` at the
send/flash choke rather than in each machine's `makesyx.py`.

So each machine's `makesyx.py` imports this and calls `report()`. The per-repo
code is a call, not a copy of the constants.

AN APP IMAGE IS NOT CONTIGUOUS, AND THAT IS WHERE THE OFF-BY-ONE LIVES
-----------------------------------------------------------------------
**`maxaddr()` is the wrong number and on the 808 it is wrong by 39 KB.** A808's
image is `.text` to 0x0D496, then the commit/staging window, then `.natcode` at
0x0E000 for 37958 bytes, then the arena at 0x17E00. `maxaddr()` returns ~0x1748A
and means nothing about headroom.

**The headroom is against the highest segment that ends at or below the ceiling**,
which is `app_top()` below. A machine with no `.natcode` does not need that rule
and a machine that puts something else above the window needs a different one, so
the rule is passed the segments and asked, rather than assuming contiguity.
A808 named this as the piece they would least want copied five times, because it
is exactly where an off-by-one would live and then diverge.

WHAT THIS MODULE CANNOT KNOW, AND WHY IT IS A PARAMETER
--------------------------------------------------------
**Whether a given build hosts the staging window is the BUILD's property, not the
layout's.** claim-069 measured that the 303 cannot host the three-entry flash
service's staging window (it is 2 pages short), so this genuinely differs per
machine and per env. This module will not guess it: `hosts_staging` is a required
argument. A default would be a wrong answer on half the fleet, silently, which is
the failure this module exists to end.

**`arena_base` is required for the same reason, and it is required because the
first version of this module got the 606 wrong** (A606, 2026-08-26). That version
read `CEILING_PLAIN = BOOT_BASE`, which encodes *"with no staging window the app
may run up to the boot section"*. **That holds only where the flash arena is
OUTSIDE the application region.** On the 606 it is not: the pattern arena is
`0x0E000..0x1E000` (606 `src/flash_store.h:18`), **64 KB through the middle of
the application region**. For env `combined-rom-usb-4mhz` at app top `0x0CBA5`:

    hosts_staging=True      2906 bytes free   ceiling 0x0D700
    hosts_staging=False    74842 bytes free   ceiling 0x1F000   <- WRONG
    the 606's truth         5210 bytes free   ceiling 0x0E000

**69632 of those 74842 bytes ARE the pattern store, and the failure is silent:
corrupted patterns, not a link error.** The 606 previously had no ceiling check
at all and `write_pages()` ran to `maxaddr()`, so an oversized app would have
written its own pages over the store.

**The lesson is the one in this module's own docstring, applied to itself:
`hosts_staging` was a PROXY for a layout question, and it was wrong on one of the
two machines it modelled.** A606 asked for the layout fact itself rather than a
third bool case, and they are right. The ceiling is now the MINIMUM of every
region the app must stop below, and `arena_base` is one of them: pass the address,
or `NO_ARENA` to state positively that this machine has none inside the
application region. Omitting it is a `TypeError`, so no existing caller can keep
the old wrong answer by doing nothing.

A machine whose layout this module still does not model has `ceiling=` as an
explicit escape, which is better than a fourth bool.

**THE MINIMUM IS ONLY AS GOOD AS THE REGION LIST, AND THE LIST IS NOT AUDITED**
(A606, 2026-08-26, on their own fix). The min-of-every-region form is the right
general shape, but **`0x0E000` is "the lowest region A606 knows of on the 606",
not "the complete list"**: nobody has audited whether anything else sits inside
that machine's application region, and no one has audited the other three at all.
**If another region is found, the fix is a NEW ARGUMENT, not a new constant** --
a constant would put the same silent assumption back one level down, which is how
`CEILING_PLAIN` got here.

**Two implementations that agree beat one shared one.** The 606's `makesyx.py`
does not import this module; it parses `FLASH_ARENA_BASE` out of its own
`src/flash_store.h`, so it fails loudly if the firmware layout moves. A606 ran
both and got the same 5210 by different routes. That cross-check is worth more
than the deduplication would be, and it is why this module is not pushed onto
every machine.
"""
import sys

# From docs/BOOT_MAP.md. Each is pinned by a linker --section-start.
PART_BYTES = 0x20000        # AT90USB1286, 128 KB. The physical part.
#: What PlatformIO actually divides by: the board's `maximum_size`, not the part.
#: A606 measured the difference (2026-08-26): this module used to print "39.8% of
#: 131072" next to a PlatformIO line reading "40.1% of 130048", and a guard whose
#: "the other tool says" figure disagrees with the other tool teaches the reader
#: to distrust it.
MAXIMUM_SIZE = 130048       # 0x1FC00
BOOT_BASE = 0x1F000         # boot section base (BOOTSZ=01, 4 KB)
ARM_FLAG = 0x0D700          # commit arm flag, inside the application region
STAGING_BASE = 0x0D800      # staging window, 8 pages, 0x0D800..0x0DFFF

#: Ceiling when the three-entry flash service is NOT resident: the app may run
#: up to the boot section.
CEILING_PLAIN = BOOT_BASE   # 0x1F000 = 126976

#: Ceiling when it IS resident. The app must stop below the ARM FLAG, not below
#: the staging window: the flag sits one page beneath it and is part of the
#: recovery path (BOOT_MAP rule 2). Stopping at STAGING_BASE would overwrite the
#: flag and lose the "interrupted commit repairs itself" property, which was
#: proven by sabotage and is not worth 256 bytes.
CEILING_STAGING = ARM_FLAG  # 0x0D700 = 55040


#: Pass as `arena_base` to state POSITIVELY that this machine has no flash arena
#: inside the application region. It is a sentinel and not `None` on purpose:
#: `None` is what a caller passes when it does not know, and not knowing is the
#: bug. The 808 passes its real base (0x17E00) even though that is above every
#: ceiling, because "above" is this module's conclusion to draw, not the
#: caller's.
NO_ARENA = "no-arena"


def app_ceiling(hosts_staging, arena_base, ceiling=None):
    """First address the application may NOT occupy.

    The ceiling is the MINIMUM of every region the app must stop below, so a
    machine can be constrained by more than one at once and the tightest wins.
    Both `hosts_staging` and `arena_base` are required; see the module docstring
    for why each one is, and which machine paid for it.

    `ceiling` is an explicit override for a layout this module does not model.
    It wins outright, and it is checked for sanity rather than trusted blindly.
    """
    if ceiling is not None:
        if not 0 < ceiling <= PART_BYTES:
            raise ValueError(f"explicit ceiling 0x{ceiling:X} is outside the part")
        return ceiling
    if hosts_staging is None:
        raise ValueError("hosts_staging must be True or False, never None: "
                         "guessing it is wrong on half the fleet, silently")
    if arena_base is None:
        raise ValueError(
            "arena_base must be an address or NO_ARENA, never None. It is the "
            "base of this machine's flash arena IF that arena sits inside the "
            "application region; the 606's does (0x0E000, 64 KB through the "
            "middle) and assuming it did not reported 74842 bytes of headroom "
            "that were the pattern store")
    limits = [CEILING_STAGING if hosts_staging else CEILING_PLAIN]
    if arena_base is not NO_ARENA and arena_base != NO_ARENA:
        if not 0 < arena_base <= PART_BYTES:
            raise ValueError(f"arena_base 0x{arena_base:X} is outside the part")
        limits.append(arena_base)
    return min(limits)


def app_top(segments, ceiling):
    """Highest address used by a segment that ends at or below `ceiling`.

    `segments` is an iterable of (start, end_inclusive), which is what
    IntelHex.segments() gives after the usual off-by-one care. Segments entirely
    above the ceiling are ignored on purpose: on the 808 that is `.natcode` at
    0x0E000, which is real code in the image but does not compete for the
    application's budget. Returns 0 for an image with nothing below the ceiling.

    A segment that STRADDLES the ceiling is counted, and deliberately: it has
    already overrun, and silently ignoring it would report healthy headroom for
    an image that cannot be flashed.
    """
    tops = [end for start, end in segments if start <= ceiling]
    return max(tops) if tops else 0


def headroom(app_top_addr, hosts_staging, arena_base, ceiling=None):
    """(bytes_free, ceiling, percent_used). Negative bytes_free means over.

    `app_top_addr` is the TOP ADDRESS the application occupies (from `app_top`),
    NOT a byte count. Those differ by one and the difference is not academic:

        free = ceiling - (top + 1)      correct
        free = ceiling - top            counts the top byte itself as free

    A808's 15-line version computes the second and reports **618 bytes free at
    top 0x0D496 when the true figure is 617**. The app occupies [0, 0x0D496]
    inclusive, which is 54423 bytes; free is [0x0D497, 0x0D6FF], which is 617.

    One byte, and it is reported the optimistic way, in a check whose entire job
    is to stop a build overrunning by a margin that has twice been under 200
    bytes. A808 predicted an off-by-one would be exactly what diverged across
    five copies of this arithmetic, and it was already there in the first copy.
    """
    ceiling = app_ceiling(hosts_staging, arena_base, ceiling)
    used_bytes = app_top_addr + 1
    return ceiling - used_bytes, ceiling, (100.0 * used_bytes / ceiling)


def report(app_top_addr, hosts_staging, arena_base, label="", ceiling=None):
    """One line a human can act on, plus the toolchain's number named as wrong.

    Prints BOTH numbers on purpose. Suppressing the toolchain's figure would
    leave a reader wondering why two tools disagree; naming it as the part-based
    one that cannot answer the question is the honest form.
    """
    free, ceiling, pct = headroom(app_top_addr, hosts_staging, arena_base,
                                  ceiling)
    used = app_top_addr + 1
    which = "staging-window build" if hosts_staging else "no staging window"
    if ceiling == arena_base and arena_base not in (NO_ARENA, None):
        which += ", capped by the flash arena"
    tag = f"{label}: " if label else ""
    lines = [
        f"{tag}flash {used} of {ceiling} usable "
        f"({pct:.1f}%), {free} bytes free  [{which}, ceiling 0x{ceiling:05X}]",
        f"{tag}  the toolchain will say {100.0 * used / MAXIMUM_SIZE:.1f}% of "
        f"{MAXIMUM_SIZE}: that is the board's maximum_size, not the layout, and "
        f"it is not the number to trust",
    ]
    if free < 0:
        lines.append(f"{tag}  OVER THE CEILING by {-free} bytes")
    elif free < 512:
        lines.append(f"{tag}  WARNING: under 512 bytes of real headroom")
    return "\n".join(lines)


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("app_top", type=lambda v: int(v, 0),
                    help="TOP ADDRESS the app occupies (not a byte count); "
                         "0x-prefixed accepted")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--staging", dest="staging", action="store_true",
                   help="this build hosts the three-entry flash service")
    g.add_argument("--no-staging", dest="staging", action="store_false",
                   help="it does not")
    a = ap.add_mutually_exclusive_group(required=True)
    a.add_argument("--arena-base", type=lambda v: int(v, 0), default=None,
                   help="base of this machine's flash arena, IF it sits inside "
                        "the application region (the 606's does: 0x0E000)")
    a.add_argument("--no-arena", dest="arena_base", action="store_const",
                   const=NO_ARENA, help="it has none inside that region")
    ap.add_argument("--ceiling", type=lambda v: int(v, 0), default=None,
                    help="explicit ceiling for a layout this module does not "
                         "model; overrides both flags above")
    ap.add_argument("--label", default="")
    args = ap.parse_args()
    print(report(args.app_top, args.staging, args.arena_base, args.label,
                 args.ceiling))
    free, _, _ = headroom(args.app_top, args.staging, args.arena_base,
                          args.ceiling)
    return 2 if free < 0 else 0


if __name__ == "__main__":
    sys.exit(main())
