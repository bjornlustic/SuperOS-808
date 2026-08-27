"""Which `git status -s` entries can actually reach a built binary.

Why this exists: `makesyx.py` marked an image `...dirty` whenever `git status`
said ANYTHING, including `coord/progress/A808.jsonl`, which is WEB's log and is
rewritten under a build every few minutes. `gitprov.py` in SuperOS-USBC already
knows better and treats `coord/` as cosmetic, but it votes on the FILENAME as
well as the tree, so the crude verdict baked into the name won and the image was
refused for a file that cannot reach a binary. Two tools, one question, two
answers, and the wrong one decided.

The rule here is deliberately the same BLOCKLIST as gitprov.py's `_NON_BUILD`,
not an allowlist of source directories: a new source tree must fail closed. Keep
the two in step; gitprov.py is the source of truth for the pattern.
"""
import re

_NON_BUILD = re.compile(
    r"^(coord/|docs/|artifacts/|backup/|README|LICENSE)"
    r"|\.(md|wav|log|txt|json|tsv|jsonl)$"
)


def split_dirt(status):
    """(build_affecting, cosmetic) path lists from `git status -s` output."""
    build, cosmetic = [], []
    for line in status.splitlines():
        if not line.strip():
            continue
        # "XY path", and "XY old -> new" for renames: the destination counts.
        path = line[3:].split(" -> ")[-1].strip().strip('"')
        (cosmetic if _NON_BUILD.search(path) else build).append(path)
    return build, cosmetic


def is_dirty(status):
    """True if anything in `status` could change the binary."""
    return bool(split_dirt(status)[0])


CASES = [
    # (status text, expect_dirty, why)
    (" M coord/progress/A808.jsonl", False, "WEB's log, rewritten under builds"),
    (" M coord/inbox/A808.md", False, "peer mail"),
    (" M docs/gates/G8_ACCOUNTING_BASELINE.md", False, "gate evidence"),
    (" M src/main.cpp", True, "firmware source"),
    (" M platformio.ini", True, "build configuration"),
    ("?? src/new_file.h", True, "untracked source still counts if listed"),
    (" M tools/makesyx.py", True, "the packer itself shapes the image"),
    (" M src/main.cpp\n M coord/inbox/A808.md", True, "one real file is enough"),
    ("R  docs/a.md -> src/a.h", True, "rename destination is what matters"),
    ("", False, "clean tree"),
    (" M src/engine.h.md", False, "blocklist is by suffix, documented as such"),
]

if __name__ == "__main__":
    bad = 0
    for status, expect, why in CASES:
        got = is_dirty(status)
        ok = got == expect
        bad += not ok
        print("%-4s %-40r expect=%-5s got=%-5s  %s"
              % ("ok" if ok else "FAIL", status, expect, got, why))
    print("%d case(s) failed" % bad)
    raise SystemExit(1 if bad else 0)
