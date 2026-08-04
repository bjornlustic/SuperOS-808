#!/usr/bin/env python3
"""Build the SuperOS-808 PanelMap from captured codes and push it over SysEx.

The panel decode is data, not code (docs/HARDWARE_MAP.md §7): the rotaries reach
the CPU through a diode encoder whose truth table could not be read reliably off
the 1981 scan. The guided calibration walk (SysEx 0x51) records what each detent
actually reads into docs/panel_capture.json; this turns that into the 43-byte
PanelMap the firmware stores and applies. With no capture file present it falls
back to the measured defaults below.

Layout MUST track `struct PanelMap` in src/controls.h. The firmware
static_asserts sizeof(PanelMap) == 43 so a field added on one side without the
other is caught at compile time rather than silently writing garbage into the
decode.

  python3 tools/panel_write.py            # show the table that would be sent
  python3 tools/panel_write.py --send     # send it, then read it back to verify
"""
import json
import sys
import time
from pathlib import Path

import mido

ROOT = Path(__file__).resolve().parents[1]
CAP = ROOT / "docs" / "panel_capture.json"

# ---- struct PanelMap byte offsets (src/controls.h) -------------------------
LAYOUT = [
    ("magic", 1), ("mode_code", 6), ("mode_mask", 1), ("clear_bit", 1),
    ("pre_code", 4), ("pre_mask", 1), ("var_code", 3), ("var_mask", 1),
    ("inst_code", 12), ("inst_mask", 1), ("fill_code", 6), ("fill_mask", 1),
    ("ifvar_bit", 1), ("run_bit", 1), ("tap_bit", 1), ("fillin_bit", 1),
    ("clock_bit", 1),
]
OFF, _o = {}, 0
for _k, _n in LAYOUT:
    OFF[_k] = _o
    _o += _n
NBYTES = _o                                    # 43
MAGIC = 0x88

MODES = ["PATTERN CLEAR", "1st PART", "2nd PART", "MANUAL PLAY", "PLAY", "COMPOSE"]
INSTS = ["AC", "BD", "SD", "LT", "MT", "HT", "RS", "CP", "CB", "CY", "OH", "CH"]
FILLS = ["MANUAL", "16", "12", "8", "4", "2"]
VARS = ["A", "AB", "B"]

# ---- measured on the machine ------------------------------------------------
# MODE and PRE SCALE are confirmed: MODE by a dedicated end-to-end sweep, PRE
# SCALE by a full 4-of-4 capture that also matched the schematic trace exactly.
#
# PLAY and COMPOSE genuinely share code 3 — the SW1a diode encoder bridges them,
# confirmed on hardware, so no table can separate them. main.cpp resolves it with
# a CLEAR long-press toggle until the pin-30 / SW1b mod lands.
MEASURED = {
    "mode_code": {"PATTERN CLEAR": 5, "1st PART": 6, "2nd PART": 7,
                  "MANUAL PLAY": 1, "PLAY": 3, "COMPOSE": 3},
    "mode_mask": 0x07,
    # INSTRUMENT: a 12-position cyclic counter that runs DOWN as the knob runs
    # up and wraps, so code = (TOP - position) mod 12 and every code 0..11 is
    # used once — a mis-read shows up as a collision, or as a UNIFORM rotation of
    # the dial. TOP = 11 is measured: the raw 0x3B probe reads code 11 with the
    # knob on the AC detent (2026-08-04). Keep this in step with INST_CODE_TOP in
    # src/controls.h, and read the code off the probe rather than off what a
    # detent sounds like — every by-ear attempt so far came out rotated, and a
    # "rotated dial" is usually a stale binary rather than a wrong table.
    "inst_code": {n: (11 + 12 - i) % 12 for i, n in enumerate(
        ["AC", "BD", "SD", "LT", "MT", "HT", "RS", "CP", "CB", "CY", "OH", "CH"])},
    "pre_code": {"1": 3, "2": 1, "3": 2, "4": 0},
    "pre_mask": 0x03,
    # status group, confirmed by watching which bit moved for each control
    "run_bit": 0, "tap_bit": 1, "fillin_bit": 2, "clock_bit": 3,
    # defaults until measured
    "clear_bit": 3, "ifvar_bit": 3,
    "var_mask": 0x0C, "inst_mask": 0x0F, "fill_mask": 0x07,
}


def from_capture(key, labels, fallback):
    """Take an ordered code list out of the capture file, if it is complete."""
    doc = json.loads(CAP.read_text()) if CAP.exists() else {}
    ent = doc.get(key)
    if ent and len(ent.get("order", [])) == len(labels):
        return dict(zip(labels, ent["order"])), True
    return fallback, False


def build():
    m = bytearray(NBYTES)
    m[OFF["magic"]] = MAGIC
    notes = []

    for i, name in enumerate(MODES):
        m[OFF["mode_code"] + i] = MEASURED["mode_code"][name]
    m[OFF["mode_mask"]] = MEASURED["mode_mask"]

    for i, name in enumerate(["1", "2", "3", "4"]):
        m[OFF["pre_code"] + i] = MEASURED["pre_code"][name]
    m[OFF["pre_mask"]] = MEASURED["pre_mask"]

    var, ok = from_capture("BASIC VARIATION", VARS, {"A": 0, "AB": 8, "B": 4})
    notes.append(f"BASIC VARIATION {'measured' if ok else 'SCHEMATIC DEFAULT'}")
    for i, n in enumerate(VARS):
        m[OFF["var_code"] + i] = var[n]
    m[OFF["var_mask"]] = MEASURED["var_mask"]

    inst, ok = from_capture("INSTRUMENT", INSTS, MEASURED["inst_code"])
    notes.append(f"INSTRUMENT {'from capture' if ok else 'measured by ear (see MEASURED)'}")
    for i, n in enumerate(INSTS):
        m[OFF["inst_code"] + i] = inst[n]
    m[OFF["inst_mask"]] = MEASURED["inst_mask"]

    fill, ok = from_capture("AUTO FILL IN", FILLS, {n: i for i, n in enumerate(FILLS)})
    notes.append(f"AUTO FILL IN {'measured' if ok else 'BINARY DEFAULT'}")
    for i, n in enumerate(FILLS):
        m[OFF["fill_code"] + i] = fill[n]
    m[OFF["fill_mask"]] = MEASURED["fill_mask"]

    for k in ("clear_bit", "ifvar_bit", "run_bit", "tap_bit", "fillin_bit", "clock_bit"):
        m[OFF[k]] = MEASURED[k]
    return bytes(m), notes, {"var": var, "inst": inst, "fill": fill}


def show(m, notes, tabs):
    print(f"PanelMap, {len(m)} bytes (struct is 43)\n")
    print("  MODE   " + "  ".join(f"{n}={MEASURED['mode_code'][n]:04b}" for n in MODES))
    print("  PRE    " + "  ".join(f"{n}={MEASURED['pre_code'][n]:04b}" for n in ["1", "2", "3", "4"]))
    print("  VAR    " + "  ".join(f"{n}={tabs['var'][n]:04b}" for n in VARS))
    print("  INST   " + " ".join(f"{n}={tabs['inst'][n]:04b}" for n in INSTS))
    print("  FILL   " + "  ".join(f"{n}={tabs['fill'][n]:04b}" for n in FILLS))
    print(f"  BITS   clear={m[OFF['clear_bit']]} ifvar={m[OFF['ifvar_bit']]} "
          f"run={m[OFF['run_bit']]} tap={m[OFF['tap_bit']]} "
          f"fill={m[OFF['fillin_bit']]} clock={m[OFF['clock_bit']]}")
    print("\n  " + "\n  ".join(notes))
    dup = [n for n in MODES if list(MEASURED["mode_code"].values()).count(
        MEASURED["mode_code"][n]) > 1]
    if dup:
        print(f"\n  NOTE: {' and '.join(dup)} share a code — the SW1a encoder bridges"
              f"\n        them. Resolved in firmware by a CLEAR long-press toggle"
              f"\n        until the pin-30 / SW1b mod is wired.")


def pack7(src):
    out = []
    for i in range(0, len(src), 7):
        run = src[i:i + 7]
        msb = 0
        for b, v in enumerate(run):
            if v & 0x80:
                msb |= 1 << b
        out.append(msb)
        out += [v & 0x7F for v in run]
    return out


def send(m):
    o = [n for n in mido.get_output_names() if "ux16" in n.lower()]
    i = [n for n in mido.get_input_names() if "ux16" in n.lower()]
    if not o or not i:
        sys.exit("no UX16 MIDI port")
    out, inp = mido.open_output(o[0]), mido.open_input(i[0])
    x = 0
    for v in m:
        x ^= v
    out.send(mido.Message("sysex", data=[0x7D, 0x50, x & 0x7F, (x >> 7) & 1] + pack7(m)))
    print("\nsent 0x50; reading it back ...")
    time.sleep(0.6)
    out.send(mido.Message("sysex", data=[0x7D, 0x33]))
    time.sleep(0.8)
    got = None
    for msg in inp.iter_pending():
        if msg.type == "sysex":
            d = list(msg.data)
            if len(d) > 4 and d[0] == 0x7D and d[1] == 0x53:
                raw, k = [], 4
                while k < len(d):
                    msb = d[k]; k += 1
                    for b in range(7):
                        if k < len(d):
                            raw.append(d[k] | (((msb >> b) & 1) << 7)); k += 1
                got = bytes(raw[:NBYTES])
    out.close(); inp.close()
    if got is None:
        print("  no read-back — is the 808 in SuperOS (not the emulator)?")
    elif got == m:
        print("  VERIFIED — the 808 stored the table byte-for-byte")
    else:
        print("  MISMATCH:")
        for k, (a, b) in enumerate(zip(m, got)):
            if a != b:
                print(f"    byte {k}: sent {a:#04x} got {b:#04x}")


if __name__ == "__main__":
    mm, nn, tt = build()
    show(mm, nn, tt)
    if "--send" in sys.argv:
        send(mm)
    else:
        print("\n(dry run — pass --send to write it to the 808)")
