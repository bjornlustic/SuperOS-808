#!/usr/bin/env python3
"""Replace the SuperOS-808's resident bootloader over USB-C or DIN.

Unlike an app update, this talks to the RUNNING FIRMWARE, not to the bootloader:
the bootloader cannot rewrite itself, so the app stages the new image in
application-region flash and then calls the boot-section SPM service to copy it
across (see src/boot_update.h). Do NOT put the machine in bootloader mode first.

Every staged page is acknowledged, and this script stops at the first bad ack,
so a bad upload never reaches the commit. The commit is power-fail safe: it only
rewrites the bootloader's body, never the permanent page holding its reset stub,
and an arm flag in flash makes the stub finish an interrupted copy at the next
power-up. Pulling the plug mid-commit costs a reboot, not the machine.

Usage:
    python3 tools/send_boot_update.py                        # list ports
    python3 tools/send_boot_update.py bootloader-update.syx -p "SuperOS-808"
    python3 tools/send_boot_update.py bootloader-update.syx -p X --stage-only
"""
import argparse
import sys
import time

import mido

STATUS = {
    0: "ok",
    1: "no flash service, or a board still carrying the older single-entry "
       "service (reinstall service-install.syx from bootloader mode first)",
    2: "bad page index",
    3: "page did not unpack to 256 bytes",
    4: "page checksum mismatch",
    5: "staging write refused by the flash service",
    6: "staged image CRC does not match the one in the .syx",
    7: "staging window is blank",
    8: "commit failed",
}


def split_messages(raw):
    msgs, i = [], 0
    while i < len(raw):
        if raw[i] != 0xF0:
            i += 1
            continue
        j = raw.index(0xF7, i)
        msgs.append(raw[i:j + 1])
        i = j + 1
    return msgs


def wait_status(port_in, timeout=4.0):
    """Return (status, staged_crc) from the next 0x73 reply, or None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        for m in port_in.iter_pending():
            if m.type != "sysex":
                continue
            d = list(m.data)
            if len(d) >= 6 and d[0] == 0x7D and d[1] == 0x73:
                crc = d[3] | (d[4] << 7) | (d[5] << 14)
                return d[2], crc
        time.sleep(0.005)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?", help="bootloader-update.syx")
    ap.add_argument("-p", "--port", help="MIDI port name (substring match)")
    ap.add_argument("--stage-only", action="store_true",
                    help="upload and verify, but stop before the commit")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation")
    args = ap.parse_args()

    outs, ins = mido.get_output_names(), mido.get_input_names()
    if not args.file:
        print("MIDI ports:")
        for n in outs:
            print("  ", n)
        print("\nRe-run with bootloader-update.syx (and -p \"Port\").")
        return

    def pick(names, what):
        cand = [n for n in names if args.port.lower() in n.lower()] if args.port else names
        if not cand:
            sys.exit(f"No {what} port matching {args.port!r}. Available: {names}")
        if len(cand) > 1:
            sys.exit(f"Multiple {what} ports; narrow with -p:\n  " + "\n  ".join(cand))
        return cand[0]

    out_name, in_name = pick(outs, "output"), pick(ins, "input")
    msgs = split_messages(open(args.file, "rb").read())
    stage = [m for m in msgs if len(m) > 2 and m[2] == 0x70]
    commit = [m for m in msgs if len(m) > 2 and m[2] == 0x71]
    if not stage or not commit:
        sys.exit("Not a bootloader-update .syx (expected 0x70 pages and a 0x71 commit).")

    print(f"out: {out_name}\nin:  {in_name}\n{len(stage)} pages to stage")

    with mido.open_input(in_name) as pin, mido.open_output(out_name) as pout:
        for _ in pin.iter_pending():
            pass

        pout.send(mido.Message("sysex", data=[0x7D, 0x72]))
        st = wait_status(pin)
        if st is None:
            sys.exit("No reply to the status request. Is the app running (not the "
                     "bootloader), and is this the right port?")
        if st[0] != 0:
            sys.exit(f"Device not ready: {STATUS.get(st[0], st[0])}")

        for k, m in enumerate(stage, 1):
            pout.send(mido.Message("sysex", data=list(m[1:-1])))
            st = wait_status(pin)
            if st is None:
                sys.exit(f"\nPage {k}: no acknowledgement. Nothing live was touched.")
            if st[0] != 0:
                sys.exit(f"\nPage {k}: {STATUS.get(st[0], st[0])}. Nothing live was touched.")
            print(f"  staged {k}/{len(stage)}", end="\r", flush=True)
        print(f"  staged {len(stage)}/{len(stage)}")

        if args.stage_only:
            print("--stage-only: image is staged and acknowledged; not committing.")
            return

        if not args.yes:
            print("\nThe commit rewrites the bootloader's body from the image just staged\n"
                  "and verified. An interrupted commit is resumed automatically at the next\n"
                  "power-up, so this is recoverable, but the machine must be allowed to boot\n"
                  "once afterwards before it is trusted.")
            if input("Type 'commit' to proceed: ").strip() != "commit":
                print("Aborted. The staged image is still there; re-run to resume.")
                return

        pout.send(mido.Message("sysex", data=list(commit[0][1:-1])))
        st = wait_status(pin, timeout=10.0)
        if st is None:
            sys.exit("No reply to the commit. Check whether the machine still enumerates "
                     "before power-cycling it.")
        if st[0] != 0:
            sys.exit(f"Commit refused: {STATUS.get(st[0], st[0])} (nothing was written).")
        print("Committed. New bootloader is live; SysEx 0x4A to enter it.")


if __name__ == "__main__":
    main()
