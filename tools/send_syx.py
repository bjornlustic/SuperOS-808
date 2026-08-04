#!/usr/bin/env python3
"""Throttled SysEx uploader for the SuperOS-808 on-board bootloader.

The bootloader (src/bootloader/bootload.c) has NO UART RX buffer and stops
reading MIDI while it erases+writes each flash page (~3-4 ms of SPM). Any bytes
that arrive during that window are lost, the next page's checksum then fails,
and the bootloader blinks step LEDs 1-4 together forever — so the app
never starts. The cure is simply to wait after every SysEx message so the page
write finishes before the next message begins.

Usage:
    python3 tools/send_syx.py                       # list MIDI output ports
    python3 tools/send_syx.py <file.syx>            # send to the only/first port
    python3 tools/send_syx.py <file.syx> -p "Port"  # send to a named port
    python3 tools/send_syx.py <file.syx> -d 0.15    # override per-message delay (s)

Put the 808 in bootloader mode first (hold STEP 1 at power-on: steps 1-4 blink
twice, then step 1 stays lit), with the sending interface's MIDI OUT into the
808's MIDI IN.
"""
import argparse
import sys
import time

import mido

BAUD = 31250          # MIDI serial rate
FLASH_MS = 0.012      # generous page erase+write allowance per message


def split_messages(raw: bytes):
    msgs = []
    i = 0
    while i < len(raw):
        if raw[i] != 0xF0:
            i += 1
            continue
        j = raw.index(0xF7, i)
        msgs.append(raw[i:j + 1])
        i = j + 1
    return msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?", help="path to the .syx app image")
    ap.add_argument("-p", "--port", help="MIDI output port name (substring match)")
    ap.add_argument("-d", "--delay", type=float, default=None,
                    help="fixed per-message delay in seconds (default: auto, "
                         "sized to message length + flash time)")
    args = ap.parse_args()

    outs = mido.get_output_names()
    if not args.file:
        print("MIDI output ports:")
        for n in outs:
            print("  ", n)
        print("\nRe-run with a .syx file (and -p \"Port\" if more than one).")
        return

    if not outs:
        sys.exit("No MIDI output ports found.")
    if args.port:
        match = [n for n in outs if args.port.lower() in n.lower()]
        if not match:
            sys.exit(f"No port matching {args.port!r}. Available: {outs}")
        port_name = match[0]
    else:
        if len(outs) > 1:
            sys.exit(f"Multiple ports; pick one with -p:\n  " + "\n  ".join(outs))
        port_name = outs[0]

    raw = open(args.file, "rb").read()
    msgs = split_messages(raw)
    if not msgs:
        sys.exit("No SysEx messages found in file.")
    print(f"Sending {len(msgs)} messages to {port_name!r} ...")

    with mido.open_output(port_name) as port:
        for k, m in enumerate(msgs):
            port.send(mido.Message("sysex", data=m[1:-1]))  # mido adds F0/F7
            if args.delay is not None:
                time.sleep(args.delay)
            else:
                # wait out this message's transmission + the flash write
                time.sleep(len(m) * 10 / BAUD + FLASH_MS)
            if (k + 1) % 10 == 0 or k + 1 == len(msgs):
                print(f"  {k + 1}/{len(msgs)}")
    print("Done. The bootloader runs the app on the final EXECUTE message.")
    print("If it still doesn't boot, power-cycle, and try a larger delay (-d 0.2).")


if __name__ == "__main__":
    main()
