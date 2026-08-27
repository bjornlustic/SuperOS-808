# patch_usb_core.py -- pre-script for the USB MIDI envs: make the Teensy AVR
# usb_midi core's identity overridable from build_flags, and give the device
# a SERIAL NUMBER descriptor.
#
# Two idempotent, in-place patches to the installed framework package (safe
# on every build; self-heals after a framework update or on a fresh
# PlatformIO install; defaults leave other projects' builds unchanged):
#
# 1. usb_private.h: wrap VENDOR_ID/PRODUCT_ID/STR_PRODUCT in #ifndef guards
#    so the -D flags in platformio.ini win (the stock core hard-#defines
#    them, and the redefinition makes the header value win otherwise).
#
# 2. usb.c: add an iSerialNumber string descriptor (STR_SERIAL_NUMBER,
#    #ifndef-guarded, default L"SUPEROS"). The stock core ships NO serial,
#    and hosts match serial-less USB devices by port location: on macOS,
#    CoreMIDI re-uses whatever device entity last lived on that port, so a
#    serial-less unit plugged where another device once was inherits that
#    device's name (both directions observed and SysEx-verified on the
#    bench). A serial gives every unit a stable host-side identity and ends
#    the aliasing.
Import("env")
import os
import re

MACROS = ("VENDOR_ID", "PRODUCT_ID", "STR_PRODUCT")

fw = env.PioPlatform().get_package_dir("framework-arduinoteensy") or ""
core = os.path.join(fw, "cores", "usb_midi")
header = os.path.join(core, "usb_private.h")
usbc = os.path.join(core, "usb.c")

if not os.path.isfile(header):
    print("patch_usb_core: %s not found; skipping (USB identity flags may not apply)" % header)
else:
    with open(header, "r") as f:
        text = f.read()
    patched = []
    for macro in MACROS:
        if re.search(r"#ifndef\s+%s\b" % macro, text):
            continue                       # already guarded
        pat = re.compile(r"^([ \t]*#define[ \t]+%s\b[^\n]*)$" % macro, re.M)
        m = pat.search(text)
        if not m:
            print("patch_usb_core: no #define %s in usb_private.h; skipping it" % macro)
            continue
        text = "%s#ifndef %s\n%s\n#endif%s" % (
            text[: m.start()], macro, m.group(1), text[m.end():])
        patched.append(macro)
    if patched:
        with open(header, "w") as f:
            f.write(text)
        print("patch_usb_core: guarded %s in %s" % (", ".join(patched), header))

if not os.path.isfile(usbc):
    print("patch_usb_core: %s not found; skipping serial-number patch" % usbc)
else:
    with open(usbc, "r") as f:
        text = f.read()
    if "STR_SERIAL_NUMBER" in text:
        pass                               # already patched
    else:
        ok = True
        # 1. device descriptor: iSerialNumber 0 -> 3
        pat = re.compile(r"^([ \t]*)0,([ \t]*//[ \t]*iSerialNumber)", re.M)
        text, n = pat.subn(r"\g<1>3,\g<2>", text, count=1)
        ok = ok and n == 1
        # 2. serial string descriptor after the product string (string1)
        anchor = "// This table defines which descriptor data is sent"
        block = (
            "#ifndef STR_SERIAL_NUMBER\n"
            "#define STR_SERIAL_NUMBER\tL\"SUPEROS\"\n"
            "#endif\n"
            "static const struct usb_string_descriptor_struct PROGMEM string3 = {\n"
            "\tsizeof(STR_SERIAL_NUMBER),\n"
            "\t3,\n"
            "\tSTR_SERIAL_NUMBER\n"
            "};\n\n"
        )
        i = text.find(anchor)
        ok = ok and i >= 0
        if ok:
            text = text[:i] + block + text[i:]
        # 3. serve wValue 0x0303 from the descriptor table
        pat = re.compile(
            r"^([ \t]*\{0x0301, 0x0409, \(const uint8_t \*\)&string1, sizeof\(STR_PRODUCT\)\},)$",
            re.M)
        text, n = pat.subn(
            r"\g<1>\n\t{0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL_NUMBER)},",
            text, count=1)
        ok = ok and n == 1
        if ok:
            with open(usbc, "w") as f:
                f.write(text)
            print("patch_usb_core: added STR_SERIAL_NUMBER descriptor to usb.c")
        else:
            print("patch_usb_core: usb.c layout unexpected; serial-number patch NOT applied")
