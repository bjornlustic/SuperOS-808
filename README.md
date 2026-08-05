# SuperOS-808

Open firmware for the Roland TR-808, running on an AT90USB1286 CPU replacement board in the
µPD650C socket (Teensy++ 2.0 class).

It adds an extended sequencer layer, MIDI in/out with clock sync, a SysEx web editor and
firmware updates over MIDI, while leaving the machine's own operating model intact. A
`combined` build also carries an emulator of the stock µPD650C-085, so one board can boot
either the original behaviour or the extended one.

**Status:** runs on hardware. Bootloader, firmware switching, the emulator (including MIDI
clock sync) and the SuperOS layer have all been exercised on a real machine. The panel rotary
decode is deliberately calibrated per-machine rather than trusted from the schematic — see
[Bring-up](#bring-up).

## The 808's model, kept

Taken from the owner's manual:

- **32 patterns**: 16 STEP buttons × two variation modes (A/B). Buttons 1-12 are BASIC
  RHYTHM, 13-16 are INTRO/FILL IN.
- **A measure is 1st PART + 2nd PART**, up to 16 steps each, concatenated at playback with
  independent step counts. That is how 20-step and other odd measures are written. A and B
  share the step counts and the pre-scale.
- **PRE SCALE** sets steps per beat: 1 = 3, 2 = 6, 3 = 4, 4 = 8.
- **CLEAR** commits the PRE SCALE dial; **CLEAR + a step button** sets that part's step
  count, per the panel's `PATTERN CLEAR / TRACK CLEAR / STEP NUMBER` legend.
- **TAP** inserts intros and fills in the play modes, and writes steps in real time in the
  write modes.
- **AUTO FILL IN** drops a fill every 2/4/8/12/16 measures; an intro is not counted.
- **COMPOSE** records a rhythm track live. Tracks store the STEP-button *slot*, not the
  variation, so the variation switch applies at playback — 12 tracks × 64 measures.

Nothing SuperOS adds moves a stock control. START/STOP, TAP and FILL IN keep their factory
behaviour in every mode.

## What SuperOS adds

The 808 has exactly one free momentary button, so the whole extended layer hangs off CLEAR
gestures: tap, hold, CLEAR + step, double-tap for the config menu, long-press to confirm.
Holding CLEAR shows a tool map; a step press latches one of 16 tools.

- Patterns extended from 32 to 64 steps (four sections instead of two). Sections 3 and 4
  default to zero length, so stock-written patterns behave identically.
- Per-step **probability** and **ratchet**, per-instrument **loop length** and **polymeter**,
  **swing**, play **direction**, live **reslice**, **arp**, pattern **generate**,
  **copy/paste**, **transform**, **mute**.
- MIDI in/out: drum notes (velocity ≥ 100 asserts accent), program change for pattern
  selection, clock sync as slave or master, soft thru.
- A web editor over SysEx (`tools/web-editor/index.html`) for pattern editing, panel
  calibration, and firmware/program-image upload.

Patterns, tracks, settings and the panel map persist in a wear-leveled internal-flash block
store, written through an SPM service that lives in the top of the boot section.

## Emulator

`combined` builds carry both firmwares in one image; a byte in internal EEPROM picks one per
boot. The emulator runs the machine's own µPD650C-085 program against the real hardware — the
emulated CPU's port latches are mirrored straight onto the socket pins, so the 808's gates
fire voices exactly as they did with the original chip. The four µPD444C RAMs are emulated as
a 4096-nibble image kept in internal EEPROM.

**No program image is distributed with this project.** The µPD650C-085 program is copyrighted;
supply your own, read out from a chip you are legally entitled to read. Because the pattern
image plus an uploadable program image would be 4 KB against a 4096-byte EEPROM, the program
image must be compiled in:

```bash
python3 tools/d650/make_image_embed.py <your-transfer.syx>
```

The generated header is gitignored and must not be committed or redistributed.

Switch into the emulator from the config menu (double-tap CLEAR, then step 9), and back out
of it by **double-pressing CLEAR** — the emulated machine has no menu of its own, so that
gesture belongs to the bridge, which keeps the key from reaching the emulated CPU until it
knows whether it was one press or two. Either direction is also SysEx `F0 7D 4D <0|1> F7`,
and holding **STEP 1 + STEP 16** at power-on forces SuperOS regardless of the stored
selection.

## Building

Needs PlatformIO and a git checkout (the build stamps the commit into the filename).

```bash
pio run -e bootloader -e flash-service -e combined
```

| env | what it is |
|---|---|
| `combined` | SuperOS + emulator, program image compiled in |
| `app-flash` | SuperOS only |
| `bootloader` | MIDI SysEx bootloader, resident at 0x1F000; flash once over ISP |
| `flash-service` | SPM flash-write service at 0x1FE00; install once over SysEx |
| `fuses` | `lfuse=0xDF hfuse=0xD2`, blank chips only |

Outputs land in the repo root: `SuperOS-808_v<ver>_<env>_<rev>.syx` to flash over MIDI, and
`-full.hex` (app + bootloader + service) for ISP.

Prebuilt SuperOS builds are on the [releases
page](https://github.com/bjornlustic/SuperOS-808/releases); every push also builds them as
workflow artifacts. **Emulator builds are never published** — they compile in the
copyrighted program image, so `combined` is a local build only. See
[CHANGELOG.md](CHANGELOG.md) for what changed per release.

USB is deliberately off. A live USB engine at 16 MHz measurably corrupts the high-impedance
panel matrix whether or not a host is attached, so all MIDI runs over DIN.

## Bring-up

The CPU cannot be bench-probed once installed — after the first ISP flash the only access is
MIDI SysEx — so the risky part of the port lives in data that can be fixed from the web editor
rather than code that needs a reflash.

1. **Flash the bootloader over ISP** (`-full.hex`, or `pio run -e bootloader` + avrdude). Set
   fuses on a blank chip with `pio run -e fuses`.
2. **Power on holding STEP 1** to stay in the bootloader: step LEDs 1-4 blink twice, then
   step 1 stays lit.
3. Open the web editor in Chrome or Edge, connect, and send the `.syx` from the
   *Firmware & program image* tab.
4. Power-cycle. Three quick CC#119 pulses on MIDI OUT mean the firmware is alive.
5. **Run the panel calibration** (*Panel calibration* tab). Not optional on a first flash.

### Why calibration is not optional

The panel rotaries reach the CPU through a diode encoder, not one cell per switch, and its
truth table has to be read off a 1981 scan. Trusted, because they trace unambiguously: the
port map and socket pinout, the instrument-data bit assignment, COMMON TRIG on PI2, accent as
a shared pulse amplitude, the step-switch and step-LED matrix, and the PRE SCALE and BASIC
VARIATION codes.

Not trusted, and therefore calibrated:

- **MODE**: as drawn, COMPOSE and PLAY share an encoder node and come out with the same code,
  which cannot be right for a 6-position selector.
- **INSTRUMENT/TRACK** (12 positions) and **AUTO FILL IN** (6): too many bridged diodes to
  transcribe reliably. The defaults are plain binary.
- **Status group bit order** (TEMPO CLOCK / START-STOP / TAP / FILL IN): the manual names the
  four signals but not which PA bit each sits on.
- **Indicator LED bus bits** (VARIATION A/B, 1ST/2ND PART). Cosmetic — a wrong guess lights
  the wrong lamp and nothing else, because every trigger pulse rewrites all 12 data lines.

Calibration itself only needs the step keys, CLEAR, START/STOP and the tempo clock, all of
which *are* unambiguous. That is what lets an uncalibrated device bootstrap itself.

## Layout

```
src/
  pins.h            socket <-> AVR map
  hw.h              matrix scan + step-LED multiplex
  controls.h        panel decode + the calibratable PanelMap
  pattern.h         data model (sections, slots, variations, tracks)
  engine.h          sequencer, variation/fill logic, trigger bus
  main.cpp          panel handling, CLEAR gestures, tools, LED frames
  midi.cpp          SysEx editor link, clock, notes
  flash_persist.h   block store glue (internal flash arena)
  bootloader/       MIDI SysEx bootloader
  flash_service/    SPM flash-write service
  combined.cpp      firmware selector for emulator builds
  d650/             the µPD650C emulator: host machine, image store, AVR bridge
tools/
  web-editor/       the editor and the calibration wizard
  makesyx.py        packs the build into a flashable .syx
```

The SysEx command set is documented at the top of `src/midi_api.h`.

## Sources

- *Roland TR-808 Service Notes*, First Edition, Jun 15 1981
- *TR-808 Owner's Manual*
- The ucom4 CPU core is a port of MAME's `src/devices/cpu/ucom4` (BSD-3-Clause, copyright hap)
