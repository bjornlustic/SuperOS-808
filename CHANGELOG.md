# Changelog

Notable changes per release. Dates are release dates.

## v0.3 — 2026-08-26

The baseline release: every feature below has been exercised on a real machine,
most of it by an automated panel-injection harness that presses the keys and
reads the state back over SysEx.

### Added

**Instant tool map, and in MANUAL PLAY too.** The map is up the moment CLEAR
goes down — no hold threshold — in both write modes and in MANUAL PLAY, where
CLEAR previously did nothing. A step press while CLEAR is down latches that
tool; the latched tool blinks in the map. The stock CLEAR+step chord is gone
with the wait: its function (step number) lives in the LENGTH tool.

**Per-voice direction and per-voice pre-scale, saved with the pattern.** The
DIRECTION tool gains the same TAP-toggled voice phase as LENGTH and POLYMETER:
set one voice running backward, random or ping-pong against the pattern. With
LENGTH latched, holding CLEAR and turning Pre-Scale in the voice phase gives
one voice its own step rate, so it plays between the master's steps. Both are
stored per pattern, per voice; pattern direction is saved with the pattern too.

**Writing a part that does not exist creates it.** A fresh pattern has only its
first sixteen steps; the first press in 2ND PART — or in sections 3 and 4 via
the SECTION tool — now extends the pattern there and lands the press, instead
of presenting a dead panel.

**The chain is drawn.** Chained slots and the queued pattern show on the step
LEDs; chains cap at four slots.

**D650C mode: instant step entry.** The 1981 program services step entry only
once per sequencer step and drops fast presses — measured against the original
program image, continuous tapping registers about one press per one and a half
steps, worse at slow tempos. The bridge now writes the entry directly into the
emulated pattern memory, addressed exactly as the program would (verified
byte-identical across every instrument, key and bank), so presses land the
instant the key goes down, as fast as you can tap, and entry works with the
transport stopped as well.

**D650C mode: the config menu opens in place, and the machine keeps playing.**
CLEAR held + TAP brings up the menu inside the emulator without a reboot; only
the panel is borrowed, so the sequencer keeps time. Menu step 9 blinks on the
D650C side to say which firmware you are standing in, and pressing it returns
to superOS.

### Changed

**The tool map is renumbered, contiguous 1–14.** PRE-SCALE folded into LENGTH,
the ACCENT tool retired (the 808's accent is one shared level per step — the
tool implied a per-instrument control the hardware does not have), SWING moved
to slot 2, slot 15 is dark and unlatchable, and step 16 is the config menu.

**The config menu moved to the tool map's step 16.** The CLEAR double-tap is
retired, and CLEAR held + TAP no longer opens the menu on the superOS side —
that chord is the D650C-mode entry only. A CLEAR tap is the only way out; TAP
is "back" inside the menu and never exits. The menu no longer stops the
sequencer on either firmware.

**Chase delete moved to PATTERN CLEAR mode**, the mode whose whole job is
erasing: hold CLEAR while running and the selected voice's hits are erased
under the playhead, from the moment the button goes down. Stopped, a CLEAR tap
erases the selected slot, as always.

**Steps a voice cannot play are unpressable.** Past a voice's own loop length,
steps are dark in the write display and refuse the pick press in RATCHET and
PROBABILITY, so hits can no longer accumulate where nothing will sound.

**The section chosen with the SECTION tool stays on display** after the tool is
released, until the Mode Dial moves — sections 3 and 4 are actually editable
from the panel now.

**Master polymeter applies while playing.** The change lands in RAM immediately
and the store write waits for the transport to stop, so nothing stalls under
the clock.

**The pattern record grew from 228 to 241 bytes** (direction and the per-voice
overrides travel with the pattern). Old records load as a prefix; the web
editor speaks the new layout.

**The program image moved out of the app region** into its own flash section,
which also un-crowded every build.

### Fixed

**D650C step entry debounced across scan periods.** The emulator bridge took
its three debounce samples one column-revisit apart (7.6 ms each), which
filters nothing and cost 15 ms of pure latency; the samples now land inside one
column dwell and a press is delivered in about 2 ms, matching superOS.

**The VARIATION A lamp flickered while superOS played.** The trigger window was
holding the lamp line low longer than the original program does; measured on
the machine against the 1981 program on the same panel, the remaining duty
difference is inside what the lamp's RC filter absorbs.

**2ND PART could not be written at all** on a pattern without a second part —
every press was silently blocked. See "writing a part that does not exist
creates it" above.


**Probability could not be set to "always", and step 16 corrupted the slot.** The panel
tool passed the raw key number into `prob_set`, so step 16 wrote a chance of 16 — which
does not fit the nibble it shares with the instrument index, and read back as the *next*
instrument with a chance of zero. The press looked dead, did nothing audible, and consumed
one of the 40 probability slots. Step 16 now means "always" (it frees the slot), and
`prob_set` clamps on both machines.

**The two-phase tools showed no value.** PROBABILITY and RATCHET drew a bare list of
affected steps and, once a step was selected, nothing but a blinking cursor — so the only
way to find out what a step was set to was to press something and listen. Both phases now
follow the 606: the pick phase draws the voice's pattern with the affected steps blinking
on top, and the set phase draws the current value as a bar.

**RATCHET ignored most of its own row.** The set phase read only steps 1-4; the rest of the
row did nothing, which is indistinguishable from a missed press. Anything above step 3 now
also means 4x, as on the 606.

**RATCHET had no key for "one hit".** The set phase mapped step 1 to a 2x retrigger, so the
lowest value a key could write was a double: trying to clear a ratcheted step by pressing
step 1 re-armed it and the step kept double-firing. The key number is now the total hit
count: step 1 = a single hit (clears the ratchet), steps 2/3/4 = 2x/3x/4x, higher keys
clamp to 4x, and the value bar lights one LED per hit.

**PATTERN CLEAR was unusable: strobing LEDs, swallowed presses.** The mode's LED frame read
all 16 patterns through the CRC-checked block store on every ~1 ms loop pass, over 20 ms
of page reads and bitwise CRC per pass. The LED multiplex strobed visibly and most button
presses fell between debounce samples, so slot selection and the CLEAR tap rarely
registered. The slot-occupancy display is now served from a cached mask rebuilt only on
mode entry, variation change, after a clear, and once a second while stopped.

**Ratchet repeats were rushed at several PRE SCALE and count combinations.** Retriggers
were scheduled on whole 24-PPQN ticks with truncating division, so a 4x ratchet on a
six-tick sixteenth landed its hits on ticks 0/1/2/3: a fast burst and then silence, audibly
out of time. Retriggers are now scheduled in microseconds from the measured tick period,
spaced evenly across the step's real duration, under the internal clock, DIN sync and MIDI
clock alike.

**Config-menu settings did not survive a power cycle.** The menu edited the settings in RAM
and nothing on the panel side ever wrote them to flash; only a save pushed from the web
editor persisted them. Settings changed in the menu are now flushed to flash when the menu
closes (deferred to the next stop if the sequencer is running, since a flash write stalls
the CPU for a few milliseconds).

### Added

- **TAP as the tool layer's second key.** CLEAR is the gateway on this machine, so it
  cannot also mean "wipe" inside a tool the way it does on the 606. TAP now carries those
  actions: wipe a voice's probability or ratchets, or reset one step.
- **POLYMETER shows every voice at once** — one key per voice, lit = free-running against
  the bar — with master all-rows switches on steps 15/16. It previously showed only the
  voice on the INSTRUMENT dial and had no way to reach the master switches. Per-voice loop
  LENGTH moved to the Length tool (TAP + step), where the 606 keeps it.
- **MUTE all / unmute all** on steps 15/16, and **COPY** gained clear-pattern on step 1,
  **TRANSFORM** gained randomize on step 4.
- **Per-voice loop length on TAP in both length-minded tools.** In LENGTH and POLYMETER,
  TAP toggles between two sticky modes: master (solid display; the tool's normal surface)
  and voice length (display blinks at the beat; steps set the selected voice's own loop
  length, absolute across the sections, and the step that already is the length clears
  it). Three earlier shapes died on hardware in one session: TAP + step as a held chord is
  physically impossible (the TAP status line reads as a short pulse, so `held()` drops
  while the button is still down); an auto-exiting phase read as the tool randomly
  alternating between master and voice length; and the global length override's stint on
  POLYMETER's TAP meant the natural polymeter gesture silently armed it. The global
  override lost its panel binding; the engine support remains for a future editor hook.
- **Web editor: probability and ratchet are editable per step.** The editor round-tripped
  both fields but exposed no way to set them; it now has a Steps / Probability / Ratchet
  layer selector, with per-voice loop length and polymeter on each row. Edits coalesce into
  one pattern push, since neither field has a per-step wire message.

## v0.2 — 2026-08-04

All six panel bugs in this release were found on hardware and fixed against either a
measurement off the machine or a measurement off the stock program image, not against the
1981 drawing. Where the two disagreed, the machine won.

### Fixed

**PRE SCALE selected the wrong detent.** `PanelMap::pre_code` was pairwise swapped, so
every position picked its pair partner: 1 selected 2, 2 selected 1, 3 selected 4, 4
selected 3. Restored to the schematic reading `{3, 1, 2, 0}` and confirmed against the raw
panel probe (`F0 7D 3B F7`) on the machine. A previous pass had swapped these on the theory
that the emulator's decode of the same bits was the stronger reference; it is not, and the
comment in `controls.h` now records how to tell the two candidate fixes apart before
touching that table again.

`PRESCALE_TICKS` is unchanged. It is indexed by detent, not by encoder code, and is pinned
by the owner's manual's worked examples — swapping it instead produces identical panel
behaviour while silently breaking both anchors.

**A double-press of CLEAR could change a pattern's step timing.** Short presses are now
counted as a burst and dispatched only once the burst closes, so a mistimed double-press
can no longer degenerate into two single taps. In a write mode a single CLEAR tap commits
the PRE SCALE dial, so each of those stray taps re-stamped whatever detent the dial
happened to be resting on. The double-press window also went from 500 ms to 600 ms.

Second guard on the same hazard: a CLEAR tap now commits PRE SCALE **only when the dial has
moved since the last commit**, which is the rule the tool layer already followed. The stock
procedure (set PRE SCALE, then press CLEAR) is unaffected.

**Config menu step 9 did nothing.** The emulator switch fired only on a step-9 press that
landed while CLEAR was *already* held past the long-press threshold — not the documented
gesture, not performable in the documented order, and with no feedback either way. Step 9
now boots the emulator immediately.

**The BASIC VARIATION lamp led the music.** It followed the switch, so B lit up to a
measure before the B pattern was audible. It now follows the pattern actually addressed,
which is re-resolved at the measure boundary where the owner's manual puts the change
(p.12). Stopped, the switch applies at once — and now also selects which memory the write
modes edit, which it did not before.

**The 1ST / 2ND PART lamp followed the MODE dial.** While running it now follows the part
being *played*, flipping as playback crosses from the 1st PART into the 2nd. Stopped it
still shows the part being edited, so the SECTION-tool lamp diagnostic still works.

**The chase light sat solid instead of flashing.** It is now lit for exactly one 24-PPQN
tempo-clock tick at the top of each step. That figure is measured off the stock program
image rather than chosen — see below.

**The A lamp flickered in the emulator with BASIC VARIATION on B.** PE0 and PE1 are panel
lamp drive, pattern-RAM address A8/A9 and the CP/RS instrument-data bits all at once; on
the machine it is R9/C5 and R10/C4 that integrate that traffic back into a steady lamp
level. Mirroring the latch bit-for-bit handed the panel the raw traffic and leaned on those
capacitors to sort it out, and the emulated machine's RAM bursts are periodic — one per
sequencer step — so the average dipped in step time. The bridge now does the integrating
itself, sampling only intervals with no RAM access in them and passing the trigger window
through raw so CP and RS still fire on exactly the data the CPU put up.

### Added

**A way out of the emulator from the panel.** A CLEAR double-press reboots into SuperOS.
The emulated machine has no config menu and no idea a second firmware exists, so the bridge
owns the gesture — and withholds the key from the emulated CPU until it resolves, because
stock CLEAR erases the selected pattern in PATTERN CLEAR, erases the rhythm track in
COMPOSE, and commits PRE SCALE in the write modes. A press held past the window passes
through, a lone short press is replayed as a clean press, and a second press switches
firmware with the CPU never having seen either one. The emulator's pattern store is flushed
to EEPROM before either direction of switch.

**`tools/d650/chase_probe.c`** — runs the stock program image on a desktop host against the
same emulator core the firmware uses, and reports what the step-LED display actually does.
This is what established the chase timing: at a fixed PRE SCALE, "one tempo-clock tick", "a
blink at some fixed rate" and "a fixed fraction of the step" are indistinguishable, and all
three fit the flat 12% duty measured at 60, 120 and 240 BPM. An irregular clock separates
them. With tick periods cycling 53 / 83 / 113 ms and the step held at 666 ms, the flash
tracked the individual tick. So the duty is not a free parameter: it falls out of PRE SCALE
(1/8 of the step at PRE SCALE 1, 1/4 at 2, 1/6 at 3, 1/3 at 4) and the flash is a constant
musical length at any tempo.

**Release automation** (`.github/workflows/release.yml`): every push builds the bootloader,
the SPM flash service and the SuperOS firmware, writes a build summary, and attaches the
`.syx` and `.hex`. Pushing a `v*` tag publishes them as a GitHub release.

### Changed

- The PRESCALE tool follows the dial while it is latched, so it doubles as a live readout
  of the decode: turn to detent *n*, step LED *n* lights.
- The SysEx device-info reply (`0x61`) takes its version from `platformio.ini` instead of a
  hand-written constant, so a release cannot ship a binary reporting the previous version.

## v0.1

First release that runs on hardware: bootloader, firmware switching, the emulator including
MIDI clock sync, and the SuperOS layer all exercised on a real machine.
