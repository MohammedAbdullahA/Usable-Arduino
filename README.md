# Arduino Rig Controller — Web UI

Tablet-based control interface for a dual-stepper positioning rig, replacing the old autonomous 8-phase sequence with parametric, on-demand control. An Android tablet connects directly to an Arduino Uno R3 over USB and drives it from a browser page using the Web Serial API — no backend, no cloud, no app store.

This repo is self-contained — no build step, no backend, no dependencies beyond the Arduino IDE and a browser. The web app lives at `index.html` in the repo root (not in a subfolder) specifically so it doubles as a zero-config GitHub Pages site — see [Live UI](#loading-the-web-app-on-the-tablet) below.

## Contents

```
index.html                                   Tablet web app (single self-contained file, doubles as the GitHub Pages site)
firmware/rig_controller/rig_controller.ino   Arduino firmware
docs/commands.md                             Full serial protocol reference
```

## Hardware

| Item | Detail |
|---|---|
| Controller | Arduino Uno R3 |
| Tablet | Android, Chrome or Edge |
| Connection | USB OTG cable, **data-capable** (not charge-only), tablet → Arduino USB-B |
| Motor 1 (rotation) | DM860H driver — step=D2, dir=D3 — 3200 steps/rev, 300 steps/s max, 120 steps/s² accel |
| Motor 2 (linear) | step=D4, dir=D5 — 3000 steps/s max, 1500 steps/s² accel |
| Limit switches | Linear axis MIN/MAX endpoints — D6, D7 (see wiring note below) |
| Library | [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) by Mike McCauley |

**Wiring note on limit switches:** the Uno's only two true external-interrupt pins (D2/D3) are already used by Motor 1's step/dir signals, so the limit switches are wired to D6/D7 instead and serviced with AVR pin-change interrupts (`PCINT2`) — still a hardware interrupt, not a polling loop, just not the "INT0/INT1" pins. Wire each switch normally-open to ground; the firmware uses `INPUT_PULLUP` and treats `LOW` as triggered.

## Safety (non-negotiable, hardware-enforced)

This firmware/UI is a **convenience layer**, not the safety system:

- **Hardware emergency stop**: wire a physical button that cuts motor driver power directly, independent of the Arduino. The web Emergency Stop button is a software convenience on top of this, not a replacement for it.
- **Hardware limit switches** on the linear axis, wired to D6/D7 as above, halt motion via interrupt and latch a fault requiring `CLEAR` before further motion.
- **Serial watchdog**: if the Arduino receives no command (including the automatic `PING` heartbeat) for 5 seconds while a motion is in progress, it halts both motors and latches a fault.
- **Validation on both ends**: the web UI rejects invalid/out-of-range input before sending; the firmware validates independently and rejects with `ERROR` rather than silently clamping or ignoring.
- **Non-blocking motion**: the firmware calls `motor.run()` every loop iteration (never `runToPosition()`), so it stays responsive to `STOP` and to limit-switch interrupts throughout any move.

See [docs/commands.md](docs/commands.md) for the full protocol, including two additions beyond the original spec (`PING`, `CLEAR`) needed to make the watchdog and fault-latching actually usable.

## Uploading the firmware

1. Install the **AccelStepper** library: Arduino IDE → Tools → Manage Libraries → search "AccelStepper" → Install.
2. Open `firmware/rig_controller/rig_controller.ino` in the Arduino IDE (the file must stay inside a folder of the same name — that's already how it's laid out here).
3. Tools → Board → Arduino Uno. Tools → Port → select the Uno's port.
4. Upload.
5. Open the Serial Monitor at **9600 baud**, line ending "Newline", to sanity-check: you should see `READY` on boot, and typing `STATUS` should return a `POS` line.

## Loading the web app on the tablet

The web app is a single self-contained `index.html` at the repo root — no build step, no dependencies.

### Option A — GitHub Pages (already live)

**https://mohammedabdullaha.github.io/Usable-Arduino/**

GitHub Pages serves whatever is at the repo root of the deployed branch — since `index.html` lives there directly, the site *is* the control UI, no separate build/output folder to configure. Repo → Settings → Pages → Source: **Deploy from a branch**, branch **main**, folder **/ (root)**. Every push to `main` updates the live page within a minute or two. Just open that URL on the tablet in Chrome — HTTPS is automatic, which satisfies Web Serial's secure-context requirement.

### Option B — Local file on the tablet

1. Copy `index.html` onto the tablet's storage (via USB file transfer, cloud drive, email — however is convenient).
2. Open it in **Chrome** (not the default Android browser if that's not Chrome/Edge).
3. Chrome treats local `file://` pages as a secure context, so Web Serial works without a server. No internet required — useful as a fallback if the workshop network is unreliable.

Both options produce the same experience — GitHub Pages is easiest to keep updated (edit, push, done); the local file works with zero network dependency.

## Using it

1. Plug the tablet into the Arduino via the OTG cable.
2. Open the page, tap **Connect**, and select the Arduino's port from the browser's picker.
3. Enter a rotation speed (RPM) or linear step count and tap **Spin** / **Move**. **Spin** rotates Motor 1 continuously at that RPM until **Emergency Stop** is pressed. Use the **±** button next to each field if your tablet's keyboard doesn't show a minus key (common on Android numeric keypads).
4. **Home** asks for confirmation before moving. **Spin**, **Move**, and **Emergency Stop** execute immediately, no confirmation.
5. If a limit switch trips or the watchdog times out, a fault banner appears — tap **Clear Fault** once it's safe to resume.
6. The Serial Log panel shows every command sent and every response received, useful for training and debugging.

## Troubleshooting

- **`navigator.serial` unsupported / "Web Serial not available" screen**: confirm you're in Chrome or Edge, not Firefox or Samsung Internet. On some Android Chrome versions, Web Serial support may need `chrome://flags/#enable-experimental-web-platform-features` enabled — this has shipped for Android at different times across Chrome versions, so verify on your specific tablet before relying on it for deployment.
- **No port shows up in the connect picker**: the OTG cable may be charge-only (no data lines) — try a known data-capable cable, ideally one you've confirmed works with a USB mouse/keyboard first. Also confirm the tablet supports USB host mode (most modern Android tablets do).
- **Connection drops mid-session**: the UI shows a "Connection lost" banner and disables action buttons; tap Connect again to re-pick the port. If this happens mid-motion, the firmware's watchdog will also halt the rig within 5 seconds since it stops receiving the automatic `PING`.
- **Everything greyed out after a `STOP` or fault**: `STOP` alone doesn't lock anything — check the fault banner. If it's showing, tap **Clear Fault**. If nothing's showing and buttons are still disabled, check the Serial Log for the last response; a stuck `BUSY` usually means the firmware's `busy` state and the UI's are out of sync, which a `STATUS` request (automatically sent on connect) should self-correct.
- **Serial Monitor and the web app both open at once**: don't — only one program can hold the serial port open at a time. Close the Arduino IDE's Serial Monitor before connecting from the browser.

## Protocol reference

See [docs/commands.md](docs/commands.md) for every command, parameter, response, and error reason, plus an example session.
