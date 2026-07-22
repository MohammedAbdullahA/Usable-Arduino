# Serial Command Protocol

Plain ASCII over USB serial at **9600 baud**. Every command is terminated by `\n` (the web UI sends `\r`-free `\n`; the firmware also tolerates `\r\n`). Every response is terminated by `\n`.

This matches the protocol in the project brief, plus three small additions needed to make the safety requirements (watchdog, limit switches) actually usable from the UI — each is marked **(addition)** below with the reason it was added.

## Commands (tablet → Arduino)

| Command | Parameters | Behavior |
|---|---|---|
| `ROTATE <degrees>` | signed float, e.g. `90`, `-45.5` | Rotate Motor 1 by the specified angle (relative move). Positive = CW, negative = CCW. Range: −3600 to +3600. Rejected with `BUSY` if a motion is already in progress, or `ERROR FAULT_ACTIVE` if the rig is faulted. |
| `SPIN <rpm>` **(addition)** | signed float, non-zero, e.g. `30`, `-12.5` | Spin Motor 1 continuously at the given RPM (converted to steps/s via `STEPS_PER_REV`). Positive = CW, negative = CCW. Runs until `STOP` is sent — there is no target angle, so it never completes on its own. Rejected with `ERROR OUT_OF_RANGE` if the resulting speed exceeds 20000 steps/s, `BUSY` if a motion is already in progress, or `ERROR FAULT_ACTIVE` if the rig is faulted. |
| `LINEAR <steps>` | signed integer, e.g. `30000`, `-15000` | Move Motor 2 by the specified step count (relative move). Positive = forward, negative = backward. Range: −100000 to +100000. Same `BUSY`/`FAULT_ACTIVE` rejection rules as `ROTATE`. |
| `STOP` | — | Immediate halt of both motors. Always processed, even mid-fault or mid-motion. Does **not** set a fault — this is a deliberate operator action, and the rig accepts new commands right away afterward. |
| `HOME` | — | Return both motors to absolute position 0. If both are already at 0, responds `DONE HOME` immediately. Subject to the same `BUSY`/`FAULT_ACTIVE` rules as motion commands. |
| `STATUS` | — | Request an immediate `POS` response. Always processed, regardless of state. |
| `SET_SPEED_1 <value>` | float, steps/s, `0 < value ≤ 20000` | Set Motor 1 max speed. Rejected with `BUSY` while a motion is in progress. |
| `SET_SPEED_2 <value>` | float, steps/s, `0 < value ≤ 20000` | Set Motor 2 max speed. Same rule. |
| `SET_ACCEL_1 <value>` | float, steps/s², `0 < value ≤ 20000` | Set Motor 1 acceleration. Same rule. |
| `SET_ACCEL_2 <value>` | float, steps/s², `0 < value ≤ 20000` | Set Motor 2 acceleration. Same rule. |
| `PING` **(addition)** | — | No-op heartbeat. The brief's watchdog requirement mentions accepting "any command (or heartbeat ping)" to reset the timeout — this is that ping. The web UI sends it every second automatically; you don't need to send it manually. Responds `OK PING`. |
| `CLEAR` **(addition)** | — | Clears a fault (see below) and re-enables motion commands. Required after a limit-switch trip or a watchdog timeout — the rig will not move again until an operator explicitly acknowledges the fault. Responds `OK CLEAR`. |

## Responses (Arduino → tablet)

| Response | Meaning |
|---|---|
| `READY` | Sent once on boot. |
| `OK <command>` | Command accepted (echoes the exact text received). For `ROTATE`/`LINEAR`/`HOME`, motion has started — expect a later `DONE`. For `SET_SPEED_*`/`SET_ACCEL_*`, the change has already been applied. |
| `DONE <command>` | The motion started by the matching `OK` has completed. `STOP` skips straight to `DONE STOP` (see below). |
| `POS <m1_steps> <m2_steps> <state>` **(extended)** | Current position of Motor 1 and Motor 2 in steps, plus `<state>` ∈ `IDLE` / `MOVING` / `FAULT`. Sent in response to `STATUS`, and automatically every 250ms while a motion is in progress so the UI's live display updates without polling. The original brief's table listed `POS <m1> <m2>` without a state field — the third field was added so the UI can tell "moving" from "faulted" from a single line. |
| `BUSY` | Command rejected — a motion is already in progress. |
| `ERROR <reason>` | Command could not be parsed or executed. See reasons below. |

### `ERROR` reasons

| Reason | Cause |
|---|---|
| `UNKNOWN_COMMAND` | First word isn't a recognized command. |
| `INVALID_PARAM` | Parameter missing or not a clean number. |
| `OUT_OF_RANGE` | Parameter parsed fine but is outside the allowed range. |
| `COMMAND_TOO_LONG` | Line exceeded the 64-byte command buffer. |
| `FAULT_ACTIVE` | A motion command was sent while the rig is faulted — send `CLEAR` first. |
| `LIMIT_MIN` / `LIMIT_MAX` | A hardware limit switch on the linear axis tripped. Halts both motors and **latches a fault** — requires `CLEAR`. |
| `WATCHDOG_TIMEOUT` | No command (including `PING`) was received for 5 seconds while a motion was in progress. Halts both motors and **latches a fault** — requires `CLEAR`. |

## Notes on the "one axis at a time" model

The rig only moves one axis at a time — this mirrors the original autonomous sequence, which never ran both motors simultaneously. `busy` is a single global flag: any `ROTATE`, `SPIN`, `LINEAR`, or `HOME` in progress rejects a second motion command with `BUSY`, regardless of which motor it targets.

`SPIN` shares Motor 1 with `ROTATE` and follows the same busy-gating rules; the difference is only in what ends the motion — `ROTATE` finishes on its own once the target angle is reached, `SPIN` only ever ends via `STOP` (or a fault).

## Example session

```
→ READY                      (on connect)
→ PING                       (every 1s, automatic)
← OK PING

→ ROTATE 90
← OK ROTATE 90
← POS 0 0 MOVING             (every 250ms while moving)
← POS 720 0 MOVING
← DONE ROTATE 90

→ LINEAR 30000
← OK LINEAR 30000
← DONE LINEAR 30000

→ STATUS
← POS 1600 30000 IDLE

→ STOP                       (mid-motion, any time)
← DONE STOP

(limit switch trips during a LINEAR move)
← ERROR LIMIT_MAX
→ ROTATE 10
← ERROR FAULT_ACTIVE
→ CLEAR
← OK CLEAR
→ ROTATE 10
← OK ROTATE 10
```
