/*
 * Dual Stepper Rig Controller — parametric serial command firmware
 * Board:   Arduino Uno R3
 * Library: AccelStepper by Mike McCauley (install via Library Manager)
 *
 * Replaces the old autonomous 8-phase loop with on-demand commands sent
 * over USB serial at 9600 baud. See ../../docs/commands.md for the full
 * protocol reference, including three additions beyond the original spec
 * (PING, CLEAR, and an extended STATUS/POS line) — each is called out
 * below at the point it's implemented.
 *
 * Wiring:
 *   Motor 1 (back-and-forth, DM860H)  step=D9  dir=D3
 *     Continuously-rotating shaft drives a worm screw that advances the
 *     jig along its track — the motor spins, but the jig moves linearly.
 *     Controlled via SPIN (continuous, by RPM — Timer1 hardware), ROTATE
 *     (fixed angle via AccelStepper, at whatever speed is currently set —
 *     fine at low speed, can't sustain high RPM, see note below), or
 *     MOVE_TO (fixed angle AND specified RPM in one command, also on
 *     Timer1 hardware like SPIN so it can actually reach that RPM — the
 *     "run to the far end at speed X, then stop" command).
 *   Motor 2 (rotary — intended to spin the product) step=D4  dir=D5
 *     Not currently connected to a motor. Controlled via ROTARY/HOME.
 *   Limit switch, Motor 1 track MIN (home) D6  (to GND when triggered, INPUT_PULLUP)
 *   Limit switch, Motor 1 track MAX (far end) D7  (to GND when triggered, INPUT_PULLUP)
 *     Normal travel boundaries, not faults — see checkLimitSwitches. MAX
 *     auto-reverses the jig back toward home at half speed; MIN just stops.
 *
 * Note on limit switch pins: D6/D7 are used instead of true external
 * interrupts, via AVR pin-change interrupts (PCINT2) — see PCINT2_vect below.
 *
 * Motor 1 STEP is on D9 (Timer1's OC1A) rather than a plain digital pin.
 * ROTATE/HOME still drive it through AccelStepper's normal digitalWrite-based
 * stepping (fine for the speeds those need). SPIN and MOVE_TO instead hand
 * the pin to Timer1 hardware waveform generation: AccelStepper's polling-loop
 * stepping tops out somewhere in the low kHz on this MCU regardless of
 * commanded RPM, while Timer1 toggling OC1A in CTC mode produces a pulse
 * train timed entirely in hardware, reaching tens of kHz with no jitter and
 * no CPU involvement once at cruise speed. A software ramp (updateSpinRamp)
 * walks the timer's frequency up gradually so the motor still accelerates
 * instead of being commanded to full speed instantly. For SPIN this just
 * ramps up and holds; for MOVE_TO, the same ramp instead recomputes the
 * reachable speed every tick from remaining distance (a standard
 * distance-based trapezoidal profile — see the moveToActive branch), so it
 * accelerates, cruises if there's room, and decelerates to land exactly on
 * target — all at real achievable RPM, unlike ROTATE. Position is
 * integrated in software from frequency x elapsed time (updateSpinPosition):
 * at 8x/16x microstepping the pulse rate reaches tens of kHz, so a per-pulse
 * counting ISR would saturate the CPU — the old TIMER1_COMPA counting ISR
 * has been removed for that reason. See cmdSpin, cmdMoveTo, timer1StartSpin/
 * timer1StopSpin, updateSpinRamp, and updateSpinPosition.
 *
 * STOP decelerates the spin to zero rather than cutting instantly (an
 * instant cut at high RPM regenerates a damaging bus-voltage spike — see
 * cmdStop). Its rate (spinStopDecelSetting, SET_STOP_DECEL_1) is deliberately
 * separate from and much faster than the normal accel setting: a decel at
 * accel's rate can take ~1s and cover enough real rotation/travel to
 * overshoot a mechanical limit on a mounted rig. ESTOP remains an instant
 * cut for genuine emergencies.
 */

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ─── Pins ────────────────────────────────────────────────────────────────
const uint8_t MOTOR1_STEP_PIN = 9;  // Timer1 OC1A — required for hardware-driven SPIN
const uint8_t MOTOR1_DIR_PIN  = 3;
const uint8_t MOTOR2_STEP_PIN = 4;
const uint8_t MOTOR2_DIR_PIN  = 5;
const uint8_t LIMIT_MIN_PIN   = 6;
const uint8_t LIMIT_MAX_PIN   = 7;

// ─── Motor defaults (preserved from the original autonomous sketch) ───────
// STEPS_PER_REV must match the DM860H's DIP-switch pulses/rev setting.
// Changed 400 -> 1600 (8x microstepping): on the usual DM860H table that is
// SW5-8 = ON OFF ON ON, but VERIFY against the table printed on your driver
// (clone tables differ), and power-cycle the driver after changing switches.
// 2x microstepping delivers coarse discrete torque impulses that pump the
// unloaded rotor's mid-band resonance — the buzzing/desync at ~600-700 RPM.
// 8x smooths the torque waveform and suppresses that resonance.
const float STEPS_PER_REV        = 1600.0f;
// Speed/accel values below are in steps (microsteps), so they're scaled x4
// along with STEPS_PER_REV to keep the physical RPM / rev/s^2 unchanged.
const float MOTOR1_MAX_SPEED_DEF = 1200.0f;
const float MOTOR1_ACCEL_DEF     = 80000.0f;  // = ACCEL_MAX — ramps through any resonance band as fast as possible by default
const float MOTOR2_MAX_SPEED_DEF = 3000.0f;
const float MOTOR2_ACCEL_DEF     = 1500.0f;

// ─── Parameter validation limits ───────────────────────────────────────────
// Widened from the original ±3600° (10 revolutions) — real jig travel is a
// full worm-screw traverse, e.g. ~81,868° for the 1400mm track. Shared by
// ROTATE and MOVE_TO (steps = degrees * STEPS_PER_REV / 360 stays well
// within `long` range at this scale).
const float ROTATE_DEG_MIN   = -1000000.0f;
const float ROTATE_DEG_MAX   =  1000000.0f;
const long  ROTARY_STEPS_MIN = -100000L;
const long  ROTARY_STEPS_MAX =  100000L;
// SPEED_MAX no longer reflects a real safety ceiling — it's set high enough
// that 3000 RPM (the manufacturer spec, 160000 steps/s at 3200 steps/rev)
// clears validation. AccelStepper on an Uno is software-timed, though: its
// real achievable step rate tops out far below that (roughly 20-40 kHz in
// practice), so requests near the manufacturer figure are expected to be
// throughput-limited by the MCU, not rejected by this check.
const float SPEED_MAX        = 200000.0f;
const float ACCEL_MAX        = 80000.0f;  // scaled x4 with STEPS_PER_REV (same physical rev/s^2 ceiling as before)
// STOP must decelerate much faster than a normal start ramps up — a full
// accel-rate ramp-down from cruise speed covers real physical travel (at
// MOTOR1_ACCEL_DEF that's roughly a full second, which can be enough
// rotation/travel to overshoot a mechanical limit on a mounted rig). This is
// deliberately separate from motor1AccelSetting so STOP stays fast even if
// accel is tuned low for gentle starts.
const float STOP_DECEL_MAX     = 400000.0f;
const float STOP_DECEL_DEFAULT = 400000.0f;  // ~0.2s from 3000 RPM (1600 ppr) — smooth enough to avoid a regen spike, fast enough to stay within mechanical limits

// ─── Protocol / timing ──────────────────────────────────────────────────
const unsigned long WATCHDOG_TIMEOUT_MS     = 5000;
const unsigned long POS_BROADCAST_INTERVAL  = 250;
const size_t        CMD_BUFFER_SIZE         = 64;

AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

// ─── State ──────────────────────────────────────────────────────────────
bool busy = false;            // a motion command is in progress (global — the rig moves one axis at a time, matching the original sequence)
bool movingMotor1 = false;
bool movingMotor2 = false;
bool faultActive = false;     // set by a watchdog trip only; requires CLEAR to resume. Limit switches (D6/D7) are normal travel boundaries, handled entirely in checkLimitSwitches — they never set this.
unsigned long lastCommandMillis = 0;
unsigned long lastBroadcastMillis = 0;
float motor1AccelSetting = MOTOR1_ACCEL_DEF;  // mirrors motor1's AccelStepper accel so the hardware SPIN ramp can use the same value
float spinStopDecelSetting = STOP_DECEL_DEFAULT;  // used only for STOP's ramp-down — deliberately independent of accel, see SET_STOP_DECEL_1

char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t bufIndex = 0;
char lastCommandText[CMD_BUFFER_SIZE];  // raw text of the command currently in progress, echoed in OK/DONE

volatile bool limitMinTriggered = false;
volatile bool limitMaxTriggered = false;

// Arm/disarm latch per switch, on top of the raw ISR flags above. Without
// this, a jig resting against (or lightly vibrating on) a switch keeps the
// pin at/near LOW indefinitely, and any bounce re-fires the ISR — which
// would re-trigger the halt repeatedly, and worse, could abort a later
// command that's deliberately moving away from that same switch (the
// "never releases" symptom). Once handled, a switch disarms itself and
// stays disarmed until the pin has read cleanly released (HIGH) for
// LIMIT_REARM_STABLE_MS continuously — only then can it trigger again.
const unsigned long LIMIT_REARM_STABLE_MS = 100;
bool limitMinArmed = true;
bool limitMaxArmed = true;
unsigned long limitMinHighSinceMillis = 0;
unsigned long limitMaxHighSinceMillis = 0;

// ─── Hardware-timer SPIN state (Timer1 drives D9/OC1A directly) ───────────
// No per-pulse ISR: position is integrated from frequency x time by
// updateSpinPosition(). Display-grade accuracy, which is all continuous
// rotation needs — and it costs nothing per pulse, so 8x/16x microstep
// rates (tens of kHz) don't load the CPU at all.
long  spinStepPos   = 0;         // integrated step position while spinningHW
float spinStepFrac  = 0;         // fractional-step remainder of the integrator
unsigned long spinPosLastMicros = 0;
int8_t  spinDirection  = 0;      // +1 or -1 for the duration of the current spin
bool    spinningHW     = false;  // true while Timer1 (not AccelStepper) is driving Motor 1
bool    spinStoppingSoft = false; // true while a STOP is ramping the spin down to zero
float   spinTargetFreq = 0;      // Hz the ramp is walking toward (plain SPIN only)
float   spinCurrentFreq = 0;     // Hz currently loaded into the timer
unsigned long spinRampLastMillis = 0;

// ─── MOVE_TO state — position-controlled move on the same Timer1 hardware
// path as SPIN, so it can actually reach the requested RPM (AccelStepper's
// software polling loop can't — see header comment). Each ramp tick
// recomputes the reachable speed from remaining distance (a standard
// distance-based trapezoidal profile), so it naturally accelerates, holds
// cruise if there's room, and decelerates to land exactly on target.
bool    moveToActive       = false;
long    moveToStartPos     = 0;   // spinStepPos when this move began
long    moveToTargetDistance = 0; // unsigned steps to travel (direction via spinDirection)
float   spinCruiseFreq     = 0;   // Hz requested for the current MOVE_TO (separate from spinTargetFreq's plain-SPIN meaning)

// ─── Limit switch interrupts (PCINT2 covers digital pins 0-7) ─────────────
ISR(PCINT2_vect) {
  if (digitalRead(LIMIT_MIN_PIN) == LOW) limitMinTriggered = true;
  if (digitalRead(LIMIT_MAX_PIN) == LOW) limitMaxTriggered = true;
}

void setupLimitInterrupts() {
  pinMode(LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_MAX_PIN, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // D6, D7
}

// ─── Timer1 hardware pulse generator for SPIN ─────────────────────────────
// Picks the smallest prescaler that keeps OCR1A in range for the requested
// frequency, maximizing timer resolution at high speed.
bool computeTimer1Config(float freqHz, uint16_t& ocrOut, uint8_t& csBitsOut) {
  static const uint16_t PRESCALERS[] = {1, 8, 64, 256, 1024};
  static const uint8_t  CS_BITS[]    = {
    (1 << CS10),
    (1 << CS11),
    (1 << CS11) | (1 << CS10),
    (1 << CS12),
    (1 << CS12) | (1 << CS10)
  };
  for (uint8_t i = 0; i < 5; i++) {
    long ocr = lround(F_CPU / (2.0 * PRESCALERS[i] * freqHz)) - 1;
    if (ocr >= 1 && ocr <= 65535L) {
      ocrOut = (uint16_t)ocr;
      csBitsOut = CS_BITS[i];
      return true;
    }
  }
  return false;
}

void timer1SetFrequency(float freqHz) {
  uint16_t ocr;
  uint8_t cs;
  if (!computeTimer1Config(freqHz, ocr, cs)) return;  // out of representable range, leave timer as-is
  noInterrupts();
  TCCR1B = (1 << WGM12) | cs;
  OCR1A = ocr;
  // If the counter has already passed the new (smaller) OCR value, CTC mode
  // would run all the way to 65535 before matching again — a multi-ms dead
  // gap in the pulse train on every ramp update. Reset the count instead;
  // worst case that stretches one pulse slightly, which the motor never sees.
  if (TCNT1 > ocr) TCNT1 = 0;
  interrupts();
}

void timer1StartSpin(float freqHz) {
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR1_STEP_PIN, LOW);
  spinStepPos = motor1.currentPosition();  // continue counting from wherever AccelStepper left off
  spinStepFrac = 0;
  spinPosLastMicros = micros();
  TCNT1 = 0;
  TCCR1A = (1 << COM1A0);  // toggle OC1A (D9) on compare match — pure hardware waveform, no ISR involved at all
  timer1SetFrequency(freqHz);
}

void timer1StopSpin() {
  TCCR1A &= ~(1 << COM1A0);  // disconnect OC1A, hand the pin back to plain digital I/O
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
  digitalWrite(MOTOR1_STEP_PIN, LOW);
  updateSpinPosition(true);  // flush the last partial integration interval
  motor1.setCurrentPosition(spinStepPos);  // keep AccelStepper's position in sync for ROTATE/HOME/STATUS afterward
}

// Returns Motor 1's logical position regardless of which path is driving it.
long motor1Position() {
  return spinningHW ? spinStepPos : motor1.currentPosition();
}

// Integrates step position from the commanded frequency. Called every loop
// while spinningHW; ~200Hz update rate is plenty for the POS display.
void updateSpinPosition(bool force) {
  unsigned long now = micros();
  float dt = (now - spinPosLastMicros) / 1.0e6f;
  if (!force && dt < 0.005f) return;
  spinPosLastMicros = now;
  spinStepFrac += spinCurrentFreq * dt;
  long whole = (long)spinStepFrac;
  spinStepFrac -= whole;
  spinStepPos += spinDirection * whole;
}

// Drives the Timer1 frequency each tick. Three modes, checked in priority
// order: (1) an explicit STOP always wins — fast decel to zero via
// spinStopDecelSetting, same regen-safety reasoning as before; (2) a
// MOVE_TO recomputes the reachable speed from remaining distance every
// tick (freq = sqrt(2 * accel * remainingSteps), capped at the requested
// cruise speed) — this is what lets it actually accelerate, hold cruise
// if there's room, and decelerate to land on target, all using the
// motor1AccelSetting rate in both directions for a smooth, symmetric
// profile; (3) plain SPIN just ramps to spinTargetFreq and holds — it has
// no target, so it never decelerates on its own (STOP handles that).
void updateSpinRamp() {
  unsigned long now = millis();
  float elapsedSec = (now - spinRampLastMillis) / 1000.0f;
  if (elapsedSec < 0.002f) return;  // throttle register writes to ~500Hz — small, smooth frequency increments
  spinRampLastMillis = now;

  if (spinStoppingSoft) {
    float freq = spinCurrentFreq - spinStopDecelSetting * elapsedSec;
    if (freq < 0) freq = 0;
    spinCurrentFreq = freq;
    if (freq <= 30.0f) {
      timer1StopSpin();
      spinningHW = false;
      spinStoppingSoft = false;
      moveToActive = false;
      busy = false;
      movingMotor1 = false;
      Serial.println(F("DONE STOP"));
      return;
    }
    timer1SetFrequency(freq);
    return;
  }

  if (moveToActive) {
    updateSpinPosition(false);  // refresh spinStepPos before measuring remaining distance
    long traveled = labs(spinStepPos - moveToStartPos);
    long remaining = moveToTargetDistance - traveled;

    if (remaining <= 0) {
      timer1StopSpin();
      spinningHW = false;
      moveToActive = false;
      busy = false;
      movingMotor1 = false;
      Serial.println(F("DONE MOVE_TO"));
      return;
    }

    // The speed that still allows decelerating to zero exactly at the
    // remaining distance, at motor1AccelSetting — this is the classic
    // "how fast can I be going right now and still stop in time" formula.
    float maxReachable = sqrt(2.0f * motor1AccelSetting * (float)remaining);
    float effectiveTarget = (maxReachable < spinCruiseFreq) ? maxReachable : spinCruiseFreq;

    float freq;
    if (spinCurrentFreq < effectiveTarget) {
      freq = spinCurrentFreq + motor1AccelSetting * elapsedSec;
      if (freq > effectiveTarget) freq = effectiveTarget;
    } else {
      freq = spinCurrentFreq - motor1AccelSetting * elapsedSec;
      if (freq < effectiveTarget) freq = effectiveTarget;
    }
    if (freq < 1.0f) freq = 1.0f;  // keep the timer in its representable range while still moving
    spinCurrentFreq = freq;
    timer1SetFrequency(freq);
    return;
  }

  // Plain SPIN: ramp toward spinTargetFreq and hold. No auto-decel — STOP
  // (the branch above) is what brings it back down.
  if (spinCurrentFreq == spinTargetFreq) return;
  float freq = spinCurrentFreq + motor1AccelSetting * elapsedSec;
  if (freq > spinTargetFreq) freq = spinTargetFreq;
  spinCurrentFreq = freq;
  timer1SetFrequency(freq);
}

// ─── Setup ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000) { }

  motor1.setMaxSpeed(MOTOR1_MAX_SPEED_DEF);
  motor1.setAcceleration(MOTOR1_ACCEL_DEF);
  motor1.setCurrentPosition(0);

  motor2.setMaxSpeed(MOTOR2_MAX_SPEED_DEF);
  motor2.setAcceleration(MOTOR2_ACCEL_DEF);
  motor2.setCurrentPosition(0);

  setupLimitInterrupts();

  Serial.println(F("READY"));
}

// ─── Main loop — non-blocking: every call is cheap, motors are serviced
//     every iteration, so STOP/limit/watchdog react within a loop tick. ──
void loop() {
  readSerialCommands();
  checkLimitSwitches();
  checkWatchdog();

  if (!faultActive) {
    if (spinningHW) { updateSpinRamp(); updateSpinPosition(false); }
    else motor1.run();
    motor2.run();
  }

  handleMotionCompletion();
  periodicBroadcast();
}

// ─── Serial line reader (non-blocking) ─────────────────────────────────────
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIndex > 0) {
        cmdBuffer[bufIndex] = '\0';
        strncpy(lastCommandText, cmdBuffer, CMD_BUFFER_SIZE - 1);
        lastCommandText[CMD_BUFFER_SIZE - 1] = '\0';
        lastCommandMillis = millis();  // any received line counts as a watchdog heartbeat
        processCommand(cmdBuffer);
        bufIndex = 0;
      }
    } else if (bufIndex < CMD_BUFFER_SIZE - 1) {
      cmdBuffer[bufIndex++] = c;
    } else {
      bufIndex = 0;
      Serial.println(F("ERROR COMMAND_TOO_LONG"));
    }
  }
}

// ─── Numeric parsing helpers — reject anything that isn't a clean number ──
bool parseFloatArg(const char* s, float* out) {
  if (s == NULL || *s == '\0') return false;
  char* end;
  float v = strtod(s, &end);
  while (*end == ' ') end++;
  if (end == s || *end != '\0') return false;
  *out = v;
  return true;
}

bool parseLongArg(const char* s, long* out) {
  if (s == NULL || *s == '\0') return false;
  char* end;
  long v = strtol(s, &end, 10);
  while (*end == ' ') end++;
  if (end == s || *end != '\0') return false;
  *out = v;
  return true;
}

// For "MOVE_TO <degrees> <rpm>" — two space-separated numbers, nothing else.
bool parseTwoFloatArgs(const char* s, float* out1, float* out2) {
  if (s == NULL || *s == '\0') return false;
  char* mid;
  float v1 = strtod(s, &mid);
  if (mid == s) return false;
  while (*mid == ' ') mid++;
  char* end;
  float v2 = strtod(mid, &end);
  if (end == mid) return false;
  while (*end == ' ') end++;
  if (*end != '\0') return false;
  *out1 = v1;
  *out2 = v2;
  return true;
}

// ─── Command dispatch ───────────────────────────────────────────────────
void processCommand(char* line) {
  char* command = strtok(line, " ");
  char* argStr  = strtok(NULL, "");

  if (command == NULL) return;

  if (strcmp(command, "STOP") == 0) {
    cmdStop();
  } else if (strcmp(command, "ESTOP") == 0) {
    cmdEstop();
  } else if (strcmp(command, "STATUS") == 0) {
    sendStatusLine();
  } else if (strcmp(command, "PING") == 0) {
    // Addition beyond the original spec: an explicit heartbeat command,
    // referenced by the brief itself ("...or heartbeat ping"). The web UI
    // sends this continuously so the watchdog below can detect a dropped
    // connection even when the rig is idle between motions.
    Serial.println(F("OK PING"));
  } else if (strcmp(command, "CLEAR") == 0) {
    // Addition beyond the original spec: an explicit fault acknowledgment.
    // Only a watchdog (dropped-connection) trip latches faultActive now —
    // that's a genuinely serious situation, so it needs a deliberate,
    // operator-initiated CLEAR to resume, mirroring how the E-stop panel
    // itself typically needs a manual reset/twist before restart. Limit
    // switches are normal travel boundaries and never set faultActive
    // (see checkLimitSwitches), so they never need this.
    faultActive = false;
    Serial.println(F("OK CLEAR"));
  } else if (strcmp(command, "ROTATE") == 0) {
    cmdRotate(argStr);
  } else if (strcmp(command, "MOVE_TO") == 0) {
    cmdMoveTo(argStr);
  } else if (strcmp(command, "SPIN") == 0) {
    cmdSpin(argStr);
  } else if (strcmp(command, "ROTARY") == 0) {
    cmdRotary(argStr);
  } else if (strcmp(command, "HOME") == 0) {
    cmdHome();
  } else if (strcmp(command, "SET_SPEED_1") == 0) {
    cmdSetSpeed(motor1, argStr);
  } else if (strcmp(command, "SET_SPEED_2") == 0) {
    cmdSetSpeed(motor2, argStr);
  } else if (strcmp(command, "SET_ACCEL_1") == 0) {
    cmdSetAccel(motor1, argStr);
  } else if (strcmp(command, "SET_STOP_DECEL_1") == 0) {
    cmdSetStopDecel(argStr);
  } else if (strcmp(command, "SET_ACCEL_2") == 0) {
    cmdSetAccel(motor2, argStr);
  } else {
    Serial.println(F("ERROR UNKNOWN_COMMAND"));
  }
}

// ─── Command handlers ───────────────────────────────────────────────────
void cmdStop() {
  // Soft stop: if a hardware SPIN is running, ramp it down to zero at the
  // configured accel instead of cutting the pulse train instantly — an
  // instant cut from high RPM regenerates a bus-voltage spike that can
  // damage the driver/supply. busy stays true during the ramp-down;
  // DONE STOP is sent by updateSpinRamp when the motor actually halts.
  if (spinningHW && !faultActive) {
    spinTargetFreq = 0;
    spinStoppingSoft = true;
    spinRampLastMillis = millis();
    Serial.println(F("OK STOP"));
    return;
  }
  // Not spinning (or mid-fault): slow AccelStepper moves carry negligible
  // stored energy, so an immediate halt is safe and simplest.
  emergencyHalt();
  Serial.println(F("DONE STOP"));
}

void cmdEstop() {
  // ESTOP always halts instantly, even mid-fault — this is the software
  // panic button. Accept the regen risk: a genuine emergency outranks
  // driver health. (The hardware E-stop that cuts driver power remains
  // the real safety system, per the README.)
  emergencyHalt();
  Serial.println(F("DONE ESTOP"));
}

void cmdRotate(char* argStr) {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  float degrees;
  if (!parseFloatArg(argStr, &degrees)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (degrees < ROTATE_DEG_MIN || degrees > ROTATE_DEG_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  long steps = lround((double)degrees * STEPS_PER_REV / 360.0);
  motor1.move(steps);
  busy = true;
  movingMotor1 = true;
  movingMotor2 = false;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

// MOVE_TO <degrees> <rpm> — moves Motor 1 by a signed angle (same convention
// as ROTATE: sign gives direction) at a specified cruise speed, stopping
// automatically on arrival. Drives D9 via Timer1 hardware, same as SPIN —
// NOT through AccelStepper's polling loop, which can't sustain the pulse
// rates real RPM needs (see header comment). The distance-based ramp that
// makes this land exactly on target lives in updateSpinRamp's moveToActive
// branch; this function just sets up the target and kicks off the spin.
void cmdMoveTo(char* argStr) {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  float degrees, rpm;
  if (!parseTwoFloatArgs(argStr, &degrees, &rpm)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (degrees < ROTATE_DEG_MIN || degrees > ROTATE_DEG_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  if (rpm <= 0) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  float freq = rpm * STEPS_PER_REV / 60.0f;
  if (freq > SPEED_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  uint16_t ocrCheck;
  uint8_t csCheck;
  if (!computeTimer1Config(freq, ocrCheck, csCheck)) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  long steps = lround((double)degrees * STEPS_PER_REV / 360.0);
  if (steps == 0) {
    Serial.println(F("ERROR OUT_OF_RANGE"));  // nothing to move
    return;
  }

  spinDirection = (steps > 0) ? 1 : -1;
  digitalWrite(MOTOR1_DIR_PIN, steps > 0 ? HIGH : LOW);

  spinCruiseFreq = freq;
  spinCurrentFreq = 1.0f;  // start near zero rather than jumping straight to cruise speed
  spinStoppingSoft = false;
  spinRampLastMillis = millis();
  spinningHW = true;
  timer1StartSpin(spinCurrentFreq);

  moveToActive = true;
  moveToStartPos = spinStepPos;         // set by timer1StartSpin, just started this move
  moveToTargetDistance = labs(steps);

  busy = true;
  movingMotor1 = true;
  movingMotor2 = false;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdSpin(char* argStr) {
  // Addition beyond the original spec: continuous RPM-driven spin of
  // Motor 1's shaft, as an alternative to ROTATE's fixed-angle moves —
  // this shaft drives the worm screw that advances the jig (see wiring
  // note at top of file). Drives D9 directly from Timer1 hardware (see timer1StartSpin) instead of through
  // AccelStepper's polling loop, since the loop's real achievable pulse rate
  // tops out far below what's needed to approach the motor's rated speed.
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  float rpm;
  if (!parseFloatArg(argStr, &rpm)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (rpm == 0) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  float freq = fabs(rpm) * STEPS_PER_REV / 60.0f;
  if (freq > SPEED_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  uint16_t ocrCheck;
  uint8_t csCheck;
  if (!computeTimer1Config(freq, ocrCheck, csCheck)) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  spinDirection = (rpm > 0) ? 1 : -1;
  digitalWrite(MOTOR1_DIR_PIN, rpm > 0 ? HIGH : LOW);

  spinTargetFreq = freq;
  spinCurrentFreq = 1.0f;  // start near zero rather than jumping straight to cruise speed
  spinStoppingSoft = false;
  moveToActive = false;  // plain SPIN has no target — must not fall into the moveToActive ramp branch
  spinRampLastMillis = millis();
  spinningHW = true;
  timer1StartSpin(spinCurrentFreq);

  busy = true;
  movingMotor1 = true;
  movingMotor2 = false;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdRotary(char* argStr) {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  long steps;
  if (!parseLongArg(argStr, &steps)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (steps < ROTARY_STEPS_MIN || steps > ROTARY_STEPS_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  motor2.move(steps);
  busy = true;
  movingMotor1 = false;
  movingMotor2 = true;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdHome() {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  movingMotor1 = motor1.currentPosition() != 0;
  movingMotor2 = motor2.currentPosition() != 0;

  if (!movingMotor1 && !movingMotor2) {
    Serial.println(F("DONE HOME"));
    return;
  }

  motor1.moveTo(0);
  motor2.moveTo(0);
  busy = true;

  Serial.println(F("OK HOME"));
}

void cmdSetSpeed(AccelStepper& motor, char* argStr) {
  if (busy) { Serial.println(F("BUSY")); return; }

  float value;
  if (!parseFloatArg(argStr, &value)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (value <= 0 || value > SPEED_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  motor.setMaxSpeed(value);
  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdSetStopDecel(char* argStr) {
  // Allowed even while busy/spinning (unlike other SET_ commands) — you may
  // need to tighten this the moment you discover the current value overshoots
  // a mechanical limit, without having to stop the rig first to change it.
  float value;
  if (!parseFloatArg(argStr, &value)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (value <= 0 || value > STOP_DECEL_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  spinStopDecelSetting = value;
  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdSetAccel(AccelStepper& motor, char* argStr) {
  if (busy) { Serial.println(F("BUSY")); return; }

  float value;
  if (!parseFloatArg(argStr, &value)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (value <= 0 || value > ACCEL_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  motor.setAcceleration(value);
  if (&motor == &motor1) motor1AccelSetting = value;  // keeps the hardware SPIN ramp in sync
  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

// ─── Safety: emergency halt, limit switches, watchdog ─────────────────────
void emergencyHalt() {
  // Immediate stop, not a decelerated one: force distanceToGo() to zero so
  // run() issues no further steps starting the very next loop() iteration.
  if (spinningHW) {
    timer1StopSpin();
    spinningHW = false;
  }
  spinStoppingSoft = false;
  moveToActive = false;  // defensive: guarantees a later plain SPIN doesn't inherit a stale MOVE_TO target
  motor1.setSpeed(0);
  motor1.moveTo(motor1.currentPosition());
  motor2.setSpeed(0);
  motor2.moveTo(motor2.currentPosition());

  busy = false;
  movingMotor1 = false;
  movingMotor2 = false;
}

// Motor 1's limit switches mark the jig's normal travel boundaries — hitting
// one is expected, wanted behavior during ordinary operation, not an error
// condition. Neither event sets faultActive or needs CLEAR (unlike a
// watchdog timeout, which stays a genuine fault requiring operator
// acknowledgment). D7/MAX (far end): stop, then automatically reverse back
// toward home at half the speed it was just moving, continuing until
// D6/MIN triggers. D6/MIN (home): just stop — no further action.
//
// Each switch only acts on its first fresh press (see the arm/disarm state
// declared above) — a jig resting against, or bouncing on, a switch after
// the initial touch is ignored rather than re-triggering the halt or
// blocking a later command that moves away from it. It re-arms only once
// the pin reads reliably released for LIMIT_REARM_STABLE_MS.
void checkLimitSwitches() {
  unsigned long now = millis();

  if (!limitMinArmed) {
    if (digitalRead(LIMIT_MIN_PIN) == HIGH) {
      if (limitMinHighSinceMillis == 0) limitMinHighSinceMillis = now;
      if (now - limitMinHighSinceMillis >= LIMIT_REARM_STABLE_MS) limitMinArmed = true;
    } else {
      limitMinHighSinceMillis = 0;  // still (or again) pressed — release timer resets
    }
  }
  if (!limitMaxArmed) {
    if (digitalRead(LIMIT_MAX_PIN) == HIGH) {
      if (limitMaxHighSinceMillis == 0) limitMaxHighSinceMillis = now;
      if (now - limitMaxHighSinceMillis >= LIMIT_REARM_STABLE_MS) limitMaxArmed = true;
    } else {
      limitMaxHighSinceMillis = 0;
    }
  }

  if (limitMaxTriggered) {
    limitMaxTriggered = false;  // consume now — the ISR only sets it, never clears it
    if (limitMaxArmed) {
      limitMaxArmed = false;  // one-shot: ignore this switch until confirmed released again
      // Half of whatever it was actually doing at the moment of impact —
      // if it wasn't on the Timer1 hardware path (e.g. hit during a ROTATE),
      // fall back to half the default cruise speed as a sensible reference.
      float returnFreq = spinningHW ? (spinCurrentFreq / 2.0f) : (MOTOR1_MAX_SPEED_DEF / 2.0f);
      if (returnFreq < 200.0f) returnFreq = 200.0f;  // floor so the return leg isn't a near-stall crawl
      int8_t returnDir = (spinDirection >= 0) ? -1 : 1;  // reverse of whatever direction it was heading
      emergencyHalt();
      startReturnToHome(returnFreq, returnDir);
      Serial.println(F("OK LIMIT_MAX_RETURNING"));
    }
    return;  // don't also process a MIN trigger on this same tick
  }
  if (limitMinTriggered) {
    limitMinTriggered = false;
    if (limitMinArmed) {
      limitMinArmed = false;
      emergencyHalt();
      Serial.println(F("DONE LIMIT_MIN_HOME"));
    }
  }
}

// Launches Motor 1 on the Timer1 hardware path (same as SPIN) at freqHz in
// the given direction, with no distance target — it runs until checkLimit-
// Switches sees D6/MIN trigger and stops it. This is the auto-return leg
// after hitting D7/MAX.
void startReturnToHome(float freqHz, int8_t direction) {
  spinDirection = direction;
  digitalWrite(MOTOR1_DIR_PIN, direction > 0 ? HIGH : LOW);

  spinTargetFreq = freqHz;
  spinCurrentFreq = 1.0f;  // start near zero rather than jumping straight to the return speed
  spinStoppingSoft = false;
  moveToActive = false;
  spinRampLastMillis = millis();
  spinningHW = true;
  timer1StartSpin(spinCurrentFreq);

  busy = true;
  movingMotor1 = true;
  movingMotor2 = false;
}

void checkWatchdog() {
  if (busy && !faultActive && (millis() - lastCommandMillis > WATCHDOG_TIMEOUT_MS)) {
    emergencyHalt();
    faultActive = true;
    Serial.println(F("ERROR WATCHDOG_TIMEOUT"));
  }
}

// ─── Motion completion + status broadcast ─────────────────────────────────
void handleMotionCompletion() {
  if (!busy) return;
  if (spinningHW) return;  // any Timer1-driven move (SPIN or MOVE_TO) ends itself via updateSpinRamp — STOP, ESTOP, or arrival at target — never here

  bool m2Done = !movingMotor1 || motor1.distanceToGo() == 0;
  bool m1Done = !movingMotor2 || motor2.distanceToGo() == 0;

  if (m1Done && m2Done) {
    busy = false;
    movingMotor1 = false;
    movingMotor2 = false;
    Serial.print(F("DONE "));
    Serial.println(lastCommandText);
  }
}

void sendStatusLine() {
  const char* state = faultActive ? "FAULT" : (busy ? "MOVING" : "IDLE");
  Serial.print(F("POS "));
  Serial.print(motor1Position());
  Serial.print(' ');
  Serial.print(motor2.currentPosition());
  Serial.print(' ');
  Serial.println(state);
}

void periodicBroadcast() {
  // Addition beyond the original spec: while a motion is in progress,
  // stream POS lines every 250ms unprompted so the UI's live position
  // display updates smoothly without needing to poll STATUS itself.
  if (busy && millis() - lastBroadcastMillis >= POS_BROADCAST_INTERVAL) {
    lastBroadcastMillis = millis();
    sendStatusLine();
  }
}
