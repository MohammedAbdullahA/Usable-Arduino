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
 *   Motor 1 (rotation, DM860H)  step=D9  dir=D3
 *   Motor 2 (linear)            step=D4  dir=D5
 *   Limit switch, linear MIN    D6  (to GND when triggered, INPUT_PULLUP)
 *   Limit switch, linear MAX    D7  (to GND when triggered, INPUT_PULLUP)
 *
 * Note on limit switch pins: D6/D7 are used instead of true external
 * interrupts, via AVR pin-change interrupts (PCINT2) — see PCINT2_vect below.
 *
 * Motor 1 STEP is on D9 (Timer1's OC1A) rather than a plain digital pin.
 * ROTATE/HOME still drive it through AccelStepper's normal digitalWrite-based
 * stepping (fine for the speeds those need). SPIN instead hands the pin to
 * Timer1 hardware waveform generation: AccelStepper's polling-loop stepping
 * tops out somewhere in the low kHz on this MCU regardless of commanded RPM,
 * while Timer1 toggling OC1A in CTC mode produces a pulse train timed
 * entirely in hardware, reaching tens of kHz with no jitter and no CPU
 * involvement once at cruise speed. A software ramp (updateSpinRamp) walks
 * the timer's frequency up gradually so the motor still accelerates instead
 * of being commanded to full speed instantly; a compare-match ISR counts
 * completed pulses so position stays known for STATUS/POS. See cmdSpin,
 * timer1StartSpin/timer1StopSpin, and updateSpinRamp for the implementation.
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
const float STEPS_PER_REV        = 400.0f;
const float MOTOR1_MAX_SPEED_DEF = 300.0f;
const float MOTOR1_ACCEL_DEF     = 3000.0f;
const float MOTOR2_MAX_SPEED_DEF = 3000.0f;
const float MOTOR2_ACCEL_DEF     = 1500.0f;

// ─── Parameter validation limits ───────────────────────────────────────────
const float ROTATE_DEG_MIN   = -3600.0f;
const float ROTATE_DEG_MAX   =  3600.0f;
const long  LINEAR_STEPS_MIN = -100000L;
const long  LINEAR_STEPS_MAX =  100000L;
// SPEED_MAX no longer reflects a real safety ceiling — it's set high enough
// that 3000 RPM (the manufacturer spec, 160000 steps/s at 3200 steps/rev)
// clears validation. AccelStepper on an Uno is software-timed, though: its
// real achievable step rate tops out far below that (roughly 20-40 kHz in
// practice), so requests near the manufacturer figure are expected to be
// throughput-limited by the MCU, not rejected by this check.
const float SPEED_MAX        = 200000.0f;
const float ACCEL_MAX        = 20000.0f;

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
bool faultActive = false;     // set by a limit switch or watchdog trip; requires CLEAR to resume
unsigned long lastCommandMillis = 0;
unsigned long lastBroadcastMillis = 0;
float motor1AccelSetting = MOTOR1_ACCEL_DEF;  // mirrors motor1's AccelStepper accel so the hardware SPIN ramp can use the same value

char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t bufIndex = 0;
char lastCommandText[CMD_BUFFER_SIZE];  // raw text of the command currently in progress, echoed in OK/DONE

volatile bool limitMinTriggered = false;
volatile bool limitMaxTriggered = false;

// ─── Hardware-timer SPIN state (Timer1 drives D9/OC1A directly) ───────────
volatile long spinStepPos = 0;   // steps accumulated by the ISR while spinningHW
volatile bool spinPhase   = false;  // toggles each compare match; a full pulse completes every other one
int8_t  spinDirection  = 0;      // +1 or -1 for the duration of the current spin
bool    spinningHW     = false;  // true while Timer1 (not AccelStepper) is driving Motor 1
float   spinTargetFreq = 0;      // Hz the ramp is walking toward
float   spinCurrentFreq = 0;     // Hz currently loaded into the timer
unsigned long spinRampLastMillis = 0;

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
// Counts a completed HIGH->LOW pulse every second compare-match toggle.
ISR(TIMER1_COMPA_vect) {
  spinPhase = !spinPhase;
  if (!spinPhase) {
    spinStepPos += spinDirection;
  }
}

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
  TCCR1B = (1 << WGM12) | cs;
  OCR1A = ocr;
}

void timer1StartSpin(float freqHz) {
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR1_STEP_PIN, LOW);
  spinPhase = false;
  noInterrupts();
  spinStepPos = motor1.currentPosition();  // continue counting from wherever AccelStepper left off
  interrupts();
  TCNT1 = 0;
  TCCR1A = (1 << COM1A0);  // toggle OC1A (D9) on compare match — pure hardware waveform, no ISR needed for the pulse itself
  timer1SetFrequency(freqHz);
  TIMSK1 |= (1 << OCIE1A);  // compare-match interrupt only used for step counting
}

void timer1StopSpin() {
  TIMSK1 &= ~(1 << OCIE1A);
  TCCR1A &= ~(1 << COM1A0);  // disconnect OC1A, hand the pin back to plain digital I/O
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
  digitalWrite(MOTOR1_STEP_PIN, LOW);
  noInterrupts();
  long finalPos = spinStepPos;
  interrupts();
  motor1.setCurrentPosition(finalPos);  // keep AccelStepper's position in sync for ROTATE/HOME/STATUS afterward
}

// Returns Motor 1's logical position regardless of which path is driving it.
long motor1Position() {
  if (!spinningHW) return motor1.currentPosition();
  noInterrupts();
  long pos = spinStepPos;
  interrupts();
  return pos;
}

// Walks the timer's frequency from 0 up to spinTargetFreq at motor1AccelSetting
// (Hz/s) so SPIN still ramps like a normal move instead of jumping to full
// speed instantly. Once cruise speed is reached this does nothing further —
// steady-state SPIN is 100% hardware-timed with zero per-pulse CPU cost.
void updateSpinRamp() {
  if (spinCurrentFreq >= spinTargetFreq) return;
  unsigned long now = millis();
  float elapsedSec = (now - spinRampLastMillis) / 1000.0f;
  if (elapsedSec < 0.01f) return;  // throttle register writes to ~100Hz during the ramp
  spinRampLastMillis = now;

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
    if (spinningHW) updateSpinRamp();
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

// ─── Command dispatch ───────────────────────────────────────────────────
void processCommand(char* line) {
  char* command = strtok(line, " ");
  char* argStr  = strtok(NULL, "");

  if (command == NULL) return;

  if (strcmp(command, "STOP") == 0) {
    cmdStop();
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
    // A limit-switch or watchdog trip latches faultActive so the rig
    // cannot be moved again by accident — CLEAR is the deliberate,
    // operator-initiated way to resume, mirroring how the E-stop panel
    // itself typically needs a manual reset/twist before restart.
    faultActive = false;
    Serial.println(F("OK CLEAR"));
  } else if (strcmp(command, "ROTATE") == 0) {
    cmdRotate(argStr);
  } else if (strcmp(command, "SPIN") == 0) {
    cmdSpin(argStr);
  } else if (strcmp(command, "LINEAR") == 0) {
    cmdLinear(argStr);
  } else if (strcmp(command, "HOME") == 0) {
    cmdHome();
  } else if (strcmp(command, "SET_SPEED_1") == 0) {
    cmdSetSpeed(motor1, argStr);
  } else if (strcmp(command, "SET_SPEED_2") == 0) {
    cmdSetSpeed(motor2, argStr);
  } else if (strcmp(command, "SET_ACCEL_1") == 0) {
    cmdSetAccel(motor1, argStr);
  } else if (strcmp(command, "SET_ACCEL_2") == 0) {
    cmdSetAccel(motor2, argStr);
  } else {
    Serial.println(F("ERROR UNKNOWN_COMMAND"));
  }
}

// ─── Command handlers ───────────────────────────────────────────────────
void cmdStop() {
  // STOP always executes, even mid-fault or mid-motion — no OK stage,
  // since the halt is effectively instantaneous (see docs/commands.md).
  emergencyHalt();
  Serial.println(F("DONE STOP"));
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

void cmdSpin(char* argStr) {
  // Addition beyond the original spec: continuous RPM-driven rotation for
  // Motor 1, as an alternative to ROTATE's fixed-angle moves. Drives D9
  // directly from Timer1 hardware (see timer1StartSpin) instead of through
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
  spinRampLastMillis = millis();
  spinningHW = true;
  timer1StartSpin(spinCurrentFreq);

  busy = true;
  movingMotor1 = true;
  movingMotor2 = false;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdLinear(char* argStr) {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  long steps;
  if (!parseLongArg(argStr, &steps)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (steps < LINEAR_STEPS_MIN || steps > LINEAR_STEPS_MAX) {
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
  motor1.setSpeed(0);
  motor1.moveTo(motor1.currentPosition());
  motor2.setSpeed(0);
  motor2.moveTo(motor2.currentPosition());

  busy = false;
  movingMotor1 = false;
  movingMotor2 = false;
}

void checkLimitSwitches() {
  if (!faultActive && (limitMinTriggered || limitMaxTriggered)) {
    emergencyHalt();
    faultActive = true;
    if (limitMinTriggered) Serial.println(F("ERROR LIMIT_MIN"));
    if (limitMaxTriggered) Serial.println(F("ERROR LIMIT_MAX"));
  }
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
  if (spinningHW) return;  // hardware SPIN never completes on its own — only STOP ends it

  bool m1Done = !movingMotor1 || motor1.distanceToGo() == 0;
  bool m2Done = !movingMotor2 || motor2.distanceToGo() == 0;

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
