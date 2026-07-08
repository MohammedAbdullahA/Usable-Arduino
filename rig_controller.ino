/*
 * Rig Controller — command-driven dual stepper firmware
 * Library: AccelStepper by Mike McCauley (Arduino Library Manager)
 *
 * Testing checklist:
 * 1. Open Serial Monitor at 9600 baud, line ending "Newline". Send PING → expect PONG.
 * 2. Send STATUS → expect a STATE line.
 * 3. Send ROTATE 90 → motor turns, STATE lines stream every 250 ms, then DONE ROTATE.
 * 4. Send MOVE 30000 then STOP mid-move → motor decelerates and halts, DONE MOVE.
 * 5. Send EHALT mid-move → motors stop hard. ROTATE 90 → ERR ehalt. RESET → OK RESET. Motion works again.
 * 6. Stop sending PING for >2 s while connected → LOG watchdog timeout and EHALT.
 * 7. AUTO runs full 8-phase cycle (900 ms handoff after each rotation). MANUAL mid-cycle stops after current motion completes.
 */

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>

// ─── Pin Configuration ───────────────────────────────────────────────────────
constexpr uint8_t MOTOR1_STEP_PIN = 2;
constexpr uint8_t MOTOR1_DIR_PIN  = 3;
constexpr uint8_t MOTOR2_STEP_PIN = 4;
constexpr uint8_t MOTOR2_DIR_PIN  = 5;

// ─── Motor 1 (Rotation) Settings ─────────────────────────────────────────────
constexpr float STEPS_PER_REV          = 1600.0f;
constexpr float MOTOR1_MAX_SPEED       = 300.0f;
constexpr float MOTOR1_ACCELERATION    = 120.0f;
constexpr long  STEPS_PER_ROTATION     = (long)(STEPS_PER_REV * 90.0f / 360.0f);

// ─── Motor 2 (Linear) Settings ───────────────────────────────────────────────
constexpr long  LINEAR_MOVE_STEPS      = 30000;
constexpr float MOTOR2_MAX_SPEED       = 3000.0f;
constexpr float MOTOR2_ACCELERATION    = 1500.0f;

// ─── Timing / Protocol ───────────────────────────────────────────────────────
constexpr unsigned long DELAY_BEFORE_LINEAR_MS = 900;
constexpr unsigned long STATE_INTERVAL_MS      = 250;
constexpr unsigned long WATCHDOG_TIMEOUT_MS    = 2000;
constexpr size_t SERIAL_BUF_SIZE               = 64;

enum Mode : uint8_t { MODE_MANUAL, MODE_AUTO };

enum PendingDone : uint8_t { DONE_NONE = 0, DONE_ROTATE, DONE_MOVE };

struct RigState {
  Mode mode;
  bool ehalt;
  bool watchdogArmed;
  unsigned long lastPingMs;
  unsigned long lastStateEmitMs;
  uint8_t autoPhase;
  unsigned long phaseTimerMs;
  bool phaseTimerActive;
  bool autoPhaseStarted;
  bool autoWaitingTimer;
  bool autoPaused;
  bool wasMoving1;
  bool wasMoving2;
  PendingDone pendingDone1;
  PendingDone pendingDone2;
};

AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

RigState rig = {
  MODE_MANUAL, false, false, 0, 0,
  0, 0, false, false, false, false, false, false,
  DONE_NONE, DONE_NONE
};

char serialBuf[SERIAL_BUF_SIZE + 1];
uint8_t serialLen = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

long degToSteps(float deg) {
  return (long)(deg * STEPS_PER_REV / 360.0f);
}

bool isMoving1() {
  return motor1.distanceToGo() != 0;
}

bool isMoving2() {
  return motor2.distanceToGo() != 0;
}

bool isAutoRotatePhase(uint8_t phase) {
  return phase == 0 || phase == 2 || phase == 4 || phase == 6;
}

void replyOk(const char* cmd) {
  Serial.print(F("OK "));
  Serial.println(cmd);
}

void replyErr(const char* reason) {
  Serial.print(F("ERR "));
  Serial.println(reason);
}

void replyDone(const char* cmd) {
  Serial.print(F("DONE "));
  Serial.println(cmd);
}

void emitLog(const char* msg) {
  Serial.print(F("LOG "));
  Serial.println(msg);
}

void emitState() {
  Serial.print(F("STATE "));
  Serial.print(rig.mode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
  Serial.print(F(" POS1:"));
  Serial.print(motor1.currentPosition());
  Serial.print(F(" POS2:"));
  Serial.print(motor2.currentPosition());
  Serial.print(F(" MOVING1:"));
  Serial.print(isMoving1() ? 1 : 0);
  Serial.print(F(" MOVING2:"));
  Serial.print(isMoving2() ? 1 : 0);
  Serial.print(F(" EHALT:"));
  Serial.println(rig.ehalt ? 1 : 0);
  rig.lastStateEmitMs = millis();
}

void cancelAutoTimer() {
  rig.phaseTimerActive = false;
  rig.autoWaitingTimer = false;
}

void resetAutoPhaseFlags() {
  rig.autoPhaseStarted = false;
  rig.autoWaitingTimer = false;
  cancelAutoTimer();
}

void doEhalt() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor1.setCurrentPosition(motor1.currentPosition());
  motor2.setCurrentPosition(motor2.currentPosition());
  rig.ehalt = true;
  resetAutoPhaseFlags();
}

void emitAutoPhaseLog(uint8_t phase) {
  Serial.print(F("LOG auto phase "));
  Serial.println(phase);
}

void startAutoPhaseAction() {
  const uint8_t p = rig.autoPhase;

  if (isAutoRotatePhase(p)) {
    motor1.move(STEPS_PER_ROTATION);
  } else if (p == 1 || p == 5) {
    motor2.move(LINEAR_MOVE_STEPS);
  } else {
    motor2.move(-LINEAR_MOVE_STEPS);
  }

  rig.autoPhaseStarted = true;
  rig.autoWaitingTimer = false;
  cancelAutoTimer();
}

void advanceAutoPhase() {
  emitAutoPhaseLog(rig.autoPhase);
  rig.autoPhase = (rig.autoPhase + 1) % 8;
  resetAutoPhaseFlags();
  startAutoPhaseAction();
}

void serviceAuto() {
  if (rig.mode != MODE_AUTO || rig.ehalt || rig.autoPaused) {
    return;
  }

  if (!rig.autoPhaseStarted) {
    startAutoPhaseAction();
    return;
  }

  const uint8_t p = rig.autoPhase;

  if (isAutoRotatePhase(p)) {
    if (isMoving1()) {
      return;
    }

    if (!rig.autoWaitingTimer) {
      rig.phaseTimerMs = millis() + DELAY_BEFORE_LINEAR_MS;
      rig.autoWaitingTimer = true;
      rig.phaseTimerActive = true;
      return;
    }

    if (millis() < rig.phaseTimerMs) {
      return;
    }

    advanceAutoPhase();
    return;
  }

  if (isMoving2()) {
    return;
  }

  advanceAutoPhase();
}

void handleMotionCompletion() {
  const bool moving1 = isMoving1();
  const bool moving2 = isMoving2();

  if (rig.wasMoving1 && !moving1) {
    if (rig.mode == MODE_MANUAL && rig.pendingDone1 == DONE_ROTATE) {
      replyDone("ROTATE");
      rig.pendingDone1 = DONE_NONE;
    }
    emitState();
  }

  if (rig.wasMoving2 && !moving2) {
    if (rig.mode == MODE_MANUAL && rig.pendingDone2 == DONE_MOVE) {
      replyDone("MOVE");
      rig.pendingDone2 = DONE_NONE;
    }
    emitState();
  }

  rig.wasMoving1 = moving1;
  rig.wasMoving2 = moving2;
}

void servicePeriodicState() {
  const bool anyMoving = isMoving1() || isMoving2();

  if (!anyMoving) {
    return;
  }

  if (millis() - rig.lastStateEmitMs >= STATE_INTERVAL_MS) {
    emitState();
  }
}

void serviceWatchdog() {
  if (!rig.watchdogArmed || rig.ehalt) {
    return;
  }

  if (millis() - rig.lastPingMs > WATCHDOG_TIMEOUT_MS) {
    doEhalt();
    emitLog("watchdog timeout");
  }
}

void processLine(char* line) {
  char* cmd = strtok(line, " ");
  if (cmd == NULL || cmd[0] == '\0') {
    replyErr("empty");
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    rig.lastPingMs = millis();
    if (!rig.watchdogArmed) {
      rig.watchdogArmed = true;
    }
    Serial.println(F("PONG"));
    return;
  }

  if (strcmp(cmd, "STATUS") == 0) {
    emitState();
    return;
  }

  if (strcmp(cmd, "RESET") == 0) {
    rig.ehalt = false;
    replyOk("RESET");
    return;
  }

  if (strcmp(cmd, "EHALT") == 0) {
    doEhalt();
    replyOk("EHALT");
    emitState();
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    motor1.stop();
    motor2.stop();
    resetAutoPhaseFlags();
    if (rig.mode == MODE_AUTO) {
      rig.autoPaused = true;
    }
    replyOk("STOP");
    return;
  }

  if (strcmp(cmd, "SETHOME") == 0) {
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    replyOk("SETHOME");
    emitState();
    return;
  }

  if (strcmp(cmd, "MANUAL") == 0) {
    rig.mode = MODE_MANUAL;
    rig.autoPaused = false;
    resetAutoPhaseFlags();
    replyOk("MANUAL");
    return;
  }

  if (rig.ehalt) {
    replyErr("ehalt");
    return;
  }

  if (strcmp(cmd, "ROTATE") == 0) {
    if (rig.mode == MODE_AUTO) {
      replyErr("auto");
      return;
    }

    char* arg = strtok(NULL, " ");
    if (arg == NULL) {
      replyErr("syntax");
      return;
    }

    const long steps = degToSteps((float)atof(arg));
    motor1.move(steps);
    rig.pendingDone1 = DONE_ROTATE;
    replyOk("ROTATE");
    return;
  }

  if (strcmp(cmd, "MOVE") == 0) {
    if (rig.mode == MODE_AUTO) {
      replyErr("auto");
      return;
    }

    char* arg = strtok(NULL, " ");
    if (arg == NULL) {
      replyErr("syntax");
      return;
    }

    const long steps = atol(arg);
    motor2.move(steps);
    rig.pendingDone2 = DONE_MOVE;
    replyOk("MOVE");
    return;
  }

  if (strcmp(cmd, "HOME") == 0) {
    motor1.moveTo(0);
    motor2.moveTo(0);
    rig.pendingDone1 = DONE_NONE;
    rig.pendingDone2 = DONE_NONE;
    replyOk("HOME");
    return;
  }

  if (strcmp(cmd, "AUTO") == 0) {
    rig.mode = MODE_AUTO;
    rig.autoPhase = 0;
    rig.autoPaused = false;
    resetAutoPhaseFlags();
    replyOk("AUTO");
    return;
  }

  replyErr("unknown");
}

void handleSerialByte(char c) {
  if (c == '\n') {
    serialBuf[serialLen] = '\0';

    if (serialLen > 0 && serialBuf[serialLen - 1] == '\r') {
      serialBuf[serialLen - 1] = '\0';
      serialLen--;
    }

    if (serialLen > 0) {
      processLine(serialBuf);
    }

    serialLen = 0;
    return;
  }

  if (c == '\r') {
    return;
  }

  if (serialLen >= SERIAL_BUF_SIZE) {
    replyErr("overflow");
    serialLen = 0;
    return;
  }

  serialBuf[serialLen++] = c;
}

void readSerial() {
  while (Serial.available() > 0) {
    handleSerialByte((char)Serial.read());
  }
}

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000) {
  }

  motor1.setMaxSpeed(MOTOR1_MAX_SPEED);
  motor1.setAcceleration(MOTOR1_ACCELERATION);
  motor1.setCurrentPosition(0);

  motor2.setMaxSpeed(MOTOR2_MAX_SPEED);
  motor2.setAcceleration(MOTOR2_ACCELERATION);
  motor2.setCurrentPosition(0);

  rig.wasMoving1 = false;
  rig.wasMoving2 = false;

  emitLog("rig controller ready");
}

void loop() {
  readSerial();

  motor1.run();
  motor2.run();

  serviceWatchdog();
  serviceAuto();
  handleMotionCompletion();
  servicePeriodicState();
}
