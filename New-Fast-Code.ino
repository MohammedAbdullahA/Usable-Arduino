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
 *   Motor 2 (rotary — spins the product) step=D11  dir=D5
 *     Controlled via ROTARY (fixed steps), HOME, PROFILE_WT, and
 *     PROFILE_UT. Runs entirely on Timer2 hardware (see below) — no
 *     AccelStepper involvement at all, for the same reason as Motor 1's
 *     SPIN/MOVE_TO: AccelStepper's software polling loop couldn't sustain
 *     real commanded speed. STEP was originally D4 (a plain digital pin,
 *     no hardware timer available there) — moved to D11 (Timer2's OC2A)
 *     specifically to enable this.
 *
 * Two preset profiles run on top of the free-control commands above:
 *   PROFILE_WT <rpm>: Motor 2 does 4x (rotate 90° at the given RPM, wait
 *     8s) = one full turn. The RPM is entered once and reused for all
 *     four quarter-turns.
 *   PROFILE_UT <rotary_rpm> <linear_rpm>: Motor 2 spins continuously at
 *     rotary_rpm while Motor 1 drives toward D6/MIN at linear_rpm — both
 *     entered directly, independently, with no ratio enforced between
 *     them (the industry-standard 5mm-overlap ratio, if wanted, is the
 *     operator's responsibility to set via the two values). Both stop
 *     together the instant D6 triggers.
 *   Both are abortable by STOP/ESTOP like any other motion. See cmdProfileWT,
 *   updateProfileWT, and cmdProfileUT.
 *   Limit switch, Motor 1 track MIN (home) D6  (to GND when triggered, INPUT_PULLUP)
 *   Limit switch, Motor 1 track MAX (far end) D7  (to GND when triggered, INPUT_PULLUP)
 *     Normal travel boundaries, not faults — see checkLimitSwitches. MAX
 *     auto-reverses the jig back toward home at half speed; MIN just stops.
 *
 * Note on limit switch pins: D6/D7 are used instead of true external
 * interrupts, via AVR pin-change interrupts (PCINT2) — see PCINT2_vect below.
 *
 * Motor 1 STEP is on D9 (Timer1's OC1A) rather than a plain digital pin.
 * ROTATE/HOME's motor 1 leg still drive it through AccelStepper's normal
 * digitalWrite-based stepping (fine for the speeds those need). SPIN and
 * MOVE_TO instead hand the pin to Timer1 hardware waveform generation:
 * AccelStepper's polling-loop stepping tops out somewhere in the low kHz
 * on this MCU regardless of commanded RPM, while Timer1 toggling OC1A in
 * CTC mode produces a pulse train timed entirely in hardware, reaching
 * tens of kHz with no jitter and no CPU involvement once at cruise speed.
 * Motor 2 gets the identical treatment on Timer2/OC2A (D11) — see
 * updateSpin2Ramp, timer2StartSpin/timer2StopSpin — since it hit the exact
 * same polling-loop ceiling once real speed was needed (ROTARY, PROFILE_WT,
 * PROFILE_UT). Timer2 is 8-bit (max OCR2A=255, vs Timer1's 65535) with its
 * own prescaler set, so its frequency resolution is coarser — most
 * noticeable at the very bottom of a ramp — but it's still a genuine
 * hardware-timed pulse train at cruise, same as Timer1.
 *
 * A software ramp (updateSpinRamp / updateSpin2Ramp) walks each timer's
 * frequency up gradually so the motor still accelerates instead of being
 * commanded to full speed instantly. For SPIN/PROFILE_UT this just ramps up
 * and holds; for MOVE_TO/ROTARY/HOME/PROFILE_WT, the same ramp instead
 * recomputes the reachable speed every tick from remaining distance (a
 * standard distance-based trapezoidal profile — see the moveToActive /
 * moveTo2Active branches), so it accelerates, cruises if there's room, and
 * decelerates to land exactly on target — all at real achievable RPM,
 * unlike plain ROTATE. Position is integrated in software from frequency x
 * elapsed time (updateSpinPosition / updateSpin2Position): at high
 * microstepping the pulse rate reaches tens of kHz, so a per-pulse counting
 * ISR would saturate the CPU — neither timer uses one. See cmdSpin,
 * cmdMoveTo, cmdRotary, startMoveTo2, timer1StartSpin/timer1StopSpin,
 * timer2StartSpin/timer2StopSpin, updateSpinRamp/updateSpin2Ramp, and
 * updateSpinPosition/updateSpin2Position.
 *
 * STOP decelerates Motor 1 to zero rather than cutting instantly (an
 * instant cut at high RPM regenerates a damaging bus-voltage spike — see
 * cmdStop). Its rate (spinStopDecelSetting, SET_STOP_DECEL_1) is deliberately
 * separate from and much faster than the normal accel setting: a decel at
 * accel's rate can take ~1s and cover enough real rotation/travel to
 * overshoot a mechanical limit on a mounted rig. Motor 2 (a much smaller
 * motor/driver with no comparable failure history) uses a simpler symmetric
 * ramp — same rate up and down (motor2AccelSetting) — and if it's running
 * alongside Motor 1 during PROFILE_UT, STOP cuts it instantly rather than
 * ramping it, to avoid two independent ramps finishing at different times.
 * ESTOP remains an instant cut on both motors for genuine emergencies.
 */

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ─── Pins ────────────────────────────────────────────────────────────────
const uint8_t MOTOR1_STEP_PIN = 9;  // Timer1 OC1A — required for hardware-driven SPIN
const uint8_t MOTOR1_DIR_PIN  = 3;
const uint8_t MOTOR2_STEP_PIN = 11;  // Timer2's OC2A — moved from D4; D4 has no timer hardware, needed for real achievable speed (see header)
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
// motor2 (AccelStepper) removed — Motor 2 now runs entirely on its own
// Timer2 hardware path (see below), same reasoning as Motor 1's Timer1:
// AccelStepper's software polling loop couldn't reach real commanded speed.

// ─── State ──────────────────────────────────────────────────────────────
bool busy = false;            // a motion command is in progress (global — the rig moves one axis at a time, matching the original sequence)
bool movingMotor1 = false;
bool movingMotor2 = false;
bool faultActive = false;     // set by a watchdog trip only; requires CLEAR to resume. Limit switches (D6/D7) are normal travel boundaries, handled entirely in checkLimitSwitches — they never set this.

// ─── Profiles: preset sequences layered on top of the free-control commands.
// PROFILE_UT: Motor 2 spins continuously at its configured max speed while
// Motor 1 drives in the negative direction until it hits D6/MIN, at which
// point both stop together (reusing the existing MIN-trigger halt).
// PROFILE_WT: Motor 2 does 4x (rotate 90°, wait 8s) = one full 360°
// revolution in quarter-turn steps, then stops on its own.
bool profileUTActive  = false;   // true only for nicer DONE messaging when MIN ends a UT run
bool profileWTActive  = false;
bool profileWTWaiting = false;   // false = currently turning this quarter, true = currently in the 8s pause
int  profileWTStep    = 0;       // 0..PROFILE_WT_REPS-1
unsigned long profileWTWaitStartMillis = 0;
float profileWTSpeed = 0;  // Hz, user-entered RPM converted once at start — reused by each subsequent quarter-turn
const int PROFILE_WT_REPS            = 4;
const float PROFILE_WT_DEG           = 90.0f;
const unsigned long PROFILE_WT_WAIT_MS = 8000;
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
//
// Time alone isn't quite enough, though: if the jig ramps up slowly, 100ms
// of "released" might elapse while it's still only a hair's-breadth off the
// switch — vibration at that range can still tip it back on, producing the
// "stops again right as it starts moving forward" symptom. So re-arming
// also requires the tracked position to have moved at least
// LIMIT_REARM_MIN_STEPS away from wherever the switch was last triggered —
// real physical clearance, not just an elapsed-time guess.
const unsigned long LIMIT_REARM_STABLE_MS = 100;
const long LIMIT_REARM_MIN_STEPS = 400;  // ~0.25 rev at 1600 steps/rev
bool limitMinArmed = true;
bool limitMaxArmed = true;
unsigned long limitMinHighSinceMillis = 0;
unsigned long limitMaxHighSinceMillis = 0;
long limitMinTriggerPos = 0;  // motor1Position() at the moment MIN was last handled — used to require real clearance before re-arming
long limitMaxTriggerPos = 0;

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

// ─── Motor 2 — same architecture as Motor 1 above, mirrored onto Timer2 ───
long  spin2StepPos   = 0;
float spin2StepFrac  = 0;
unsigned long spin2PosLastMicros = 0;
int8_t  spin2Direction  = 0;
bool    spinningHW2       = false;
bool    spin2StoppingSoft = false;
float   spin2TargetFreq  = 0;
float   spin2CurrentFreq = 0;
unsigned long spin2RampLastMillis = 0;

bool    moveTo2Active        = false;
long    moveTo2StartPos      = 0;
long    moveTo2TargetDistance = 0;
float   spin2CruiseFreq      = 0;

float motor2Speed        = MOTOR2_MAX_SPEED_DEF;  // replaces AccelStepper's setMaxSpeed for Motor 2
float motor2AccelSetting = MOTOR2_ACCEL_DEF;      // replaces setAcceleration; used for both ramp-up and STOP ramp-down (Motor 2 has no documented regen-spike history like Motor 1, so a symmetric rate is a reasonable, simpler default)

// A moveTo2Active move is shared by three different callers (ROTARY, HOME,
// and PROFILE_WT's quarter-turns), each needing different behavior on
// completion — this tag tells updateSpin2Ramp which. 0=none, 1=ROTARY
// (end the command directly), 2=HOME (let checkHomeCompletion finish once
// both motors are done), 3=WT_STEP (just stop — updateProfileWT notices
// and decides what's next).
int motor2MoveKind = 0;
bool homeInProgress = false;  // true while cmdHome is waiting on both motors together

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

// ─── Timer2 hardware pulse generator for Motor 2 ──────────────────────────
// Same approach as Timer1 above, on the only other general-purpose timer
// free on this MCU (Timer0 runs millis()/micros()/delay() — never touch
// it). Timer2 is 8-bit (max OCR2A = 255, vs Timer1's 65535) with its own
// prescaler set — coarser frequency resolution, most noticeable at the
// very bottom of a ramp, but it's still a genuine hardware-timed pulse
// train reaching tens of kHz at cruise, exactly like Timer1 — no more
// AccelStepper polling-loop ceiling for Motor 2.
bool computeTimer2Config(float freqHz, uint8_t& ocrOut, uint8_t& csBitsOut) {
  static const uint16_t PRESCALERS[] = {1, 8, 32, 64, 128, 256, 1024};
  static const uint8_t  CS_BITS[]    = {
    (1 << CS20),
    (1 << CS21),
    (1 << CS21) | (1 << CS20),
    (1 << CS22),
    (1 << CS22) | (1 << CS20),
    (1 << CS22) | (1 << CS21),
    (1 << CS22) | (1 << CS21) | (1 << CS20)
  };
  for (uint8_t i = 0; i < 7; i++) {
    long ocr = lround(F_CPU / (2.0 * PRESCALERS[i] * freqHz)) - 1;
    if (ocr >= 1 && ocr <= 255L) {
      ocrOut = (uint8_t)ocr;
      csBitsOut = CS_BITS[i];
      return true;
    }
  }
  return false;
}

void timer2SetFrequency(float freqHz) {
  uint8_t ocr, cs;
  if (!computeTimer2Config(freqHz, ocr, cs)) return;  // out of representable range, leave timer as-is
  noInterrupts();
  TCCR2B = cs;  // WGM22 stays 0 — CTC-via-OCR2A doesn't need it
  OCR2A = ocr;
  if (TCNT2 > ocr) TCNT2 = 0;  // same wrap-through-255 dead-gap guard as Timer1's TCNT1 fix
  interrupts();
}

void timer2StartSpin(float freqHz) {
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR2_STEP_PIN, LOW);
  spin2StepFrac = 0;
  spin2PosLastMicros = micros();
  TCNT2 = 0;
  TCCR2A = (1 << COM2A0) | (1 << WGM21);  // toggle OC2A (D11) in CTC mode on compare match
  timer2SetFrequency(freqHz);
}

void timer2StopSpin() {
  TCCR2A &= ~(1 << COM2A0);  // disconnect OC2A, hand the pin back to plain digital I/O
  TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
  digitalWrite(MOTOR2_STEP_PIN, LOW);
  updateSpin2Position(true);  // flush the last partial integration interval
}

// Motor 2's position — always spin2StepPos now, since it never uses
// AccelStepper's own counter (unlike motor1Position(), which still has to
// choose between Timer1 and AccelStepper depending on what's running).
long motor2Position() {
  return spin2StepPos;
}

void updateSpin2Position(bool force) {
  unsigned long now = micros();
  float dt = (now - spin2PosLastMicros) / 1.0e6f;
  if (!force && dt < 0.005f) return;
  spin2PosLastMicros = now;
  spin2StepFrac += spin2CurrentFreq * dt;
  long whole = (long)spin2StepFrac;
  spin2StepFrac -= whole;
  spin2StepPos += spin2Direction * whole;
}

// Launches a Timer2 position-controlled move on Motor 2: signed relative
// `steps`, target cruise speed `cruiseFreq`, and a `kind` tag telling
// updateSpin2Ramp what to do on completion (see motor2MoveKind above).
// Shared by ROTARY, HOME's motor 2 leg, and PROFILE_WT's quarter-turns.
void startMoveTo2(long steps, float cruiseFreq, int kind) {
  motor2MoveKind = kind;
  spin2Direction = (steps >= 0) ? 1 : -1;
  digitalWrite(MOTOR2_DIR_PIN, steps >= 0 ? HIGH : LOW);
  spin2CruiseFreq = cruiseFreq;
  spin2CurrentFreq = 1.0f;
  spin2StoppingSoft = false;
  spin2RampLastMillis = millis();
  spinningHW2 = true;
  timer2StartSpin(spin2CurrentFreq);
  moveTo2Active = true;
  moveTo2StartPos = spin2StepPos;
  moveTo2TargetDistance = labs(steps);
}

// Drives the Timer2 frequency each tick — same three-mode structure as
// updateSpinRamp (see there for the full rationale), mirrored for Motor 2.
void updateSpin2Ramp() {
  unsigned long now = millis();
  float elapsedSec = (now - spin2RampLastMillis) / 1000.0f;
  if (elapsedSec < 0.002f) return;
  spin2RampLastMillis = now;

  if (spin2StoppingSoft) {
    float freq = spin2CurrentFreq - motor2AccelSetting * elapsedSec;
    if (freq < 0) freq = 0;
    spin2CurrentFreq = freq;
    if (freq <= 30.0f) {
      timer2StopSpin();
      spinningHW2 = false;
      spin2StoppingSoft = false;
      moveTo2Active = false;
      motor2MoveKind = 0;
      busy = false;
      movingMotor2 = false;
      // If a WT sequence was mid-step, STOP aborts the whole thing, not
      // just this quarter-turn.
      profileWTActive = false;
      profileWTWaiting = false;
      Serial.println(F("DONE STOP"));
      return;
    }
    timer2SetFrequency(freq);
    return;
  }

  if (moveTo2Active) {
    updateSpin2Position(false);
    long traveled = labs(spin2StepPos - moveTo2StartPos);
    long remaining = moveTo2TargetDistance - traveled;

    if (remaining <= 0) {
      timer2StopSpin();
      spinningHW2 = false;
      moveTo2Active = false;
      int kind = motor2MoveKind;
      motor2MoveKind = 0;
      if (kind == 1) {  // ROTARY — ends the command directly
        busy = false;
        movingMotor2 = false;
        Serial.print(F("DONE "));
        Serial.println(lastCommandText);
      } else if (kind == 2) {  // HOME — this leg is done; checkHomeCompletion() finishes once motor1's leg is too
        movingMotor2 = false;
      }
      // kind == 3 (WT_STEP): just stop here — updateProfileWT() notices
      // spinningHW2/moveTo2Active went false and decides what's next.
      return;
    }

    float maxReachable = sqrt(2.0f * motor2AccelSetting * (float)remaining);
    float effectiveTarget = (maxReachable < spin2CruiseFreq) ? maxReachable : spin2CruiseFreq;

    float freq;
    if (spin2CurrentFreq < effectiveTarget) {
      freq = spin2CurrentFreq + motor2AccelSetting * elapsedSec;
      if (freq > effectiveTarget) freq = effectiveTarget;
    } else {
      freq = spin2CurrentFreq - motor2AccelSetting * elapsedSec;
      if (freq < effectiveTarget) freq = effectiveTarget;
    }
    if (freq < 1.0f) freq = 1.0f;
    spin2CurrentFreq = freq;
    timer2SetFrequency(freq);
    return;
  }

  // Plain continuous hold (PROFILE_UT): ramp toward spin2TargetFreq and
  // hold — no auto-decel, STOP (the branch above) handles that.
  if (spin2CurrentFreq == spin2TargetFreq) return;
  float freq = spin2CurrentFreq + motor2AccelSetting * elapsedSec;
  if (freq > spin2TargetFreq) freq = spin2TargetFreq;
  spin2CurrentFreq = freq;
  timer2SetFrequency(freq);
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
      // cmdStop() already stopped Motor 2 immediately (if PROFILE_UT had it
      // running alongside) before this ramp even started — just pick the
      // right final message.
      bool wasUT = profileUTActive;
      profileUTActive = false;
      Serial.println(wasUT ? F("DONE PROFILE_UT") : F("DONE STOP"));
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

  // Motor 2 has no AccelStepper object anymore — motor2Speed/
  // motor2AccelSetting already default correctly at declaration. Just
  // need the pins explicitly configured once (AccelStepper's constructor
  // used to do this for us).
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR2_STEP_PIN, LOW);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR2_DIR_PIN, LOW);

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

    // Motor 2 has no AccelStepper path left at all — every kind of move
    // (ROTARY, HOME's leg, PROFILE_WT's steps, PROFILE_UT's continuous
    // spin) runs through Timer2 via updateSpin2Ramp.
    if (spinningHW2) { updateSpin2Ramp(); updateSpin2Position(false); }

    updateProfileWT();
  }

  handleMotionCompletion();
  checkHomeCompletion();
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
  } else if (strcmp(command, "PROFILE_UT") == 0) {
    cmdProfileUT(argStr);
  } else if (strcmp(command, "PROFILE_WT") == 0) {
    cmdProfileWT(argStr);
  } else if (strcmp(command, "HOME") == 0) {
    cmdHome();
  } else if (strcmp(command, "SET_SPEED_1") == 0) {
    cmdSetSpeed(motor1, argStr);
  } else if (strcmp(command, "SET_SPEED_2") == 0) {
    cmdSetSpeed2(argStr);
  } else if (strcmp(command, "SET_ACCEL_1") == 0) {
    cmdSetAccel(motor1, argStr);
  } else if (strcmp(command, "SET_STOP_DECEL_1") == 0) {
    cmdSetStopDecel(argStr);
  } else if (strcmp(command, "SET_ACCEL_2") == 0) {
    cmdSetAccel2(argStr);
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
    // If Motor 2 is also running (PROFILE_UT), stop it immediately right
    // here rather than ramping it down too: it's a much smaller motor/
    // driver without Motor 1's documented regen-spike failure history, so
    // an instant cut is a reasonable, simpler trade-off — and it avoids
    // two independent ramps finishing at different times and each trying
    // to print its own completion message.
    if (spinningHW2) {
      timer2StopSpin();
      spinningHW2 = false;
      moveTo2Active = false;
      motor2MoveKind = 0;
      movingMotor2 = false;
    }
    spinTargetFreq = 0;
    spinStoppingSoft = true;
    spinRampLastMillis = millis();
    Serial.println(F("OK STOP"));
    return;
  }
  if (spinningHW2 && !faultActive) {
    // Motor 2 running alone (ROTARY/PROFILE_WT step) — same soft-ramp
    // treatment via its own accel rate, for mechanical smoothness.
    spin2TargetFreq = 0;
    spin2StoppingSoft = true;
    spin2RampLastMillis = millis();
    Serial.println(F("OK STOP"));
    return;
  }
  // Neither hardware path running (or mid-fault): slow AccelStepper moves
  // carry negligible stored energy, so an immediate halt is safe and simplest.
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
  if (steps == 0) {
    Serial.println(F("ERROR OUT_OF_RANGE"));  // nothing to move
    return;
  }

  startMoveTo2(steps, motor2Speed, 1 /* ROTARY */);
  busy = true;
  movingMotor1 = false;
  movingMotor2 = true;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdHome() {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  bool m1AtZero = (motor1Position() == 0);
  bool m2AtZero = (motor2Position() == 0);

  if (m1AtZero && m2AtZero) {
    Serial.println(F("DONE HOME"));
    return;
  }

  movingMotor1 = !m1AtZero;
  movingMotor2 = !m2AtZero;

  if (!m1AtZero) motor1.moveTo(0);  // motor1's HOME leg is unchanged — still fine on AccelStepper at ROTATE-class speeds
  if (!m2AtZero) startMoveTo2(-motor2Position(), motor2Speed, 2 /* HOME */);

  homeInProgress = true;
  busy = true;

  Serial.println(F("OK HOME"));
}

// Waits for both HOME legs to finish (motor1 via handleMotionCompletion's
// existing AccelStepper check, motor2 via its own Timer2 path clearing
// movingMotor2 when its moveTo2Active move lands — see updateSpin2Ramp's
// kind==2 case) before declaring the whole command done.
void checkHomeCompletion() {
  if (!homeInProgress) return;
  bool m1Done = !movingMotor1 || motor1.distanceToGo() == 0;
  bool m2Done = !movingMotor2;
  if (m1Done && m2Done) {
    homeInProgress = false;
    busy = false;
    movingMotor1 = false;
    Serial.println(F("DONE HOME"));
  }
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

void cmdSetSpeed2(char* argStr) {
  // Motor 2's plain-variable equivalent of cmdSetSpeed — no AccelStepper
  // object left to call setMaxSpeed() on.
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

  motor2Speed = value;
  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

void cmdSetAccel2(char* argStr) {
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

  motor2AccelSetting = value;
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
  if (spinningHW2) {
    timer2StopSpin();
    spinningHW2 = false;
  }
  spinStoppingSoft = false;
  spin2StoppingSoft = false;
  moveToActive = false;   // defensive: guarantees a later plain SPIN doesn't inherit a stale MOVE_TO target
  moveTo2Active = false;
  motor2MoveKind = 0;
  profileUTActive = false;
  profileWTActive = false;   // abort any in-progress PROFILE_WT sequence
  profileWTWaiting = false;
  homeInProgress = false;
  motor1.setSpeed(0);
  motor1.moveTo(motor1.currentPosition());
  // Motor 2 has no AccelStepper object left to reset — timer2StopSpin()
  // above already halted its pulse train.

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
// the pin reads reliably released for LIMIT_REARM_STABLE_MS *and* the jig
// has moved at least LIMIT_REARM_MIN_STEPS away from the trigger point —
// time alone can elapse before there's real physical clearance, especially
// during a slow ramp-up, letting stray vibration re-trigger a halt right as
// it starts departing.
void checkLimitSwitches() {
  unsigned long now = millis();

  if (!limitMinArmed) {
    bool released = (digitalRead(LIMIT_MIN_PIN) == HIGH);
    bool clearedDistance = labs(motor1Position() - limitMinTriggerPos) >= LIMIT_REARM_MIN_STEPS;
    if (released && clearedDistance) {
      if (limitMinHighSinceMillis == 0) limitMinHighSinceMillis = now;
      if (now - limitMinHighSinceMillis >= LIMIT_REARM_STABLE_MS) limitMinArmed = true;
    } else {
      limitMinHighSinceMillis = 0;  // not yet both released and clear — reset the stable-timer
    }
  }
  if (!limitMaxArmed) {
    bool released = (digitalRead(LIMIT_MAX_PIN) == HIGH);
    bool clearedDistance = labs(motor1Position() - limitMaxTriggerPos) >= LIMIT_REARM_MIN_STEPS;
    if (released && clearedDistance) {
      if (limitMaxHighSinceMillis == 0) limitMaxHighSinceMillis = now;
      if (now - limitMaxHighSinceMillis >= LIMIT_REARM_STABLE_MS) limitMaxArmed = true;
    } else {
      limitMaxHighSinceMillis = 0;
    }
  }

  if (limitMaxTriggered) {
    limitMaxTriggered = false;  // consume now — the ISR only sets it, never clears it
    if (limitMaxArmed) {
      limitMaxArmed = false;  // one-shot: ignore this switch until confirmed released and clear
      limitMaxTriggerPos = motor1Position();  // reference point for the distance-based re-arm check
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
      limitMinTriggerPos = motor1Position();  // reference point for the distance-based re-arm check
      bool wasUT = profileUTActive;  // capture before emergencyHalt() clears it
      emergencyHalt();
      Serial.println(wasUT ? F("DONE PROFILE_UT") : F("DONE LIMIT_MIN_HOME"));
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

// PROFILE_UT — Motor 2 spins continuously at its configured max speed
// (Timer2 hardware path, no target) while Motor 1 drives in the negative
// direction until it hits D6/MIN, at a speed locked to exactly 2x Motor
// 2's — the industry-standard overlap ratio (1 rotary rotation per 2
// linear rotations, i.e. one wrap every 5mm of linear travel). Since both
// motors share the same 1600 steps/rev, a 2x *step-rate* ratio is exactly
// a 2x *rotation-rate* ratio. This ratio is now genuinely accurate at
// runtime, not just commanded — Motor 2 can actually reach motor2Speed on
// its own hardware timer, unlike the old AccelStepper polling loop. Both
// motors stop together at D6: checkLimitSwitches' existing MIN handling
// calls emergencyHalt() regardless of what's running, and emergencyHalt()
// stops both timers — no extra completion logic needed here beyond
// starting the two motions.
// PROFILE_UT <rotary_rpm> <linear_rpm> — Motor 2 spins continuously at the
// given rotary RPM (Timer2, no target) while Motor 1 drives in the
// negative direction at the given linear RPM (Timer1, no target) until it
// hits D6/MIN, at which point both stop together. Both speeds are entered
// directly and used as-is — no automatic ratio is enforced between them
// (an earlier version locked Motor 1 to exactly 2x Motor 2 for the
// industry-standard 5mm-overlap ratio; that's now the operator's call —
// enter matching values yourself if you need that ratio preserved).
void cmdProfileUT(char* argStr) {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  float rotaryRpm, linearRpm;
  if (!parseTwoFloatArgs(argStr, &rotaryRpm, &linearRpm)) {
    Serial.println(F("ERROR INVALID_PARAM"));
    return;
  }
  if (rotaryRpm <= 0 || linearRpm <= 0) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }
  float rotaryFreq = rotaryRpm * STEPS_PER_REV / 60.0f;
  float linearFreq = linearRpm * STEPS_PER_REV / 60.0f;
  if (rotaryFreq > SPEED_MAX || linearFreq > SPEED_MAX) {
    Serial.println(F("ERROR OUT_OF_RANGE"));
    return;
  }

  // Motor 2: continuous, no target, Timer2 hardware path.
  spin2Direction = 1;  // arbitrary — product-rotation direction doesn't matter for this profile
  digitalWrite(MOTOR2_DIR_PIN, HIGH);
  spin2TargetFreq = rotaryFreq;
  spin2CurrentFreq = 1.0f;
  spin2StoppingSoft = false;
  moveTo2Active = false;
  motor2MoveKind = 0;
  spin2RampLastMillis = millis();
  spinningHW2 = true;
  timer2StartSpin(spin2CurrentFreq);

  // Motor 1: negative direction, Timer1 hardware path, no target — runs
  // until D6/MIN triggers, at the entered linear RPM.
  spinDirection = -1;
  digitalWrite(MOTOR1_DIR_PIN, LOW);
  spinTargetFreq = linearFreq;
  spinCurrentFreq = 1.0f;
  spinStoppingSoft = false;
  moveToActive = false;
  spinRampLastMillis = millis();
  spinningHW = true;
  timer1StartSpin(spinCurrentFreq);

  profileUTActive = true;
  busy = true;
  movingMotor1 = true;
  movingMotor2 = true;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

// PROFILE_WT <rpm> — Motor 2 does PROFILE_WT_REPS x (rotate PROFILE_WT_DEG°
// at the given RPM, wait PROFILE_WT_WAIT_MS), completing one full
// revolution in quarter-turn steps, on the Timer2 hardware path
// (startMoveTo2, kind=WT_STEP). The timing between turns is tracked here
// in updateProfileWT(), called every loop() iteration so the rig stays
// responsive to STOP/ESTOP throughout.
void cmdProfileWT(char* argStr) {
  if (faultActive) { Serial.println(F("ERROR FAULT_ACTIVE")); return; }
  if (busy)         { Serial.println(F("BUSY")); return; }

  float rpm;
  if (!parseFloatArg(argStr, &rpm)) {
    Serial.println(F("ERROR INVALID_PARAM"));
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

  profileWTSpeed = freq;  // reused by each subsequent quarter-turn in updateProfileWT
  profileWTActive = true;
  profileWTWaiting = false;
  profileWTStep = 0;
  long steps = lround((double)PROFILE_WT_DEG * STEPS_PER_REV / 360.0);
  startMoveTo2(steps, profileWTSpeed, 3 /* WT_STEP */);

  busy = true;
  movingMotor1 = false;
  movingMotor2 = true;

  Serial.print(F("OK "));
  Serial.println(lastCommandText);
}

// Advances the WT sequence: waits for the current quarter-turn to finish
// (signaled by updateSpin2Ramp clearing spinningHW2/moveTo2Active once it
// lands exactly on target), pauses PROFILE_WT_WAIT_MS, then either starts
// the next quarter-turn or — once PROFILE_WT_REPS have completed — ends
// the sequence itself (this is why handleMotionCompletion() explicitly
// skips over PROFILE_WT: it would otherwise see the first quarter-turn's
// own completion and end the whole sequence right there).
void updateProfileWT() {
  if (!profileWTActive) return;

  if (profileWTWaiting) {
    if (millis() - profileWTWaitStartMillis < PROFILE_WT_WAIT_MS) return;
    profileWTWaiting = false;
    profileWTStep++;
    if (profileWTStep >= PROFILE_WT_REPS) {
      profileWTActive = false;
      busy = false;
      movingMotor2 = false;
      Serial.println(F("DONE PROFILE_WT"));
      return;
    }
    long steps = lround((double)PROFILE_WT_DEG * STEPS_PER_REV / 360.0);
    startMoveTo2(steps, profileWTSpeed, 3 /* WT_STEP */);
    return;
  }

  if (!spinningHW2 && !moveTo2Active) {
    profileWTWaiting = true;
    profileWTWaitStartMillis = millis();
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
  if (spinningHW) return;  // any Timer1-driven move (SPIN or MOVE_TO) ends itself via updateSpinRamp — STOP, ESTOP, or arrival at target — never here
  if (spinningHW2 || moveTo2Active) return;  // same for any Timer2-driven Motor 2 move — ends itself via updateSpin2Ramp
  if (profileWTActive) return;  // updateProfileWT owns completion for the whole 4-step sequence, not just one quarter-turn
  if (homeInProgress) return;   // checkHomeCompletion() owns this instead — it waits on both motors together

  // Only remaining consumer: ROTATE (motor1-only, still on AccelStepper —
  // Motor 2 has no AccelStepper path left at all).
  if (movingMotor1 && motor1.distanceToGo() == 0) {
    busy = false;
    movingMotor1 = false;
    Serial.print(F("DONE "));
    Serial.println(lastCommandText);
  }
}

void sendStatusLine() {
  const char* state = faultActive ? "FAULT" : (busy ? "MOVING" : "IDLE");
  Serial.print(F("POS "));
  Serial.print(motor1Position());
  Serial.print(' ');
  Serial.print(motor2Position());
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
