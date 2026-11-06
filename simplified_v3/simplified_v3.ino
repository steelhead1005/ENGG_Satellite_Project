// ============================================================================
//  simplified_v3.ino: tracking-only version of v3_test (2026-11-06)
// ============================================================================
//  No relocate or wait-until-still. The satellite tracks with the same code
//  as v3_test, holds still once it's on the beacon, and fires the laser
//  alongside either (firing never steers):
//   1. TRACKING:  turn toward the sensor closest to the front.
//   2. COUNTDOWN: a front sensor (+30 or -30) has read the beacon. Keep
//      tracking, and fire as soon as it looks centred, or FIRE_DELAY_MS after
//      the first front reading at the latest. Centred = both front sensors
//      have read it since the countdown began (it's between them), or nothing
//      has seen it for GAP_CONFIRM_MS just after a front reading (it slipped
//      into the blind gap straight ahead, where tracking stops).
//   3. FIRING:    laser on for LASER_TIME while tracking/holding carries on.
//   4. COOLDOWN:  laser off for COOLDOWN_MS while tracking/holding carries on.
//  Then back to TRACKING (re-armed), so it fires again a bit after the next
//  front reading. Only one beacon is on at a time, so firing at the same one
//  again is harmless.
//
//  While a front sensor has the beacon, drive steps are FRONT_STEP_MS instead
//  of TRACK_STEP_MS, so it re-checks the sensors more often and corrects
//  before it drifts off target.
//
//  HOLD: when both front sensors see the beacon at once it's straight ahead,
//  so the gyro holds that heading instead of the motor just stopping. Motor
//  off doesn't stop a flywheel satellite on a string (leftover spin and the
//  string's twist keep turning it), so the flywheel is left running at
//  whatever speed cancels the drift. Tracking takes over again if the overlap
//  is gone for HOLD_LOST_MS or it ends up HOLD_MAX_ERROR off. See HOLD.
//
//  Kept from v3_test: pins, 90L + 150L merged into one left sensor, IR latch,
//  gyro speed control (MPU upright, yaw about gravity), I2C timeout, no Serial.
//
//  Startup laser blinks: 1 = gyro calibrated and ready, 3 = MPU not found
//  (fixed PWM instead of speed control, and no hold). Keep it still until
//  the blink.
//
//  Tune: FIRE_DELAY_MS (lower = fires sooner but may be off-centre),
//  GAP_CONFIRM_MS (raise if it fires off-centre when a reading flickers),
//  TRACK_RATE (speed), SPEED_KI (lower if it surges, higher if it stalls),
//  FRONT_STEP_MS (lower = corrects more often once a front sensor has it).
//  Hold, in this order: HOLD_INVERT (flip if it gets pushed off the beacon
//  instead of steadying), HOLD_KI (higher = steadies sooner, too high =
//  wobbles), HOLD_RETURN (how hard it eases back onto the heading).
// ============================================================================

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// ---------------- MOTOR DRIVER ----------------
const int PIN_DIR = 8;
const int PIN_PWM = 9;

// ---------------- LASER ----------------
const int PIN_LASER = 6;

// ---------------- IR SENSORS (active LOW) ----------------
// Positive = right, negative = left
const int PIN_IR_30R  = 3;
const int PIN_IR_90R  = 10;
const int PIN_IR_150R = 12;
const int PIN_IR_150L = 7;
const int PIN_IR_90L  = 11;
const int PIN_IR_30L  = 5;

// 90L and 150L always trigger together (reflection or wiring), so they are one
// logical LEFT sensor at -120: PIN_IR_90L is OR'd into the 150L slot in
// updateIR().
const int SENSOR_PINS[]   = { PIN_IR_30R, PIN_IR_30L, PIN_IR_150R, PIN_IR_150L, PIN_IR_90R };
const int SENSOR_ANGLES[] = { 30,         -30,        150,         -120,        90         };
const int NUM_SENSORS = 5;
const int IDX_30R = 0;
const int IDX_30L = 1;
const int IDX_LEFT = 3;     // merged 90L + 150L

// ---------------- IR FILTERING ----------------
const unsigned long IR_HOLD_MS = 40;    // a sensor "sees" for this long after its last LOW

// ---------------- MOTOR SPEED (gyro speed control) ----------------
// Instead of a fixed PWM, the gyro holds a slow, steady rotation speed. PWM
// creeps up until the robot moves at the target speed, then backs off, so it
// overcomes stiction without lurching and accelerates gently (less sway).
const float TRACK_RATE    = 12.0;  // deg/s, one slow speed everywhere (less momentum to kill)
const int   START_PWM     = 180;   // PWM to start from (v1 found ~180 = slowest that beats stiction)
const int   MIN_PWM       = 130;   // never drop below this while driving, or it stalls
const int   MAX_PWM       = 230;
const float SPEED_KI      = 0.03;  // PWM per (deg/s of error) per ms. Higher = reacts faster
const int   NO_GYRO_PWM   = 180;   // fixed PWM if the MPU isn't found
const unsigned long TRACK_STEP_MS = 150; // length of one drive step
const unsigned long FRONT_STEP_MS = 40;  // shorter step while a front sensor has it, so it
                                         // re-checks ~4x as often and drifts less past centre
const unsigned long REVERSE_PAUSE_MS = 250; // full stop before reversing direction
// Blind gaps between side/back sensors: after the beacon slips out of a side
// sensor's view, keep turning the same way this long so the next sensor picks
// it up. (Not in the gap straight ahead: that's where it should stop.)
const unsigned long GAP_SWEEP_MS = 1000;

// ---------------- HOLD (both front sensors see it) ----------------
// Holds the heading where both front sensors saw the beacon: the middle of
// all the headings where they both saw it during this hold, so it settles
// inside the overlap, not at its edge. Same gyro speed control as tracking,
// but signed so it can push either way: it aims for 0 deg/s plus a slow
// drift back onto the heading, and leaves the flywheel spinning at whatever
// speed that takes. Adapted from v3_test's ACTIVE_DAMPING.
const float HOLD_KI      = 0.03;    // like SPEED_KI. Higher = steadies sooner, too high = wobbles
const float HOLD_RETURN  = 1.0;     // deg/s back toward the heading per deg off (0 = only stop the turning)
const float HOLD_U_START = 20.0;    // flywheel stays off until the push builds past this (no chatter)
const unsigned long HOLD_KICK_MS = 40;   // START_PWM this long when the flywheel starts from stopped (stiction)
const unsigned long HOLD_LOST_MS = 500;  // overlap gone this long = back to tracking
const float HOLD_MAX_ERROR = 10.0;  // deg: pushed further off the heading than this = back to tracking
const bool  HOLD_INVERT  = true;    // which way DIR turns it (CORRECTION_INVERT in v3_test). Flip if
                                    // the hold pushes it off the beacon instead of steadying it

// ---------------- FIRING ----------------
const unsigned long FIRE_DELAY_MS  = 5000;  // fire at the latest this long after the first front reading
                                            // (usually fires sooner, the moment it looks centred)
const unsigned long GAP_CONFIRM_MS = 200;   // nothing seen this long = in the gap ahead, not a flicker
const unsigned long GAP_MS         = 600;   // ...as long as a front sensor read it this recently
const unsigned long LOST_MS        = 1000;  // nothing seen this long = beacon gone, cancel the countdown
const unsigned long LASER_TIME     = 2000;  // laser on this long
const unsigned long COOLDOWN_MS    = 2000;  // then off at least this long before it can fire again

// ---------------- STATE ----------------
enum FireState { TRACKING, COUNTDOWN, FIRING, COOLDOWN };
FireState fireState = TRACKING;
unsigned long stateSince = 0;

bool gyroOk = false;
float trackPwm = 0;
int trackDir = -1;            // -1 = not driving

bool holding = false;         // gyro heading hold, see HOLD
float holdTarget = 0;         // heading being held (deg, gyro angle)
float overlapLo = 0;          // range of headings where both front sensors saw it this hold
float overlapHi = 0;
unsigned long lastOverlap = 0;
float holdU = 0;              // flywheel push, + = toward + angle. PWM = MIN_PWM + (|holdU| - HOLD_U_START)
unsigned long holdLast = 0;
bool holdDriving = false;     // flywheel being driven (false = motor off)
int holdLastDir = -1;         // direction the flywheel was last driven in
unsigned long holdKickUntil = 0;

unsigned long lastSeen[NUM_SENSORS];

// The MPU stands UPRIGHT in the breadboard, so its Z axis is horizontal and
// can't see the robot turning. Instead: at startup, measure gravity to find
// which board axis points "up", then read rotation about that axis using all
// three gyro axes. Works for any FIXED mounting, no sign setting needed.
float biasX = 0, biasY = 0, biasZ = 0;
float upX = 0, upY = 0, upZ = 1;
float gyroZRate = 0;
float angleZ = 0;             // heading from the gyro (deg, + = counter-clockwise seen from above)
unsigned long previousGyroTime = 0;

// =====================================================
// IR
// =====================================================

void updateIR() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_SENSORS; i++) {
    bool low = digitalRead(SENSOR_PINS[i]) == LOW;
    if (i == IDX_LEFT && digitalRead(PIN_IR_90L) == LOW) low = true;
    if (low) {
      lastSeen[i] = now;
    }
  }
}

bool sees(int i) {
  return (lastSeen[i] != 0) && (millis() - lastSeen[i] <= IR_HOLD_MS);
}

bool recent(int i, unsigned long window) {
  return (lastSeen[i] != 0) && (millis() - lastSeen[i] <= window);
}

bool anyRecent(unsigned long window) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (recent(i, window)) return true;
  }
  return false;
}

// Read at any point since time t
bool readSince(int i, unsigned long t) {
  return (lastSeen[i] != 0) && (lastSeen[i] >= t);
}

// =====================================================
// GYRO
// =====================================================

// Rotation speed and heading about vertical. + = counter-clockwise seen from
// above. Tracking only uses the speed's size; the hold uses both, signed.
void updateGyro() {
  if (!gyroOk) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - previousGyroTime) / 1000000.0;
  previousGyroTime = now;

  float gx = g.gyro.x * 180.0 / PI - biasX;
  float gy = g.gyro.y * 180.0 / PI - biasY;
  float gz = g.gyro.z * 180.0 / PI - biasZ;
  gyroZRate = gx * upX + gy * upY + gz * upZ;
  // Small rates are no longer rounded to 0 (was: below 0.3 deg/s): the hold
  // has to see the string's slow creep, or it would let it wander off.
  angleZ += gyroZRate * dt;
}

// =====================================================
// FIRING (runs alongside tracking)
// =====================================================

void updateFire() {
  unsigned long now = millis();

  if (fireState == TRACKING) {
    if (sees(IDX_30R) || sees(IDX_30L)) {
      fireState = COUNTDOWN;
      stateSince = now;
    }
    return;
  }

  if (fireState == COUNTDOWN) {
    if (!anyRecent(LOST_MS)) {
      fireState = TRACKING;          // beacon gone (switched off): start again
      return;
    }
    unsigned long from = stateSince - IR_HOLD_MS;   // include the reading that started it
    bool between = readSince(IDX_30R, from) && readSince(IDX_30L, from);
    bool inGap = !anyRecent(GAP_CONFIRM_MS) &&
                 (recent(IDX_30R, GAP_MS) || recent(IDX_30L, GAP_MS));
    if (between || inGap || now - stateSince >= FIRE_DELAY_MS) {
      digitalWrite(PIN_LASER, HIGH);
      fireState = FIRING;
      stateSince = now;
    }
    return;
  }

  if (fireState == FIRING) {
    if (now - stateSince >= LASER_TIME) {
      digitalWrite(PIN_LASER, LOW);
      fireState = COOLDOWN;
      stateSince = now;
    }
    return;
  }

  // COOLDOWN: laser off, still tracking, not allowed to fire yet
  if (now - stateSince >= COOLDOWN_MS) {
    fireState = TRACKING;            // re-armed: fires again after the next front reading
  }
}

// Waits while keeping the gyro, IR and laser timing up to date.
void gyroDelay(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    updateGyro();
    updateIR();
    updateFire();
    delay(2);
  }
}

// =====================================================
// MOTOR
// =====================================================

void stopMotor() {
  analogWrite(PIN_PWM, 0);
}

void drive(int dir, int pwm) {
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_PWM, pwm);
}

// Status signal, since there's no Serial: short laser blinks
void blinkLaser(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LASER, HIGH);
    delay(150);
    digitalWrite(PIN_LASER, LOW);
    delay(250);
  }
}

// =====================================================
// HOLD (both front sensors see it)
// =====================================================

// DIR that pushes toward + gyro angle (push > 0) or - angle (push < 0)
int holdDir(float push) {
  int dir = (push > 0) ? HIGH : LOW;
  if (HOLD_INVERT) dir = !dir;
  return dir;
}

void startHold() {
  holding = true;
  overlapLo = angleZ;
  overlapHi = angleZ;
  holdTarget = angleZ;
  holdU = 0;                  // stop the flywheel first: that's what stops the turn
  holdLast = millis();
  holdDriving = false;
  holdLastDir = trackDir;     // tracking may have left it spinning this way
  stopMotor();
}

// Back to tracking. Tracking carries on from the flywheel's last direction,
// so it still pauses before reversing it (REVERSE_PAUSE_MS).
void endHold() {
  holding = false;
  trackDir = holdLastDir;
  trackPwm = holdDriving ? MIN_PWM + (fabs(holdU) - HOLD_U_START) : START_PWM;
  holdDriving = false;
}

// One step of the hold. The push builds up while the satellite turns faster
// than it should (HOLD_RETURN * error, so 0 deg/s once on the heading) and
// stays where it is once it matches, so the flywheel ends up spinning just
// fast enough to cancel the drift. Same idea as the tracking speed control,
// but signed so it can push either way.
void holdStep() {
  unsigned long t = millis();
  float want = HOLD_RETURN * (holdTarget - angleZ);   // deg/s back toward the heading
  if (want > TRACK_RATE) want = TRACK_RATE;
  if (want < -TRACK_RATE) want = -TRACK_RATE;
  holdU += HOLD_KI * (want - gyroZRate) * (t - holdLast);
  holdLast = t;
  float uMax = MAX_PWM - MIN_PWM + HOLD_U_START;
  if (holdU > uMax) holdU = uMax;
  if (holdU < -uMax) holdU = -uMax;

  if (fabs(holdU) < HOLD_U_START) {
    stopMotor();              // small push: flywheel off (no chatter)
    holdDriving = false;
    return;
  }
  if (!holdDriving) holdKickUntil = t + HOLD_KICK_MS;   // flywheel was stopped: kick it free
  holdDriving = true;
  int pwm = MIN_PWM + (int)(fabs(holdU) - HOLD_U_START);
  if (t < holdKickUntil && pwm < START_PWM) pwm = START_PWM;
  holdLastDir = holdDir(holdU);
  drive(holdLastDir, pwm);
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
    lastSeen[i] = 0;
  }
  pinMode(PIN_IR_90L, INPUT_PULLUP);   // not in SENSOR_PINS: merged into IDX_LEFT
  stopMotor();
  digitalWrite(PIN_LASER, LOW);

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);   // don't hang forever on an I2C glitch
#endif

  gyroOk = mpu.begin();
  if (!gyroOk) {
    blinkLaser(3);   // 3 blinks = MPU not found, fixed PWM instead of speed control
    return;
  }
  Wire.setClock(400000);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500);   // let hand-twist settle after power-on
  float sgx = 0, sgy = 0, sgz = 0, sax = 0, say = 0, saz = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sgx += g.gyro.x * 180.0 / PI;
    sgy += g.gyro.y * 180.0 / PI;
    sgz += g.gyro.z * 180.0 / PI;
    sax += a.acceleration.x;
    say += a.acceleration.y;
    saz += a.acceleration.z;
    delay(5);
  }
  biasX = sgx / 500.0;
  biasY = sgy / 500.0;
  biasZ = sgz / 500.0;
  // At rest the accelerometer reads +1 g pointing UP: that's the turning axis
  float mag = sqrt(sax * sax + say * say + saz * saz);
  if (mag > 1.0) {
    upX = sax / mag;
    upY = say / mag;
    upZ = saz / mag;
  }

  blinkLaser(1);     // 1 blink = gyro calibrated, ready
  angleZ = 0;
  previousGyroTime = micros();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  updateGyro();
  updateIR();
  updateFire();

  bool overlap = sees(IDX_30R) && sees(IDX_30L);

  // ---------- HOLD ----------
  // Both front sensors see it: it's straight ahead, so hold this heading with
  // the gyro instead of chasing one of them.
  if (gyroOk) {
    if (holding && fabs(holdTarget - angleZ) > HOLD_MAX_ERROR) {
      endHold();                     // pushed too far off: give up (re-locks below if still overlapping)
    }
    if (overlap) {
      if (!holding) startHold();
      lastOverlap = millis();
      if (angleZ < overlapLo) overlapLo = angleZ;
      if (angleZ > overlapHi) overlapHi = angleZ;
      holdTarget = (overlapLo + overlapHi) / 2.0;   // middle of the overlap seen so far
    }
    if (holding && millis() - lastOverlap > HOLD_LOST_MS) {
      endHold();                     // beacon moved or switched off: back to tracking
    }
    if (holding) {
      holdStep();
      delay(2);
      return;
    }
  } else if (overlap) {
    // No gyro, so no hold: just stop between the two front sensors
    stopMotor();
    trackDir = -1;
    return;
  }

  // ---------- TRACKING ----------
  bool found = false;
  int bestAngle = 0;
  int bestAbs = 9999;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sees(i)) {
      int absAngle = abs(SENSOR_ANGLES[i]);
      if (absAngle < bestAbs) {
        bestAbs = absAngle;
        bestAngle = SENSOR_ANGLES[i];
        found = true;
      }
    }
  }

  // Side gap: the beacon has just slipped out of a side/back sensor's view and
  // nothing sees it. Keep turning the same way until the next sensor picks it
  // up, instead of stopping short of the front sensors.
  if (!found && trackDir != -1 &&
      !recent(IDX_30R, GAP_SWEEP_MS) && !recent(IDX_30L, GAP_SWEEP_MS)) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (i != IDX_30R && i != IDX_30L && recent(i, GAP_SWEEP_MS)) found = true;
    }
    if (found) bestAngle = (trackDir == HIGH) ? 1 : -1;
  }

  if (found) {
    int dir = bestAngle > 0 ? HIGH : LOW;

    if (dir != trackDir) {
      // Reversing (zig-zag across the beacon): stop fully first, then restart gently
      if (trackDir != -1) {
        stopMotor();
        gyroDelay(REVERSE_PAUSE_MS);
      }
      trackDir = dir;
      trackPwm = START_PWM;
    }

    // Drive step with gyro speed control. Shorter while a front sensor has it,
    // so it gets back to the checks above sooner and corrects before drifting.
    unsigned long stepLen = (sees(IDX_30R) || sees(IDX_30L)) ? FRONT_STEP_MS : TRACK_STEP_MS;
    unsigned long stepStart = millis();
    unsigned long last = stepStart;
    while (millis() - stepStart < stepLen) {
      updateGyro();
      updateIR();
      updateFire();
      if (sees(IDX_30R) && sees(IDX_30L)) break;   // overlap: start the hold now, not after the step
      unsigned long t = millis();
      if (gyroOk) {
        trackPwm += SPEED_KI * (TRACK_RATE - fabs(gyroZRate)) * (t - last);
        if (trackPwm < MIN_PWM) trackPwm = MIN_PWM;
        if (trackPwm > MAX_PWM) trackPwm = MAX_PWM;
      } else {
        trackPwm = NO_GYRO_PWM;
      }
      last = t;
      drive(dir, (int)trackPwm);
      delay(2);
    }

    return;
  }

  // ---------- NOTHING SEEN ----------
  stopMotor();
  trackDir = -1;
}
