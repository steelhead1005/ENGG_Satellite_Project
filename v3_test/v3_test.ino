// ============================================================================
//  v3_test.ino: seth_test.ino with demo-day fixes, no Serial (2026-11-06)
// ============================================================================
//  Same pins, same idea (gyro heading lock + hold) as seth_test.ino. Changes:
//   1. Hold where it actually stopped. After braking, if both front sensors
//      still see the beacon, THAT heading becomes the target. The old code
//      saved the heading at the very edge of the lock zone and drove back to it.
//   2. Relocate timeout no longer skips the beacon. It fires anyway.
//   3. Rate (D) term in corrections. No push while already swinging toward
//      the target, and "reached" needs the robot to be nearly still.
//   4. No Serial output at all (it runs untethered; the old 9600 per-loop
//      prints were stalling the control loop to about 20 Hz).
//   5. Brake ends early once the gyro says rotation has stopped, so the
//      reverse pulse can't push it backwards at low speed.
//   6. Tracking no longer runs blind for 150 ms. It stops the drive step the
//      moment both front sensors see the beacon.
//   7. IR latch (IR_HOLD_MS) so one missed 38 kHz frame doesn't re-arm a lock,
//      and a lock only re-arms after the overlap is gone for REARM_MS.
//   8. I2C timeout so motor noise can't freeze the bus forever, and if the
//      MPU isn't found the robot falls back to IR-only (v1-style) instead of
//      hanging in setup().
//
//  10. Demo tuning (fixed beacons, each left on a while): waits until it is
//      on target AND still for STABLE_MS before firing, keeps holding the
//      heading after the 2 s, and only re-arms when all sensors go dark
//      (beacon off) or a side/back sensor sees a new beacon.
//
//  11. Slow + smooth (after 2nd hang test: too fast, jittery): gyro speed
//      control holds TRACK_RATE deg/s instead of a fixed PWM,
//      full stop before reversing, no reverse-plug brake, softer and fewer
//      hold nudges. Tune: TRACK_RATE for speed, SPEED_KI if it
//      surges (lower) or stalls (higher).
//
//  12. Wait until still before firing (STILL_*), and USE_HEADING_CORRECTION
//      switch: false = gyro never steers, only governs speed + waits.
//
//  13. After 3rd hang test (stalled, laser never fired again): START_PWM 180 /
//      MIN_PWM 130 so it doesn't stall, and re-arm if the beacon is still seen
//      but not locked for LOST_REARM_MS (moved beacon / new beacon in front).
//
//  14. MPU stands upright in the breadboard: yaw is measured about the
//      gravity "up" axis found at startup (any fixed mounting works).
//      Keep it still until the 1 blink. CORRECTION_INVERT = true.
//
//  15. Continuous hold (HOLD_*): steady push proportional to error instead of
//      taps, so it actually stops on target and resists cord-twist drift.
//
//  16. 90L and 150L merged into one left sensor at -120 (they always trigger
//      together). Either pin LOW counts. See IR SENSORS.
//
//  17. No phantom shot after a beacon switches off. The lock was checked
//      before the re-arm, and the split lock keeps front sightings 600 ms
//      while re-arm waits 500 ms, so it fired at the empty spot. Re-arm now
//      forgets those sightings and starts fresh on the next loop.
//
//  18. Active damping (ACTIVE_DAMPING). Motor off doesn't stop a reaction-wheel
//      satellite, so the wait before firing now uses the wheel to stop the
//      rotation and ease back onto the locked heading, and keeps it running
//      through the 2 s laser instead of on/off heading nudges. Still now
//      means under 2 deg/s. See ACTIVE DAMPING.
//
//   9. Split lock (2026-11-06, after first hang test): fires even when the
//      two front sensors never see the beacon at the same time. See SPLIT LOCK.
//
//  Startup laser blinks: 1 = gyro calibrated and ready, 3 = MPU not found
//  (running IR-only). Keep it still until the blink.
//
//  Tuning on the rig, in this order: CORRECTION_INVERT (sign), KD, then
//  CORRECTION_PULSE / CORRECTION_PWM, then ANGLE_TOLERANCE.
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
// updateIR(). At -120 it also ranks behind 90R, so a stray left reading can't
// win a tie and turn it away from a beacon on the right.
const int SENSOR_PINS[]   = { PIN_IR_30R, PIN_IR_30L, PIN_IR_150R, PIN_IR_150L, PIN_IR_90R };
const int SENSOR_ANGLES[] = { 30,         -30,        150,         -120,        90         };
const int NUM_SENSORS = 5;
const int IDX_30R = 0;
const int IDX_30L = 1;
const int IDX_LEFT = 3;     // merged 90L + 150L

// ---------------- IR FILTERING ----------------
const unsigned long IR_HOLD_MS = 40;    // a sensor "sees" for this long after its last LOW
const unsigned long REARM_MS   = 500;   // all sensors dark this long = beacon switched off
const unsigned long LOST_REARM_MS = 1500; // after firing: beacon seen but not locked this long = it moved, go again

// ---------------- SPLIT LOCK ----------------
// With the shrouds moved inside, +30 and -30 may never see the beacon at the
// same time (robot zig-zags and never fires). Split lock: if BOTH front
// sensors have seen the beacon within SPLIT_WINDOW_MS, aim at the gyro
// midpoint between the headings where each last saw it.
const unsigned long SPLIT_WINDOW_MS = 600;
const float SPLIT_MAX_DEG = 70.0;        // ignore if the two sightings are further apart

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
const unsigned long TRACK_STEP_MS = 150; // max length of one drive step (exits early on lock)
const unsigned long REVERSE_PAUSE_MS = 250; // full stop before reversing direction

// ---------------- BRAKE ----------------
const unsigned long OVERSHOOT_MS = 0;    // keep driving after lock (0 now that it arrives slowly)
const bool USE_REVERSE_BRAKE = false;    // slow approach: driver brake alone is enough, gentler
const unsigned long BRAKE_GAP_MS = 30;
const unsigned long BRAKE_MAX_MS = 50;   // reverse pulse, ends early when rotation stops
const float BRAKE_STOP_RATE = 15.0;      // deg/s: below this the reverse pulse ends

// ---------------- GYRO CORRECTION ----------------
const float ANGLE_TOLERANCE = 3.5;       // deg
const float HOLD_DEADBAND   = 6.0;       // deg: after firing, only correct beyond this
const float SETTLED_RATE    = 8.0;       // deg/s: "still enough" to call it reached
const float KD              = 0.15;      // s: predicted error = error - KD * rate
const int CORRECTION_PWM = 190;          // softer nudges = less jitter
const unsigned long CORRECTION_PULSE = 25;
const unsigned long CORRECTION_COAST = 150; // let each nudge finish before judging again
const unsigned long MAX_RELOCATE_TIME = 3000;  // beacons stay on a while: take time to settle
const unsigned long STABLE_MS = 250;           // must stay on target + still this long before firing

const bool CORRECTION_INVERT = true;     // flip if the hold drives AWAY from target

// ---------------- CONTINUOUS HOLD ----------------
// Instead of short taps, push continuously with PWM that grows with the error
// and backs off as it moves toward the target. This beats stiction and the
// twist of the hanging cord, which the taps couldn't.
const bool  HOLD_CONTINUOUS = true;   // false = old tap-style corrections
const float HOLD_KP      = 8.0;       // PWM per degree of error
const float HOLD_KD      = 2.0;       // PWM per deg/s (damping: eases off when moving toward target)
const int   HOLD_MIN_PWM = 150;       // smallest push (just under stiction; error adds on top)
const int   HOLD_MAX_PWM = 220;
const float HOLD_TOL     = 2.0;       // deg: inside this, motor off

// ---------------- WAIT UNTIL STILL (before firing) ----------------
// Fires once rotation stays below the still rate for STILL_MS, or after
// STILL_MAX_MS regardless.
const float STILL_RATE       = 2.0;           // deg/s: below this counts as still (ACTIVE_DAMPING)
const float STILL_RATE_COAST = 5.0;           // same, for the old motor-off wait
const unsigned long STILL_MS     = 400;       // must stay still this long
const unsigned long STILL_MAX_MS = 3000;      // give up waiting and fire anyway

// ---------------- ACTIVE DAMPING (wait + laser) ----------------
// Motor off doesn't stop a reaction-wheel satellite: whatever spin the swivel
// and wound string have added stays in the body when the wheel stops, and the
// string keeps pushing. Instead, the same gyro speed control as tracking runs
// in either direction, aiming for 0 deg/s plus a slow drift back onto the
// locked heading: the wheel speeds up while the body still turns, then keeps
// running at that speed through the whole 2 s laser. Still = on target too.
// Direction comes from CORRECTION_INVERT: if the satellite speeds up during
// the wait instead of stopping, flip that. Off when USE_HEADING_CORRECTION is.
// false = old behaviour: motor off to wait, heading nudges while firing.
const bool  ACTIVE_DAMPING  = true;
const float DAMP_KI         = 0.05;   // PWM per (deg/s) per ms. Higher = stops sooner, too high = wobble
const float DAMP_RETURN     = 2.0;    // deg/s back toward the locked heading per deg off (0 = just stop)
const float DAMP_U_START    = 20.0;   // wheel stays off until the push builds past this (no chatter)
const float DAMP_ABORT_RATE = 40.0;   // deg/s: faster than this = pushing the wrong way, give up

// Heading correction (relocate + nudges while firing + hold after).
// false = option 2: gyro only governs speed and waits for still; never steers
// to a heading, so CORRECTION_INVERT can't matter.
const bool USE_HEADING_CORRECTION = true;

// ---------------- LASER ----------------
const unsigned long LASER_TIME = 2000;

// ---------------- STATE ----------------
bool gyroOk = false;
bool wasSpinning = false;
float trackPwm = 0;
int trackDir = -1;            // -1 = not driving
int currentDir = LOW;
float dampU = 0;              // active damping push, + = toward + angle
unsigned long dampLast = 0;
bool dampGaveUp = false;

bool targetHandled = false;
float targetAngle = 0;

unsigned long lastSeen[NUM_SENSORS];
float lastSeenAngle[NUM_SENSORS];        // gyro heading when each sensor last saw the beacon

// The MPU stands UPRIGHT in the breadboard, so its Z axis is horizontal and
// can't see the robot turning. Instead: at startup, measure gravity to find
// which board axis points "up", then read rotation about that axis using all
// three gyro axes. Works for any FIXED mounting, no sign setting needed.
float biasX = 0, biasY = 0, biasZ = 0;
float upX = 0, upY = 0, upZ = 1;
float gyroZRate = 0;
float angleZ = 0;
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
      lastSeenAngle[i] = angleZ;
    }
  }
}

bool sees(int i) {
  return (lastSeen[i] != 0) && (millis() - lastSeen[i] <= IR_HOLD_MS);
}

bool overlapNow() {
  return sees(IDX_30R) && sees(IDX_30L);
}

bool recent(int i, unsigned long window) {
  return (lastSeen[i] != 0) && (millis() - lastSeen[i] <= window);
}

// Both front sensors saw the beacon recently (not necessarily together)
bool splitLockNow() {
  if (!recent(IDX_30R, SPLIT_WINDOW_MS) || !recent(IDX_30L, SPLIT_WINDOW_MS)) return false;
  if (!gyroOk) return true;
  return fabs(lastSeenAngle[IDX_30R] - lastSeenAngle[IDX_30L]) <= SPLIT_MAX_DEG;
}

bool lockNow() {
  return overlapNow() || splitLockNow();
}

// =====================================================
// GYRO
// =====================================================

void updateGyro() {
  if (!gyroOk) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - previousGyroTime) / 1000000.0;
  previousGyroTime = now;

  // Rotation about vertical. + = counter-clockwise seen from above.
  float gx = g.gyro.x * 180.0 / PI - biasX;
  float gy = g.gyro.y * 180.0 / PI - biasY;
  float gz = g.gyro.z * 180.0 / PI - biasZ;
  gyroZRate = gx * upX + gy * upY + gz * upZ;
  if (fabs(gyroZRate) < 0.3) {
    gyroZRate = 0;
  }

  angleZ += gyroZRate * dt;
}

// Waits while keeping the gyro and IR up to date.
void gyroDelay(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    updateGyro();
    updateIR();
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
  currentDir = dir;
  analogWrite(PIN_PWM, pwm);
}

// Reverse pulse that ends as soon as the gyro says rotation has stopped.
void brakeFromSpin(bool overshoot) {
  if (overshoot) gyroDelay(OVERSHOOT_MS);
  stopMotor();
  gyroDelay(BRAKE_GAP_MS);

  if (!USE_REVERSE_BRAKE) {
    // Driver brake (PWM 0) and let it settle
    gyroDelay(BRAKE_MAX_MS);
    return;
  }

  if (!gyroOk) {
    // No gyro: old fixed brake
    drive(!currentDir, 255);
    gyroDelay(BRAKE_MAX_MS);
    stopMotor();
    return;
  }

  float startRate = gyroZRate;
  int brakeDir = !currentDir;
  drive(brakeDir, 255);

  unsigned long start = millis();
  while (millis() - start < BRAKE_MAX_MS) {
    updateGyro();
    updateIR();
    // Stopped, or rate has changed sign (we'd start going backwards)
    if (fabs(gyroZRate) < BRAKE_STOP_RATE || (gyroZRate * startRate) < 0) {
      break;
    }
    delay(2);
  }
  stopMotor();
}

// One correction step towards targetAngle. Returns true when on target and still.
bool correctStep() {
  updateGyro();
  float error = targetAngle - angleZ;

  if (HOLD_CONTINUOUS) {
    updateIR();
    if (fabs(error) <= HOLD_TOL && fabs(gyroZRate) <= SETTLED_RATE) {
      stopMotor();
      delay(2);
      return true;
    }
    float u = HOLD_KP * error - HOLD_KD * gyroZRate;   // + means push toward + angle
    if (fabs(error) <= HOLD_TOL || fabs(u) < 1.0) {
      stopMotor();              // close enough, or damping says let it coast in
      delay(2);
      return false;
    }
    int pwm = HOLD_MIN_PWM + (int)fabs(u);
    if (pwm > HOLD_MAX_PWM) pwm = HOLD_MAX_PWM;
    bool direction = (u > 0) ? HIGH : LOW;
    if (CORRECTION_INVERT) direction = !direction;
    drive(direction, pwm);
    delay(2);
    return false;
  }

  float predicted = error - KD * gyroZRate;   // where we'll be shortly if we do nothing

  if (fabs(error) <= ANGLE_TOLERANCE && fabs(gyroZRate) <= SETTLED_RATE) {
    stopMotor();
    return true;
  }

  if (fabs(predicted) <= ANGLE_TOLERANCE) {
    // Already drifting into the window: don't push, let it arrive
    stopMotor();
    gyroDelay(10);
    return false;
  }

  bool direction = (predicted > 0) ? HIGH : LOW;
  if (CORRECTION_INVERT) {
    direction = !direction;
  }

  drive(direction, CORRECTION_PWM);
  gyroDelay(CORRECTION_PULSE);
  stopMotor();
  gyroDelay(CORRECTION_COAST);
  return false;
}

// =====================================================
// RETURN TO SAVED TARGET
// =====================================================

bool relocateToTarget() {
  unsigned long startTime = millis();

  unsigned long stableSince = 0;

  while (millis() - startTime < MAX_RELOCATE_TIME) {
    if (correctStep()) {
      // On target and still: wait until it STAYS that way (sway has died down)
      if (stableSince == 0) stableSince = millis();
      if (millis() - stableSince >= STABLE_MS) return true;
      gyroDelay(5);
    } else {
      stableSince = 0;
    }
  }

  stopMotor();
  return false;
}

// =====================================================
// LASER FIRE + HOLD
// =====================================================

// Active damping: start from a stopped wheel (relocate always ends stopped).
void startDamping() {
  stopMotor();
  dampU = 0;
  dampLast = millis();
  dampGaveUp = false;
}

// One step of the damping speed control. The push builds up while the body
// turns faster than it should (DAMP_RETURN * error, so 0 deg/s once on the
// heading) and stays where it is once it matches, so the wheel ends up
// spinning just fast enough to cancel the drift. Same idea as the tracking
// speed control, but signed so it can push either way.
void dampStep() {
  updateGyro();
  updateIR();
  unsigned long t = millis();
  if (fabs(gyroZRate) > DAMP_ABORT_RATE) dampGaveUp = true;
  if (dampGaveUp) {
    stopMotor();                    // speeding up instead of stopping: back to motor off
    delay(2);
    return;
  }
  float error = targetAngle - angleZ;
  dampU += DAMP_KI * (DAMP_RETURN * error - gyroZRate) * (t - dampLast);
  dampLast = t;
  float uMax = MAX_PWM - MIN_PWM + DAMP_U_START;
  if (dampU > uMax) dampU = uMax;
  if (dampU < -uMax) dampU = -uMax;
  if (fabs(dampU) < DAMP_U_START) {
    stopMotor();
  } else {
    bool direction = (dampU > 0) ? HIGH : LOW;   // same mapping as correctStep()
    if (CORRECTION_INVERT) direction = !direction;
    drive(direction, MIN_PWM + (int)(fabs(dampU) - DAMP_U_START));
  }
  delay(2);
}

// Wait for the twist/sway to die down before firing. With ACTIVE_DAMPING the
// wheel stops the rotation, eases it back onto the locked heading and is left
// running for the laser; otherwise the motor is off while it waits (option 4).
void waitUntilStill() {
  if (!gyroOk) {
    stopMotor();
    gyroDelay(500);                 // no gyro: just a fixed settle
    return;
  }
  bool damping = ACTIVE_DAMPING && USE_HEADING_CORRECTION;
  float stillRate = damping ? STILL_RATE : STILL_RATE_COAST;
  if (damping) startDamping(); else stopMotor();
  unsigned long start = millis();
  unsigned long stillSince = 0;
  while (millis() - start < STILL_MAX_MS) {
    if (damping) {
      dampStep();
    } else {
      updateGyro();
      updateIR();
      delay(2);
    }
    if (fabs(gyroZRate) < stillRate) {
      if (stillSince == 0) stillSince = millis();
      if (millis() - stillSince >= STILL_MS) return;
    } else {
      stillSince = 0;
    }
  }
}

void fireLaserAndHold() {
  digitalWrite(PIN_LASER, HIGH);

  unsigned long fireStart = millis();
  while (millis() - fireStart < LASER_TIME) {
    if (gyroOk && USE_HEADING_CORRECTION) {
      if (ACTIVE_DAMPING) dampStep();   // keep holding, carrying on from waitUntilStill()
      else correctStep();
    } else {
      updateIR();
      delay(2);
    }

  }

  digitalWrite(PIN_LASER, LOW);
  stopMotor();
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
// SETUP
// =====================================================

void setup() {

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
    lastSeen[i] = 0;
    lastSeenAngle[i] = 0;
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
    blinkLaser(3);   // 3 blinks = MPU not found, running IR-only
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

  angleZ = 0;
  previousGyroTime = micros();
  blinkLaser(1);     // 1 blink = gyro calibrated, ready
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  updateGyro();
  updateIR();

  bool overlap = lockNow();

  // Re-arm for the next beacon when either:
  //  - every sensor has been dark for REARM_MS (current beacon switched off), or
  //  - a side/back sensor sees a beacon (a different beacon has come on).
  // Beacons stay on a while in the demo, so this stops it re-firing at the
  // same beacon or wandering off it after the 2 s.
  if (targetHandled) {
    bool anyRecent = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (recent(i, REARM_MS)) anyRecent = true;
    }
    bool sideOrBack = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (i != IDX_30R && i != IDX_30L && sees(i)) sideOrBack = true;
    }
    // Beacon still visible to a front sensor but no longer locked (it was moved
    // or a new beacon is in front): after LOST_REARM_MS, go for it again.
    static unsigned long notLockedSince = 0;
    if (lockNow()) {
      notLockedSince = 0;
    } else if (anyRecent) {
      if (notLockedSince == 0) notLockedSince = millis();
      if (millis() - notLockedSince >= LOST_REARM_MS) sideOrBack = true;
    }
    if (!anyRecent || sideOrBack) {
      targetHandled = false;
      notLockedSince = 0;
      // Forget the old beacon's front sightings so the split lock can't fire
      // straight away at the spot where it was, and start fresh next loop:
      // `overlap` above was worked out before this re-arm.
      lastSeen[IDX_30R] = 0;
      lastSeen[IDX_30L] = 0;
      return;
    }
  }

  // ---------- NEW LOCK ----------
  if (overlap && !targetHandled) {
    targetHandled = true;
    float entryAngle = angleZ;
    bool trueOverlap = overlapNow();
    float splitMid = (lastSeenAngle[IDX_30R] + lastSeenAngle[IDX_30L]) / 2.0;

    if (wasSpinning) {
      brakeFromSpin(trueOverlap);  // no overshoot push for a split lock
      wasSpinning = false;
    }
    trackDir = -1;

    if (!gyroOk || !USE_HEADING_CORRECTION) {
      waitUntilStill();
      fireLaserAndHold();
      return;
    }

    // Let it settle a moment, then decide what to hold
    gyroDelay(100);
    if (!trueOverlap) {
      targetAngle = splitMid;      // aim between where +30 and -30 saw it
    } else if (overlapNow()) {
      targetAngle = angleZ;        // still locked where it stopped: hold here
    } else {
      targetAngle = entryAngle;    // slid out of the zone: go back to where lock began
    }


    relocateToTarget();            // fires whether or not this times out
    waitUntilStill();
    fireLaserAndHold();
    return;
  }

  // ---------- ALREADY FIRED, BEACON STILL ON ----------
  // Stay pointed at it (gyro hold) instead of chasing front-sensor flicker.
  if (targetHandled) {
    digitalWrite(PIN_LASER, LOW);
    trackDir = -1;
    if (gyroOk && USE_HEADING_CORRECTION && fabs(targetAngle - angleZ) > HOLD_DEADBAND) {
      correctStep();               // only fix big drift, otherwise leave it alone
    } else {
      stopMotor();
    }
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

  if (found) {
    digitalWrite(PIN_LASER, LOW);
    int dir = bestAngle > 0 ? HIGH : LOW;
    float targetRate = TRACK_RATE;

    if (dir != trackDir) {
      // Reversing (zig-zag across the beacon): stop fully first, then restart gently
      if (trackDir != -1) {
        stopMotor();
        gyroDelay(REVERSE_PAUSE_MS);
      }
      trackDir = dir;
      trackPwm = START_PWM;
    }
    wasSpinning = true;

    // Drive step with gyro speed control; stop waiting the moment we lock
    unsigned long stepStart = millis();
    unsigned long last = stepStart;
    while (millis() - stepStart < TRACK_STEP_MS) {
      updateGyro();
      updateIR();
      unsigned long t = millis();
      if (gyroOk) {
        trackPwm += SPEED_KI * (targetRate - fabs(gyroZRate)) * (t - last);
        if (trackPwm < MIN_PWM) trackPwm = MIN_PWM;
        if (trackPwm > MAX_PWM) trackPwm = MAX_PWM;
      } else {
        trackPwm = NO_GYRO_PWM;
      }
      last = t;
      drive(dir, (int)trackPwm);
      if (lockNow() && !targetHandled) break;
      delay(2);
    }

    return;
  }

  // ---------- NOTHING SEEN ----------
  stopMotor();
  digitalWrite(PIN_LASER, LOW);
  wasSpinning = false;
  trackDir = -1;
}
