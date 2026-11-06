// ============================================================================
//  v3_test.ino: demo sketch, simplified (2026-11-06). No Serial.
// ============================================================================
//  What it does, in order:
//   1. SEEK: turn toward the most-forward IR sensor that sees a beacon, at a
//      steady TRACK_RATE deg/s (gyro speed control).
//   2. LOCK: 30R and 30L both see it at once, or both saw it within
//      SPLIT_WINDOW_MS (split lock).
//   3. BRAKE (PWM 0 = driver brake), settle, pick the aim heading:
//        split lock          -> halfway between where 30R and 30L last saw it
//        still overlapping   -> where it stopped
//        slid out of overlap -> where the lock began
//      then add AIM_OFFSET_DEG.
//   4. RELOCATE to that heading (gyro hold) until on target and still.
//   5. FIRE the laser for 2 s while holding the heading.
//   6. DONE: motor off unless pushed more than IDLE_DEADBAND off. Goes again
//      when all sensors are dark for 0.5 s, a side/back sensor sees a beacon,
//      or the beacon is seen but not locked for 1.5 s.
//
//  Startup blinks: 1 = gyro calibrated and ready (keep it STILL until then),
//  3 = MPU not found (runs IR-only, no heading hold).
//
//  The MPU stands upright, so yaw is measured about the gravity "up" axis
//  found at startup. + yaw = counter-clockwise seen from above.
// ============================================================================

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// ============================================================================
//  TUNE THESE (one per test run)
// ============================================================================
const float AIM_OFFSET_DEG = 0.0;  // laser always lands one side: + aims right, - aims left (deg)
const float HOLD_TOL       = 2.0;  // deg: "on target". Smaller = more accurate, but may buzz
const int   HOLD_MIN_PWM   = 165;  // smallest push near target. Stops short: raise. Overshoots: lower
const float TRACK_RATE     = 12.0; // deg/s seeking speed. Sways: lower. Too slow: raise
const float IDLE_DEADBAND  = 5.0;  // deg: between beacons, only push back if turned more than this
const bool  CORRECTION_INVERT = true; // flip ONLY if the hold drives away from the target

// ============================================================================
//  FIXED (leave these alone)
// ============================================================================
// Pins
const int PIN_DIR = 8;
const int PIN_PWM = 9;
const int PIN_LASER = 6;
const int PIN_IR_30R  = 3;
const int PIN_IR_90R  = 10;
const int PIN_IR_150R = 12;
const int PIN_IR_150L = 7;
const int PIN_IR_90L  = 11;
const int PIN_IR_30L  = 5;

// Sensor list (angle: + = right, - = left)
const int SENSOR_PINS[]   = { PIN_IR_30R, PIN_IR_30L, PIN_IR_150R, PIN_IR_150L, PIN_IR_90L, PIN_IR_90R };
const int SENSOR_ANGLES[] = { 30,         -30,        150,         -150,        -90,        90         };
const int NUM_SENSORS = 6;
const int IDX_30R = 0;
const int IDX_30L = 1;

// IR timing
const unsigned long IR_HOLD_MS      = 40;   // a sensor "sees" for this long after its last LOW
const unsigned long SPLIT_WINDOW_MS = 600;  // split lock: both fronts seen within this
const float         SPLIT_MAX_DEG   = 70.0; // ...and their sightings this close together
const unsigned long REARM_MS        = 500;  // all dark this long = beacon switched off
const unsigned long LOST_REARM_MS   = 1500; // seen but not locked this long = go again

// Seeking (gyro speed control)
const int   START_PWM = 180;
const int   MIN_PWM   = 130;
const int   MAX_PWM   = 230;
const float SPEED_KI  = 0.03;              // PWM per (deg/s error) per ms
const int   NO_GYRO_PWM = 180;             // fixed PWM if no MPU
const unsigned long TRACK_STEP_MS    = 150;
const unsigned long REVERSE_PAUSE_MS = 250; // full stop before reversing

// Brake / hold
const unsigned long BRAKE_MS = 80;         // PWM 0 (driver brake) after lock
const float HOLD_KP      = 8.0;            // PWM per deg of error
const float HOLD_KD      = 2.0;            // PWM per deg/s (eases off when moving toward target)
const int   HOLD_MAX_PWM = 220;
const float HOLD_U_START = 6.0;            // ignore tiny pushes (gyro noise)
const float SETTLED_RATE = 8.0;            // deg/s: slow enough to count as on target
const float STUCK_RATE   = 2.0;            // pushing but slower than this = stuck
const float STUCK_BOOST_PER_MS = 0.1;      // extra PWM per ms while stuck
const unsigned long STABLE_MS = 250;       // on target + still this long, then fire
const unsigned long MAX_RELOCATE_TIME = 5000;

// Fallback wait (only if relocate times out, or no MPU)
const float STILL_RATE = 5.0;
const unsigned long STILL_MS     = 400;
const unsigned long STILL_MAX_MS = 3000;

const unsigned long LASER_TIME = 2000;

// ---------------- STATE ----------------
bool gyroOk = false;
bool wasSpinning = false;
float trackPwm = 0;
int trackDir = -1;            // -1 = not driving

bool targetHandled = false;
float targetAngle = 0;

unsigned long lastSeen[NUM_SENSORS];
float lastSeenAngle[NUM_SENSORS];        // gyro heading when each sensor last saw the beacon

float biasX = 0, biasY = 0, biasZ = 0;
float upX = 0, upY = 0, upZ = 1;
float gyroZRate = 0;
float angleZ = 0;
unsigned long previousGyroTime = 0;

bool  idleActive = false;      // false = take a new idle heading next time
bool  idleCorrecting = false;
float idleTarget = 0;

// =====================================================
// IR
// =====================================================

void updateIR() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) {
      lastSeen[i] = now;
      lastSeenAngle[i] = angleZ;
    }
  }
}

bool sees(int i) {
  return (lastSeen[i] != 0) && (millis() - lastSeen[i] <= IR_HOLD_MS);
}

bool recent(int i, unsigned long window) {
  return (lastSeen[i] != 0) && (millis() - lastSeen[i] <= window);
}

bool overlapNow() {
  return sees(IDX_30R) && sees(IDX_30L);
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
  if (fabs(gyroZRate) < 0.3) gyroZRate = 0;

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
  analogWrite(PIN_PWM, pwm);
}

// One hold step toward targetAngle. Returns true when on target and still.
// Only ever pushes TOWARD the target; if it's already swinging in, it brakes.
bool correctStep() {
  updateGyro();
  updateIR();
  float error = targetAngle - angleZ;

  if (fabs(error) <= HOLD_TOL && fabs(gyroZRate) <= SETTLED_RATE) {
    stopMotor();
    delay(2);
    return true;
  }
  float u = HOLD_KP * error - HOLD_KD * gyroZRate;   // + means push toward + angle
  bool pushAway = (u > 0) != (error > 0);
  if (fabs(error) <= HOLD_TOL || pushAway || fabs(u) < HOLD_U_START) {
    stopMotor();
    delay(2);
    return false;
  }
  // Stiction: if pushing but barely moving, keep adding PWM until it moves
  static float stuckBoost = 0;
  static unsigned long lastPush = 0;
  unsigned long nowMs = millis();
  if (fabs(gyroZRate) < STUCK_RATE && lastPush != 0 && nowMs - lastPush < 50) {
    stuckBoost += STUCK_BOOST_PER_MS * (nowMs - lastPush);
  } else {
    stuckBoost = 0;
  }
  lastPush = nowMs;
  int pwm = HOLD_MIN_PWM + (int)fabs(u) + (int)stuckBoost;
  if (pwm > HOLD_MAX_PWM) pwm = HOLD_MAX_PWM;
  bool direction = (u > 0) ? HIGH : LOW;
  if (CORRECTION_INVERT) direction = !direction;
  drive(direction, pwm);
  delay(2);
  return false;
}

// Turn to targetAngle; done once on target and still for STABLE_MS.
bool relocateToTarget() {
  unsigned long startTime = millis();
  unsigned long stableSince = 0;
  while (millis() - startTime < MAX_RELOCATE_TIME) {
    if (correctStep()) {
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

// Motor off, wait for the sway to die down (fallback only).
void waitUntilStill() {
  stopMotor();
  if (!gyroOk) {
    gyroDelay(500);
    return;
  }
  unsigned long start = millis();
  unsigned long stillSince = 0;
  while (millis() - start < STILL_MAX_MS) {
    updateGyro();
    updateIR();
    if (fabs(gyroZRate) < STILL_RATE) {
      if (stillSince == 0) stillSince = millis();
      if (millis() - stillSince >= STILL_MS) return;
    } else {
      stillSince = 0;
    }
    delay(2);
  }
}

// Between beacons: keep the motor off unless turned more than IDLE_DEADBAND.
void idleHold() {
  if (!gyroOk) { stopMotor(); return; }
  if (!idleActive) {
    idleActive = true;
    idleCorrecting = false;
    idleTarget = angleZ;
  }
  if (!idleCorrecting && fabs(idleTarget - angleZ) > IDLE_DEADBAND) idleCorrecting = true;
  if (!idleCorrecting) { stopMotor(); return; }
  float saved = targetAngle;
  targetAngle = idleTarget;
  if (correctStep()) idleCorrecting = false;   // back within HOLD_TOL and still
  targetAngle = saved;
}

void fireLaserAndHold() {
  digitalWrite(PIN_LASER, HIGH);
  unsigned long fireStart = millis();
  while (millis() - fireStart < LASER_TIME) {
    if (gyroOk) {
      correctStep();
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

  // ---------- RE-ARM after firing ----------
  // Go again when: all sensors dark for REARM_MS, a side/back sensor sees a
  // beacon, or the beacon is seen but not locked for LOST_REARM_MS.
  if (targetHandled) {
    bool anyRecent = false;
    bool sideOrBack = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (recent(i, REARM_MS)) anyRecent = true;
      if (i != IDX_30R && i != IDX_30L && sees(i)) sideOrBack = true;
    }
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
    }
  }

  // ---------- NEW LOCK ----------
  if (overlap && !targetHandled) {
    targetHandled = true;
    idleActive = false;
    float entryAngle = angleZ;
    bool trueOverlap = overlapNow();
    float splitMid = (lastSeenAngle[IDX_30R] + lastSeenAngle[IDX_30L]) / 2.0;

    if (wasSpinning) {
      stopMotor();               // PWM 0 = driver brake
      gyroDelay(BRAKE_MS);
      wasSpinning = false;
    }
    trackDir = -1;

    if (!gyroOk) {
      waitUntilStill();
      fireLaserAndHold();
      return;
    }

    // Let it settle a moment, then decide what to aim at
    gyroDelay(100);
    if (!trueOverlap) {
      targetAngle = splitMid;      // aim between where 30R and 30L saw it
    } else if (overlapNow()) {
      targetAngle = angleZ;        // still locked where it stopped: hold here
    } else {
      targetAngle = entryAngle;    // slid out of the zone: go back to where lock began
    }
    targetAngle -= AIM_OFFSET_DEG; // + yaw is CCW (left), so right = minus

    if (!relocateToTarget()) waitUntilStill();
    fireLaserAndHold();
    return;
  }

  // ---------- ALREADY FIRED, BEACON STILL ON ----------
  if (targetHandled) {
    digitalWrite(PIN_LASER, LOW);
    trackDir = -1;
    idleHold();
    return;
  }

  // ---------- SEEK ----------
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

    if (dir != trackDir) {
      // Reversing: stop fully first, then restart gently
      if (trackDir != -1) {
        stopMotor();
        gyroDelay(REVERSE_PAUSE_MS);
      }
      trackDir = dir;
      trackPwm = START_PWM;
    }
    wasSpinning = true;
    idleActive = false;

    // Drive step with gyro speed control; stop the moment we lock
    unsigned long stepStart = millis();
    unsigned long last = stepStart;
    while (millis() - stepStart < TRACK_STEP_MS) {
      updateGyro();
      updateIR();
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
      if (lockNow() && !targetHandled) break;
      delay(2);
    }
    return;
  }

  // ---------- NOTHING SEEN ----------
  idleHold();
  digitalWrite(PIN_LASER, LOW);
  wasSpinning = false;
  trackDir = -1;
}
