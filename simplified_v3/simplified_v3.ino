// ============================================================================
//  simplified_v3.ino: tracking-only version of v3_test (2026-11-06)
// ============================================================================
//  No lock, relocate, wait-until-still or damping. The satellite tracks with
//  the same code as v3_test and fires the laser while it tracks:
//   1. TRACKING:  turn toward the sensor closest to the front.
//   2. COUNTDOWN: a front sensor (+30 or -30) has read the beacon. Keep
//      tracking, and fire as soon as it looks centred, or FIRE_DELAY_MS after
//      the first front reading at the latest. Centred = both front sensors
//      have read it since the countdown began (it's between them), or nothing
//      has seen it for GAP_CONFIRM_MS just after a front reading (it slipped
//      into the blind gap straight ahead, where tracking stops).
//   3. FIRING:    laser on for LASER_TIME while tracking carries on.
//  Then straight back to TRACKING (re-armed), so it fires again a bit after
//  the next front reading. Only one beacon is on at a time, so firing at the
//  same one again is harmless.
//
//  Kept from v3_test: pins, 90L + 150L merged into one left sensor, IR latch,
//  gyro speed control (MPU upright, yaw about gravity), I2C timeout, no Serial.
//
//  Startup laser blinks: 1 = gyro calibrated and ready, 3 = MPU not found
//  (fixed PWM instead of speed control). Keep it still until the blink.
//
//  Tune: FIRE_DELAY_MS (lower = fires sooner but may be off-centre),
//  GAP_CONFIRM_MS (raise if it fires off-centre when a reading flickers),
//  TRACK_RATE (speed), SPEED_KI (lower if it surges, higher if it stalls).
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
const unsigned long REVERSE_PAUSE_MS = 250; // full stop before reversing direction
// Blind gaps between side/back sensors: after the beacon slips out of a side
// sensor's view, keep turning the same way this long so the next sensor picks
// it up. (Not in the gap straight ahead: that's where it should stop.)
const unsigned long GAP_SWEEP_MS = 1000;

// ---------------- FIRING ----------------
const unsigned long FIRE_DELAY_MS  = 5000;  // fire at the latest this long after the first front reading
                                            // (usually fires sooner, the moment it looks centred)
const unsigned long GAP_CONFIRM_MS = 200;   // nothing seen this long = in the gap ahead, not a flicker
const unsigned long GAP_MS         = 600;   // ...as long as a front sensor read it this recently
const unsigned long LOST_MS        = 1000;  // nothing seen this long = beacon gone, cancel the countdown
const unsigned long LASER_TIME     = 2000;

// ---------------- STATE ----------------
enum FireState { TRACKING, COUNTDOWN, FIRING };
FireState fireState = TRACKING;
unsigned long stateSince = 0;

bool gyroOk = false;
float trackPwm = 0;
int trackDir = -1;            // -1 = not driving

unsigned long lastSeen[NUM_SENSORS];

// The MPU stands UPRIGHT in the breadboard, so its Z axis is horizontal and
// can't see the robot turning. Instead: at startup, measure gravity to find
// which board axis points "up", then read rotation about that axis using all
// three gyro axes. Works for any FIXED mounting, no sign setting needed.
float biasX = 0, biasY = 0, biasZ = 0;
float upX = 0, upY = 0, upZ = 1;
float gyroZRate = 0;

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

// Rotation speed about vertical (only its size is used, for speed control).
void updateGyro() {
  if (!gyroOk) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gx = g.gyro.x * 180.0 / PI - biasX;
  float gy = g.gyro.y * 180.0 / PI - biasY;
  float gz = g.gyro.z * 180.0 / PI - biasZ;
  gyroZRate = gx * upX + gy * upY + gz * upZ;
  if (fabs(gyroZRate) < 0.3) {
    gyroZRate = 0;
  }
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

  // FIRING
  if (now - stateSince >= LASER_TIME) {
    digitalWrite(PIN_LASER, LOW);
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
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  updateGyro();
  updateIR();
  updateFire();

  // ---------- CENTRED ----------
  // Both front sensors see it: it's between them, so hold still instead of
  // chasing one of them.
  if (sees(IDX_30R) && sees(IDX_30L)) {
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

    // Drive step with gyro speed control
    unsigned long stepStart = millis();
    unsigned long last = stepStart;
    while (millis() - stepStart < TRACK_STEP_MS) {
      updateGyro();
      updateIR();
      updateFire();
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
