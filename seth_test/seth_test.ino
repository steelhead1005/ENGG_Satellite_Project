#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// ---------------- MOTOR DRIVER ----------------
const int PIN_DIR = 8;
const int PIN_PWM = 9;

// ---------------- LASER ----------------
const int PIN_LASER = 6;

// ---------------- IR SENSORS ----------------
// Positive = right
// Negative = left
const int PIN_IR_30R  = 3;
const int PIN_IR_90R  = 10;
const int PIN_IR_150R = 12;
const int PIN_IR_150L = 7;
const int PIN_IR_90L  = 11;
const int PIN_IR_30L  = 5;

const int SENSOR_PINS[] = {
  PIN_IR_30R,
  PIN_IR_30L,
  PIN_IR_150R,
  PIN_IR_150L,
  PIN_IR_90L,
  PIN_IR_90R
};

const int SENSOR_ANGLES[] = {
  30,
  -30,
  150,
  -150,
  -90,
  90
};

const int NUM_SENSORS = 6;

// ---------------- MOTOR SPEED ----------------
const int TRACK_PWM = 220;

// ---------------- GYRO CORRECTION ----------------
const float ANGLE_TOLERANCE = 2.5;
const int CORRECTION_PWM = 220;
const unsigned long CORRECTION_PULSE = 40;
const unsigned long CORRECTION_COAST = 50;
const unsigned long MAX_RELOCATE_TIME = 1500;

const bool CORRECTION_INVERT = true;

// ---------------- LASER ----------------
const unsigned long LASER_TIME = 2000;

// ---------------- MOTOR STATE ----------------
bool wasSpinning = false;
int currentDir = -1;

// ---------------- TARGET STATE ----------------
bool targetHandled = false;
float targetAngle = 0;

// ---------------- GYRO STATE ----------------
float gyroZOffset = 0;
float gyroZRate = 0;
float angleZ = 0;

unsigned long previousGyroTime = 0;


// =====================================================
// UPDATE GYRO
// =====================================================

void updateGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();

  float dt = (now - previousGyroTime) / 1000000.0;

  previousGyroTime = now;

  gyroZRate = g.gyro.z * 180.0 / PI;

  gyroZRate -= gyroZOffset;

  if (fabs(gyroZRate) < 0.3) {
    gyroZRate = 0;
  }

  angleZ += gyroZRate * dt;
}


// =====================================================
// GYRO-AWARE DELAY
// =====================================================

void gyroDelay(unsigned long duration) {
  unsigned long start = millis();

  while (millis() - start < duration) {
    updateGyro();
    delay(2);
  }
}


// =====================================================
// STOP MOTOR
// =====================================================

void stopMotor() {
  analogWrite(PIN_PWM, 0);
}


// =====================================================
// CORRECTION PULSE
// =====================================================

void correctAngle(float error) {
  bool direction;

  if (error > 0) {
    direction = HIGH;
  } else {
    direction = LOW;
  }

  if (CORRECTION_INVERT) {
    direction = !direction;
  }

  digitalWrite(PIN_DIR, direction);
  currentDir = direction;

  analogWrite(PIN_PWM, CORRECTION_PWM);

  gyroDelay(CORRECTION_PULSE);

  stopMotor();

  gyroDelay(CORRECTION_COAST);
}


// =====================================================
// RETURN TO SAVED TARGET
// =====================================================

bool relocateToTarget() {
  unsigned long startTime = millis();

  while (true) {
    updateGyro();

    float error = targetAngle - angleZ;

    Serial.print("TARGET: ");
    Serial.print(targetAngle, 2);

    Serial.print(" | CURRENT: ");
    Serial.print(angleZ, 2);

    Serial.print(" | ERROR: ");
    Serial.println(error, 2);

    if (fabs(error) <= ANGLE_TOLERANCE) {
      stopMotor();

      Serial.println("TARGET ANGLE REACHED");

      return true;
    }

    if (millis() - startTime > MAX_RELOCATE_TIME) {
      stopMotor();

      Serial.println("RELOCATION TIMEOUT");

      return false;
    }

    correctAngle(error);
  }
}


// =====================================================
// LASER FIRE + CONTINUOUS ANGLE HOLD
// =====================================================

void fireLaserAndHold() {
  Serial.println("*** LASER ON ***");

  digitalWrite(PIN_LASER, HIGH);

  unsigned long fireStart = millis();

  while (millis() - fireStart < LASER_TIME) {
    updateGyro();

    float error = targetAngle - angleZ;

    if (fabs(error) > ANGLE_TOLERANCE) {
      correctAngle(error);
    } else {
      stopMotor();
      delay(2);
    }

    Serial.print("FIRING | Target: ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Current: ");
    Serial.print(angleZ, 2);

    Serial.print(" | Error: ");
    Serial.println(error, 2);
  }

  digitalWrite(PIN_LASER, LOW);

  stopMotor();

  Serial.println("*** LASER OFF ***");
}


// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  }

  stopMotor();

  digitalWrite(PIN_LASER, LOW);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");

    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU Detected");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating gyro - KEEP STILL");

  float total = 0;

  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    total += g.gyro.z * 180.0 / PI;

    delay(5);
  }

  gyroZOffset = total / 500.0;

  angleZ = 0;
  previousGyroTime = micros();

  Serial.println("SYSTEM READY");
}


// =====================================================
// LOOP
// =====================================================

void loop() {
  updateGyro();

  bool found = false;
  int bestAngle = 0;
  int bestAbs = 9999;

  // 1. SCAN SENSORS
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) {
      Serial.print("IR PIN: ");
      Serial.println(SENSOR_PINS[i]);

      int absAngle = abs(SENSOR_ANGLES[i]);

      if (absAngle < bestAbs) {
        bestAbs = absAngle;
        bestAngle = SENSOR_ANGLES[i];
        found = true;
      }
    }
  }

  // 2. FRONT OVERLAP
  bool frontRight = digitalRead(PIN_IR_30R) == LOW;
  bool frontLeft = digitalRead(PIN_IR_30L) == LOW;

  bool overlap = frontRight && frontLeft;

  if (!overlap) {
    targetHandled = false;
  }

  // 3. NEW TARGET LOCK
  if (overlap && !targetHandled) {
    updateGyro();

    targetAngle = angleZ;
    targetHandled = true;

    Serial.println();
    Serial.println("*** TARGET DETECTED ***");

    Serial.print("Saved target angle: ");
    Serial.println(targetAngle, 2);

    // Brake existing motion
    if (wasSpinning) {
      gyroDelay(80);

      stopMotor();

      gyroDelay(30);

      bool brakeDirection = !currentDir;

      digitalWrite(PIN_DIR, brakeDirection);
      analogWrite(PIN_PWM, 255);

      gyroDelay(50);

      stopMotor();

      currentDir = brakeDirection;
      wasSpinning = false;
    }

    Serial.print("Post-brake angle: ");
    Serial.println(angleZ, 2);

    Serial.println("Relocating to target...");

    bool aligned = relocateToTarget();

    if (aligned) {
      fireLaserAndHold();
    }

    return;
  }

  // 4. TARGET ALREADY FIRED
  if (overlap && targetHandled) {
    stopMotor();

    digitalWrite(PIN_LASER, LOW);

    return;
  }

  // 5. NORMAL IR TRACKING
  if (found) {
    digitalWrite(PIN_LASER, LOW);

    wasSpinning = true;

    if (bestAngle > 0) {
      digitalWrite(PIN_DIR, HIGH);
      currentDir = HIGH;
    } else {
      digitalWrite(PIN_DIR, LOW);
      currentDir = LOW;
    }

    analogWrite(PIN_PWM, TRACK_PWM);

    gyroDelay(150);

    return;
  }

  // 6. LOSS OF TARGET
  stopMotor();

  digitalWrite(PIN_LASER, LOW);

  wasSpinning = false;
}