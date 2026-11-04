// -------------- Adafruit MPU6050 include ----------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


// ---------------- MOTOR DRIVER (DRV8874) ----------------
const int PIN_DIR = 8;
const int PIN_PWM = 9;

// ---------------- LASER (Active HIGH) ----------------
const int PIN_LASER = 6;

// ---------------- IR SENSOR PIN DEFINITIONS (Active LOW) ----------------
// The sensors are arranged in a circle to detect a constantly on IR emitter.
// When both PIN_IR_30L and PIN_IR_30R are HIGH the laser will turn on for two seconds. 
// Angles are relative to laser (0 deg). Positive = clockwise/right.
const int PIN_IR_30R  = 3;   // +30  Front Right LASER TRIGGER TWO
const int PIN_IR_90R  = 10;  // +90  Right
const int PIN_IR_150R = 12;  // +150 Back Right
const int PIN_IR_150L = 7;   // -150 Back Left
const int PIN_IR_90L  = 11;  // -90  Left
const int PIN_IR_30L  = 5;   // -30  Front Left LASER TRIGGER TWO

// ---------------- ACTIVE SENSOR ARRAYS ----------------
const int SENSOR_PINS[] = {
  PIN_IR_30R,    // +30
  PIN_IR_30L,    // -30
  PIN_IR_150R,   // +150
  PIN_IR_150L,   // -150
  PIN_IR_90L,    // -90
  PIN_IR_90R, // +90 
};

const int SENSOR_ANGLES[] = {
  30,
  -30,
  150,
  -150,
  -90,
  90,
};

const int NUM_SENSORS = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

// ---------------- MOTOR SPEED ----------------
// Slowest PWM that still reliably overcomes mechanical stiction.
const int SLOW_PWM = 180;
bool wasSpinning = false;

// ---------------- STATE MEMORY ----------------
// Remembers the last known error angle so we can detect when the
// beacon has drifted into the 0 degree blind spot between shrouds.
int lastError = 0;

// Track current motor direction for electrical deadband
int currentDir = -1;

void setup() {
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  }

  digitalWrite(PIN_LASER, LOW);
  digitalWrite(PIN_PWM, 0);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU Detected");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  bool found = false;
  int bestAngle = 0;
  int bestAbs = 9999;

  // 1. SCAN SENSORS
  // Find the triggered sensor closest to 0 degrees
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) { 
    	Serial.print("Array value:");
    	Serial.print(i);
	Serial.print("       PIN NUMBER:  ");
	Serial.print(SENSOR_PINS[i]);
	
      int absAngle = abs(SENSOR_ANGLES[i]);
      if (absAngle < bestAbs) {
        bestAbs = absAngle;
        bestAngle = SENSOR_ANGLES[i];
        found = true;
      }
    }
  }
  
// 2. OVERLAP TARGET LOCK
  bool frontRight = (digitalRead(PIN_IR_30R) == LOW);
  bool frontLeft  = (digitalRead(PIN_IR_30L) == LOW);

  if (frontRight && frontLeft) {
    if (wasSpinning) {
      // HEURISTIC We just hit the edge of the overlap zone. 
      // Keep spinning in the current direction for a few milliseconds
      // to push closer to the true center before braking.
      delay(80);

      // 1. Coast Deadband: Drop power to 0 to clear the electrical spike
      analogWrite(PIN_PWM, 0); // Motor pauses here 
      delay(30); 
        
      // 2. Counter-torque brake: Instantly flip direction and pulse power
      digitalWrite(PIN_DIR, !digitalRead(PIN_DIR)); 
      analogWrite(PIN_PWM, 255); 
      delay(50);                 // Brake duration
      wasSpinning = false;
    }
    
    analogWrite(PIN_PWM, 0);
    digitalWrite(PIN_LASER, HIGH);
    delay(2000);
    return;
  }

  // 3. TRACKING
  if (found) {
    digitalWrite(PIN_LASER, LOW);
    wasSpinning = true;

    if (bestAngle > 0) {
      digitalWrite(PIN_DIR, HIGH); // Target is Right
    } else {
      digitalWrite(PIN_DIR, LOW);  // Target is Left
    }
    
    analogWrite(PIN_PWM, SLOW_PWM);
    
    // Minimal pulse bridge to ignore the 38kHz IR flashing gaps
    delay(150); 
    return;
  }

  // 4. LOSS OF TARGET
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_LASER, LOW);
}
