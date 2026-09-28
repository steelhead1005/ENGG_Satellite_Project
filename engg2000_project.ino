#include <EEPROM.h>
int eepromAddr = 0;
bool isRecording = true; // Set false when dumping

const int irEmitterPin = 5;

const int irPins[6] = {2, 3, 4, 7, 10, 11}; 
const int sensorAngles[6] = {0, 60, 120, 180, -120, -60};
bool irReadings[6] = {false, false, false, false, false, false};

const int motorPWM = 9;
const int motorDIR = 8;

const int laserPin = 6;

const int MOTOR_SPEED = 120;

bool emitterOn = false;
unsigned long lastEmitterChange = 0;

bool triggered = false;
bool active = false;

unsigned long activeStartTime = 0;

void setup() {
  pinMode(irEmitterPin, OUTPUT);
  for (int i = 0; i < 6; i++) {
    pinMode(irPins[i], INPUT_PULLUP);
  }

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  pinMode(laserPin, OUTPUT);

  // current motor wiring
  // HIGH = counter-clockwise
  digitalWrite(motorDIR, HIGH);

  analogWrite(motorPWM, 0);
  digitalWrite(laserPin, LOW);

  Serial.begin(9600);

  Serial.println("System started");
  Serial.println("Send 'D' within 5 seconds to dump saved EEPROM run, or wait to start new recording...");
  unsigned long waitStart = millis();
  while (millis() - waitStart < 5000) {
    if (Serial.available() > 0 && (Serial.read() == 'D' || Serial.read() == 'd')) {
      isRecording = false;
      for (int addr = 0; addr < 1024; addr += 4) {
        byte packedState = EEPROM.read(addr);
        if (packedState == 255) break; // End of recorded data

        int8_t savedAngle = (int8_t)EEPROM.read(addr + 1);
        uint16_t savedLoopUs = word(EEPROM.read(addr + 2), EEPROM.read(addr + 3));

        bool emOn  = bitRead(packedState, 7);
        bool motOn = bitRead(packedState, 6);
        byte arr   = packedState & 0b00111111;

        Serial.print("Emitter: "); Serial.print(emOn ? "ON" : "OFF");
        Serial.print(" | IR: ");   Serial.print(arr > 0 ? "DETECTED" : "NONE");
        Serial.print(" | Array: ");
        for (int i = 0; i < 6; i++) Serial.print(bitRead(arr, i) ? "1" : "0");
        Serial.print(" | Angle: "); Serial.print(savedAngle);
        Serial.print(" | Motor: "); Serial.print(motOn ? "ON" : "OFF");
        Serial.print(" | Laser: "); Serial.print(motOn ? "ON" : "OFF");
        Serial.print(" | LoopUs: "); Serial.println(savedLoopUs);
      }
      while (true); // Stop here after dumping
    }
  }
}

void loop() {
  unsigned long loopStartMicros = micros();
  unsigned long currentTime = millis();

  // ---------------------------------
  // IR EMITTER: 5 sec ON / 5 sec OFF
  // ---------------------------------

  if (currentTime - lastEmitterChange >= 5000) {
    lastEmitterChange = currentTime;

    emitterOn = !emitterOn;

    if (emitterOn) {
      tone(irEmitterPin, 38000);
      Serial.println("EMITTER ON");
    } 
    else {
      noTone(irEmitterPin);
      digitalWrite(irEmitterPin, LOW);

      Serial.println("EMITTER OFF");
    }
  }

  // ---------------------------------
  // READ IR RECEIVER
  // ---------------------------------

  bool irDetected = checkTargetDetected();

  // ---------------------------------
  // IR DETECTED
  // MOTOR + LASER ON
  // ---------------------------------

  if (irDetected && !triggered) {
    triggered = true;
    active = true;

    activeStartTime = currentTime;
    triggerActuation();

    Serial.println("IR DETECTED");
    Serial.println("MOTOR ON");
    Serial.println("LASER ON");
  }

  // ---------------------------------
  // AFTER 1 SECOND
  // MOTOR + LASER OFF
  // ---------------------------------

  if (active && currentTime - activeStartTime >= 1000) {
    stopActuation();
    active = false;

    Serial.println("MOTOR OFF");
    Serial.println("LASER OFF");
  }

  // ---------------------------------
  // RE-ARM WHEN IR IS GONE
  // ---------------------------------

  if (!irDetected) {
    triggered = false;
  }

  // ---------------------------------
  // TELEMETRY
  // ---------------------------------

  static unsigned long lastTelemetry = 0;

  if (currentTime - lastTelemetry >= 250) {
    if (isRecording && eepromAddr <= 1020) {
      byte packed = 0;
      for (int i = 0; i < 6; i++) {
        if (irReadings[i]) bitSet(packed, i);
      }
      if (active)    bitSet(packed, 6);
      if (emitterOn) bitSet(packed, 7);

      uint16_t loopUs = micros() - loopStartMicros;
      EEPROM.update(eepromAddr,     packed);
      EEPROM.update(eepromAddr + 1, (byte)((int8_t)getTargetAngle()));
      EEPROM.update(eepromAddr + 2, highByte(loopUs));
      EEPROM.update(eepromAddr + 3, lowByte(loopUs));
      eepromAddr += 4;
    }

    Serial.print("Emitter: ");
    Serial.print(emitterOn ? "ON" : "OFF");

    Serial.print(" | IR: ");
    Serial.print(irDetected ? "DETECTED" : "NONE");

    Serial.print(" | Array: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(irReadings[i] ? "1" : "0");
    }

    Serial.print(" | Angle: ");
    Serial.print(getTargetAngle());

    Serial.print(" | Motor: ");
    Serial.print(active ? "ON" : "OFF");

    Serial.print(" | Laser: ");
    Serial.print(digitalRead(laserPin) == HIGH ? "ON" : "OFF");

    Serial.print(" | LoopUs: ");
    Serial.println(micros() - loopStartMicros);

    lastTelemetry = currentTime;
  }
}

bool checkTargetDetected() {
  updateReceiverValues();
  for (int i = 0; i < 6; i++) {
    if (irReadings[i]) return true;
  }
  return false;
}

void triggerActuation() {
  digitalWrite(motorDIR, HIGH);       // set spin direction
  analogWrite(motorPWM, MOTOR_SPEED); // turn motor on
  digitalWrite(laserPin, HIGH);       // turn laser on
}

void stopActuation() {
  analogWrite(motorPWM, 0);     // stop the motor
  digitalWrite(laserPin, LOW);  // turn the laser off
}

void updateReceiverValues() {
  for (int i = 0; i < 6; i++) {
    irReadings[i] = (digitalRead(irPins[i]) == LOW);
  }
}

float getTargetAngle() {
  float sumX = 0.0;
  float sumY = 0.0;
  int activeCount = 0;

  for (int i = 0; i < 6; i++) {
    if (irReadings[i]) {
      float rad = sensorAngles[i] * (PI / 180.0);
      sumX += cos(rad);
      sumY += sin(rad);
      activeCount++;
    }
  }

  if (activeCount == 0) return 0.0;
  return atan2(sumY, sumX) * (180.0 / PI);
}
