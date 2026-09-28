#include <EEPROM.h>

// Set to false for Test Run 1 (V1 Baseline), then true for Test Run 2 (V2 Aiming)
const bool USE_V2_AIMING = true;

const int irEmitterPin = 7;

const int irPins[6] = {3, 12, 4, 5, 10, 11}; 
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

int eepromAddr = 0;

void setup() {
  pinMode(irEmitterPin, OUTPUT);
  for (int i = 0; i < 6; i++) {
    pinMode(irPins[i], INPUT_PULLUP);
  }

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);
  pinMode(laserPin, OUTPUT);

  digitalWrite(motorDIR, HIGH);
  analogWrite(motorPWM, 0);
  digitalWrite(laserPin, LOW);

  Serial.begin(9600);

  // 5-second window on startup: if plugged into laptop and Python sends 'D', dump EEPROM
  unsigned long waitStart = millis();
  while (millis() - waitStart < 5000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'D' || c == 'd') {
        dumpEEPROM();
        while (true); // Halt here after dumping so we don't overwrite data
      }
    }
  }

  // No 'D' received -> We are on the hanging rig! Mark start of new recording
  EEPROM.update(0, 255);
  Serial.println("System started");
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
    } else {
      noTone(irEmitterPin);
      digitalWrite(irEmitterPin, LOW);
    }
  }

  // ---------------------------------
  // READ IR RECEIVERS
  // ---------------------------------
  bool irDetected = checkTargetDetected();

  // ---------------------------------
  // ACTUATION (V1 vs V2 Switch)
  // ---------------------------------
  if (!USE_V2_AIMING) {
    // V1 Baseline: blind 1-second motor + laser burst
    if (irDetected && !triggered) {
      triggered = true;
      active = true;
      activeStartTime = currentTime;
      triggerActuation();
    }
    if (active && currentTime - activeStartTime >= 1000) {
      stopActuation();
      active = false;
    }
    if (!irDetected) {
      triggered = false;
    }
  } else {
    // V2 Aiming: spin toward targetAngle, brake and fire laser on Sensor 0 (0 deg)
    float targetAngle = getTargetAngle();
    bool frontAligned = irReadings[0];

    if (irDetected && !frontAligned) {
      active = true;
      digitalWrite(motorDIR, (targetAngle > 0) ? HIGH : LOW);
      analogWrite(motorPWM, MOTOR_SPEED);
      digitalWrite(laserPin, LOW);
    }

    if (frontAligned && !triggered) {
      triggered = true;
      active = false;
      activeStartTime = currentTime;

      digitalWrite(motorDIR, (targetAngle > 0) ? LOW : HIGH);
      analogWrite(motorPWM, 140);
      delay(80);

      analogWrite(motorPWM, 0);
      digitalWrite(laserPin, HIGH);
    }

    if (triggered && currentTime - activeStartTime >= 2000) {
      stopActuation();
    }
    if (!irDetected) {
      triggered = false;
      active = false;
      stopActuation();
    }
  }

  // ---------------------------------
  // TELEMETRY & EEPROM RECORDING (Every 250ms, up to 64 seconds)
  // ---------------------------------
  static unsigned long lastTelemetry = 0;

  if (currentTime - lastTelemetry >= 250) {
    uint16_t loopUs = micros() - loopStartMicros;
    bool laserOn = (digitalRead(laserPin) == HIGH);

    if (eepromAddr <= 1020) {
      byte sensorByte = 0;
      for (int i = 0; i < 6; i++) {
        if (irReadings[i]) bitSet(sensorByte, i);
      }

      byte stateByte = 0;
      if (emitterOn) bitSet(stateByte, 0);
      if (active)    bitSet(stateByte, 1);
      if (laserOn)   bitSet(stateByte, 2);

      EEPROM.update(eepromAddr,     sensorByte);
      EEPROM.update(eepromAddr + 1, stateByte);
      EEPROM.update(eepromAddr + 2, highByte(loopUs));
      EEPROM.update(eepromAddr + 3, lowByte(loopUs));
      eepromAddr += 4;

      if (eepromAddr <= 1020) {
        EEPROM.update(eepromAddr, 255); // End-of-log marker
      }
    }

    lastTelemetry = currentTime;
  }
}

void dumpEEPROM() {
  for (int addr = 0; addr <= 1020; addr += 4) {
    byte sensorByte = EEPROM.read(addr);
    if (sensorByte == 255) break; // Reached end of recorded run

    byte stateByte = EEPROM.read(addr + 1);
    uint16_t savedLoopUs = word(EEPROM.read(addr + 2), EEPROM.read(addr + 3));

    for (int i = 0; i < 6; i++) {
      irReadings[i] = bitRead(sensorByte, i);
    }
    bool emOn  = bitRead(stateByte, 0);
    bool motOn = bitRead(stateByte, 1);
    bool lasOn = bitRead(stateByte, 2);
    bool irDet = (sensorByte > 0);

    Serial.print("Emitter: "); Serial.print(emOn ? "ON" : "OFF");
    Serial.print(" | IR: ");   Serial.print(irDet ? "DETECTED" : "NONE");
    Serial.print(" | Array: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(irReadings[i] ? "1" : "0");
    }
    Serial.print(" | Angle: ");  Serial.print(getTargetAngle());
    Serial.print(" | Motor: ");  Serial.print(motOn ? "ON" : "OFF");
    Serial.print(" | Laser: ");  Serial.print(lasOn ? "ON" : "OFF");
    Serial.print(" | LoopUs: "); Serial.println(savedLoopUs);
  }
  Serial.println("DUMP_COMPLETE");
}

bool checkTargetDetected() {
  updateReceiverValues();
  for (int i = 0; i < 6; i++) {
    if (irReadings[i]) return true;
  }
  return false;
}

void triggerActuation() {
  digitalWrite(motorDIR, HIGH);
  analogWrite(motorPWM, MOTOR_SPEED);
  digitalWrite(laserPin, HIGH);
}

void stopActuation() {
  analogWrite(motorPWM, 0);
  digitalWrite(laserPin, LOW);
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
