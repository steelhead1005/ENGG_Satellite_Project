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
}

void loop() {
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
    Serial.print("Emitter: ");
    Serial.print(emitterOn ? "ON" : "OFF");

    Serial.print(" | IR: ");
    Serial.print(irDetected ? "DETECTED" : "NONE");

// -------------------------------------------------------
// basic implementation of multi reciever telemetry
// -------------------------------------------------------

//    for (int i =0; i<=7;i++){
        // Serial.print(strcar(" | IR");
        //Serial.print(i, DEC);
        // Serial.print(": ");
//      Serial.print(irReadings[i] ? "DETECTED" : "NONE");
//    }

    Serial.print(" | Motor: ");
    Serial.print(active ? "ON" : "OFF");

    Serial.print(" | Laser: ");
    Serial.println(active ? "ON" : "OFF");

    lastTelemetry = currentTime;
  }
}

// reads the IR receiver pin (always active, never switched off)
bool checkTargetDetected() {
  return (digitalRead(irReceiverPin) == LOW); // true if target is detected
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

// ---------------------------------------------------------------------------------------------------
//   To be changed, edited or removed completley upon clearer hardware implementation
// ---------------------------------------------------------------------------------------------------


// void updateRecieverValues(){
//   irReadingsdigitalRead={(irReceiverPin) == LOW, (irReceiverPin1) == LOW, (irReceiverPin2) == LOW, (irReceiverPin3) == LOW, (irReceiverPin4) == LOW, (irReceiverPin5) == LOW, (irReceiverPin6) == LOW, (irReceiverPin7) == LOW};
// }

