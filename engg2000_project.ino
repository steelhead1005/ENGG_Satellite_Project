const int irEmitterPin = 5;
const int irReceiverPin = 3;

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
  pinMode(irReceiverPin, INPUT_PULLUP);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  pinMode(laserPin, OUTPUT);

  // Your current motor wiring
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

  bool irDetected = (digitalRead(irReceiverPin) == LOW);

  // ---------------------------------
  // IR DETECTED
  // MOTOR + LASER ON
  // ---------------------------------

  if (irDetected && !triggered) {
    triggered = true;
    active = true;

    activeStartTime = currentTime;

    // Motor ON
    digitalWrite(motorDIR, HIGH);
    analogWrite(motorPWM, MOTOR_SPEED);

    // Laser ON
    digitalWrite(laserPin, HIGH);

    Serial.println("IR DETECTED");
    Serial.println("MOTOR ON");
    Serial.println("LASER ON");
  }

  // ---------------------------------
  // AFTER 1 SECOND
  // MOTOR + LASER OFF
  // ---------------------------------

  if (active && currentTime - activeStartTime >= 1000) {
    analogWrite(motorPWM, 0);

    digitalWrite(laserPin, LOW);

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

    Serial.print(" | Motor: ");
    Serial.print(active ? "ON" : "OFF");

    Serial.print(" | Laser: ");
    Serial.println(active ? "ON" : "OFF");

    lastTelemetry = currentTime;
  }
}
