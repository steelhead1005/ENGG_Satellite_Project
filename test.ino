const int frontIrPin = 3; // 0 deg
const int backIrPin  = 5; // 180 deg

const int motorPWM = 9;
const int motorDIR = 8;
const int laserPin = 6;

const int MOTOR_SPEED = 150;

bool frontDetected = false;
bool backDetected  = false;


bool triggered = false;
bool active = false;
unsigned long activeStartTime = 0;

void setup() {
  pinMode(irEmitterPin, OUTPUT);
  pinMode(frontIrPin, INPUT_PULLUP);
  pinMode(backIrPin, INPUT_PULLUP);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  digitalWrite(motorDIR, HIGH);
  analogWrite(motorPWM, 0);
  digitalWrite(laserPin, LOW);

  Serial.begin(9600);
  Serial.println("System started");
}

void loop() {
  unsigned long loopStartMicros = micros();
  unsigned long currentTime = millis();


  // READ FRONT AND BACK IR RECEIVERS
  frontDetected = (digitalRead(frontIrPin) == LOW);
  backDetected  = (digitalRead(backIrPin) == LOW);
  bool irDetected = (frontDetected || backDetected);

  // TRIGGER SPIN WHEN EITHER SENSOR DETECTS IR
  if (irDetected && !triggered) {
    triggered = true;
    active = true;
    activeStartTime = currentTime;

    // Spin HIGH if front triggered, LOW if back triggered
    digitalWrite(motorDIR, frontDetected ? HIGH : LOW);
    analogWrite(motorPWM, MOTOR_SPEED);
    digitalWrite(laserPin, HIGH);

    if (frontDetected) Serial.println("FRONT IR (PIN 3) DETECTED - MOTOR ON");
    if (backDetected)  Serial.println("BACK IR (PIN 5) DETECTED - MOTOR ON");
  }

  // STOP MOTOR & LASER AFTER 1 SECOND
  if (active && (currentTime - activeStartTime >= 3000)) {
    stopActuation();
    active = false;
    Serial.println("MOTOR OFF");
  }

  // RE-ARM AFTER 1.5 SECONDS ONCE BEAM IS GONE (prevents jitter)
  if (triggered && !active && !irDetected && (currentTime - activeStartTime >= 1500)) {
    triggered = false;
  }

  // SERIAL MONITOR TELEMETRY (Every 250ms)
  static unsigned long lastTelemetry = 0;

  if (currentTime - lastTelemetry >= 250) {
    int angle = 0;
    if (frontDetected && !backDetected) angle = 0;
    else if (!frontDetected && backDetected) angle = 180;

    Serial.print("Emitter: ");
    Serial.print(emitterOn ? "ON" : "OFF");

    Serial.print(" | IR: ");
    Serial.print(irDetected ? "DETECTED" : "NONE");

    Serial.print(" | Front(D3): ");
    Serial.print(frontDetected ? "1" : "0");

    Serial.print(" | Back(D5): ");
    Serial.print(backDetected ? "1" : "0");

    Serial.print(" | Angle: ");
    Serial.print(angle);

    Serial.print(" | Motor: ");
    Serial.print(active ? "ON" : "OFF");

    Serial.print(" | Laser: ");
    Serial.print(active ? "ON" : "OFF");

    Serial.print(" | LoopUs: ");
    Serial.println(micros() - loopStartMicros);

    lastTelemetry = currentTime;
  }
}

void stopActuation() {
  analogWrite(motorPWM, 0);
  digitalWrite(laserPin, LOW);
}
