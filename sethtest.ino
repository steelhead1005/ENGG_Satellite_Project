const int irPin = 3;

const int motorPWM = 9;
const int motorDIR = 8;

bool triggered = false;

void setup() {
  pinMode(irPin, INPUT);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  Serial.begin(9600);

  analogWrite(motorPWM, 0);
}

void loop() {
  int irState = digitalRead(irPin);

  // IR detected and hasn't triggered yet
  if (irState == LOW && triggered == false) {

    Serial.println("IR DETECTED");

    triggered = true;

    // HIGH = your counter-clockwise direction
    digitalWrite(motorDIR, HIGH);
    analogWrite(motorPWM, 120);

    delay(1000);

    // Stop motor
    analogWrite(motorPWM, 0);

    Serial.println("Motor stopped");
  }

  // Re-arm only once IR is no longer detected
  if (irState == HIGH) {
    triggered = false;
  }
}
