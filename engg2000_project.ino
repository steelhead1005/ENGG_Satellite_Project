const int inPin1 = 9;
const int inPin2 = 10;
const int irPin = A0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Nano initialized successfully.");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void testMotorSequence() {
  // Forward at 50% speed
  analogWrite(inPin1, 127);
  analogWrite(inPin2, 0);
  delay(2000);

  // Lift and Coast
  analogWrite(inPin1, 0);
  analogWrite(inPin2, 0);
  delay(1000);

  // Reverse at 50% speed
  analogWrite(inPin1, 0);
  analogWrite(inPin2, 127);
  delay(2000);

  // Brake
  analogWrite(inPin1, 255);
  analogWrite(inPin2, 255);
  delay(1000);
}

void testIRSensor() {
  int sensorValue = analogRead(irPin);
  Serial.print("Ir value: " + sensorValue);
  delay(100);
}
