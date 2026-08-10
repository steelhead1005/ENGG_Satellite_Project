// MQ Sentinel - motor bring-up test
// Drives the reaction wheel alternately CW and CCW with ramped
// transitions. No blocking calls; safe to extend into the control loop.

const uint8_t PIN_PWM     = 9;   // -> EN/IN1  (Timer1)
const uint8_t PIN_DIR     = 7;   // -> PH/IN2
const uint8_t PIN_SLEEP   = 8;   // -> SLEEP
const uint8_t PIN_FAULT   = 5;   // <- nFAULT, active low
const uint8_t PIN_ENC_A   = 2;   // <- Hall A, interrupt
const uint8_t PIN_ENC_B   = 4;   // <- Hall B

const int  CRUISE_PWM   = 120;   // 0-255, start low
const int  RAMP_STEP    = 3;     // PWM units per control tick
const unsigned long TICK_MS  = 10;    // 100 Hz control tick
const unsigned long PHASE_MS = 2000;  // time at cruise before reversing

volatile long encoderCount = 0;

int  targetPwm  = 0;
int  currentPwm = 0;
bool forward    = true;

unsigned long lastTick  = 0;
unsigned long phaseStart = 0;

void encoderISR() {
  // A has just changed. B's level tells us which way.
  if (digitalRead(PIN_ENC_B)) encoderCount++;
  else                        encoderCount--;
}

// Command the motor with a signed value: sign = direction, magnitude = effort.
void setMotor(int signedPwm) {
  signedPwm = constrain(signedPwm, -255, 255);
  digitalWrite(PIN_DIR, signedPwm >= 0 ? HIGH : LOW);
  analogWrite(PIN_PWM, abs(signedPwm));
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  pinMode(PIN_FAULT, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // Outputs to a safe state BEFORE the driver wakes.
  digitalWrite(PIN_SLEEP, LOW);
  digitalWrite(PIN_DIR, HIGH);
  analogWrite(PIN_PWM, 0);

  // Timer1 -> ~31 kHz PWM on D9/D10. Above hearing, smoother torque.
  // Do NOT touch Timer0; millis() depends on it.
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, RISING);

  delay(10);                      // PMODE settled low before enable
  digitalWrite(PIN_SLEEP, HIGH);  // driver latches PH/EN mode here

  phaseStart = millis();
  targetPwm  = CRUISE_PWM;
}

void loop() {
  unsigned long now = millis();
  if (now - lastTick < TICK_MS) return;
  lastTick = now;

  if (digitalRead(PIN_FAULT) == LOW) {
    setMotor(0);
    digitalWrite(PIN_SLEEP, LOW);
    Serial.println(F("FAULT - driver disabled"));
    return;
  }

  // Ramp toward the target rather than stepping. A hard reversal
  // draws far more current than the driver is rated to deliver.
  if (currentPwm < targetPwm)      currentPwm = min(currentPwm + RAMP_STEP, targetPwm);
  else if (currentPwm > targetPwm) currentPwm = max(currentPwm - RAMP_STEP, targetPwm);

  setMotor(forward ? currentPwm : -currentPwm);

  if (targetPwm > 0 && now - phaseStart > PHASE_MS) {
    targetPwm = 0;                       // decelerate first
  } else if (targetPwm == 0 && currentPwm == 0) {
    forward = !forward;                  // only flip once stopped
    targetPwm = CRUISE_PWM;
    phaseStart = now;
  }

  static uint8_t n = 0;
  if (++n >= 20) {                       // ~5 Hz telemetry
    n = 0;
    noInterrupts();
    long c = encoderCount;
    interrupts();
    Serial.print(F("dir=")); Serial.print(forward ? F("CW ") : F("CCW"));
    Serial.print(F(" pwm=")); Serial.print(currentPwm);
    Serial.print(F(" enc=")); Serial.println(c);
  }
}