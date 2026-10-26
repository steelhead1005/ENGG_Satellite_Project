// DC Gearmotor, FIT0186, 12 VDC, 251 rpm, incremental encoder
//
// MOTOR TEST: when any IR receiver sees a beacon, the satellite turns TURN_DEG
// degrees and stops. It re-arms once the IR has been gone for REARM_MS, so one
// detection gives one turn.
//
// The encoder measures the wheel, not the body. The wheel and body share their
// angular momentum, so the body turns roughly a fixed fraction of the wheel's
// rotation. A PID position loop on the encoder count therefore controls the
// body angle, with COUNTS_PER_BODY_DEG setting the ratio (must be calibrated).
//
// Telemetry at 115200 baud: Tools > Serial Plotter shows target, position and
// pwm during each turn. Serial Monitor also shows a summary after each turn.

// ---------------- MOTOR DRIVER (DRV8874) ----------------
// Pololu DRV8874 Single brushed driver carrier
const int PIN_DIR = 8;   // PH/IN2
const int PIN_PWM = 9;   // EN/IN1

// Must match the driver's PMODE pin (measure it with the board powered):
//   PMODE at 0 V -> true   (PH/EN mode: D8 = direction, D9 = speed)
//   PMODE at 5 V -> false  (PWM mode:   D8 = IN2, D9 = IN1)
// If this is wrong the motor drives when it should brake.
const bool DRIVER_PH_EN_MODE = false;

// ---------------- MOTOR ENCODER (Hall A/B) ----------------
const int PIN_ENC_A = 2;   // Hall sensor A, interrupt pin (INT0)
const int PIN_ENC_B = 4;   // Hall sensor B, read in ISR for direction

// Updated inside the ISR, so must be volatile.
volatile long encoderCount = 0;

// ---------------- LASER (Active HIGH) ----------------
const int PIN_LASER = 6;

// ---------------- IR SENSOR PIN DEFINITIONS (Active LOW) ----------------
// Angles are relative to laser (0 deg). Positive = clockwise/right.
const int PIN_IR_30R  = 3;   // +30  Front Right
const int PIN_IR_90R  = 10;  // +90  Right  [CRITICAL: currently hanging off bot - DISABLED]
const int PIN_IR_150R = 12;  // +150 Back Right
const int PIN_IR_150L = 7;   // -150 Back Left
const int PIN_IR_90L  = 11;  // -90  Left
const int PIN_IR_30L  = 5;   // -30  Front Left

const int SENSOR_PINS[] = {
  PIN_IR_30R,
  PIN_IR_30L,
  PIN_IR_150R,
  PIN_IR_150L,
  PIN_IR_90L,
  PIN_IR_90R,
};

const int NUM_SENSORS = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

// ---------------- TURN SETTINGS ----------------
const float TURN_DEG = 30.0;   // body rotation per IR detection. Negative = other way.

// Wheel encoder counts per degree of body rotation. Depends on the wheel and
// body inertia and the swivel friction, so it has to be measured: trigger a
// turn, measure how far the body really turned, then set this to
//   (counts moved, printed after the turn) / (degrees measured)
const float COUNTS_PER_BODY_DEG = 10.0;   // placeholder until calibrated

const unsigned long REARM_MS = 1000;          // IR must be gone this long before the next turn
const unsigned long SETTLE_MS = 200;          // wheel must sit still on target this long
const unsigned long TURN_TIMEOUT_MS = 3000;   // give up on a turn after this long

// ---------------- PID ----------------
// Starting gains from the example code; tune them on the real hardware.
//   overshoots or wobbles around target  -> raise KD or lower KP
//   stops short of target                -> raise MIN_PWM, or set KI to about 0.5
//   slow to get there                    -> raise KP or MAX_PWM
const float KP = 1.0;     // PWM per count of error
const float KI = 0.0;     // PWM per count-second of accumulated error
const float KD = 0.025;   // PWM per count/s of wheel speed
const float INTEGRAL_LIMIT = 300;   // stops the I term growing forever

const int DEADBAND_COUNTS = 4;   // close enough: the motor brakes inside this
const int MIN_PWM = 45;          // below this the motor hums but won't turn
const int MAX_PWM = 180;         // caps speed and current

const unsigned long LOOP_INTERVAL_MS = 10;   // 100 Hz control loop

// ---------------- STATE ----------------
enum State { WAITING_FOR_IR, TURNING, WAITING_FOR_CLEAR };
State state = WAITING_FOR_IR;

long targetCounts = 0;
long turnStartCounts = 0;
long prevCounts = 0;
float integral = 0;
unsigned long turnStartMs = 0;
unsigned long settleStartMs = 0;
long settleStartCounts = 0;
unsigned long lastIrSeenMs = 0;
unsigned long prevLoopMs = 0;

// On each rising edge of A, B's level tells us which way the wheel is turning.
void encoderISR() {
  if (digitalRead(PIN_ENC_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// encoderCount is 4 bytes, so pause interrupts while copying it
// to avoid reading a half-updated value.
long readEncoder() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

bool anyIrDetected() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) return true;
  }
  return false;
}

// power: -255..255. Sign sets direction, size sets PWM duty, 0 brakes.
// Positive power should make the encoder count up.
void setMotor(int power) {
  int duty = min(abs(power), 255);

  if (DRIVER_PH_EN_MODE) {
    // EN LOW brakes, so duty 0 brakes too.
    digitalWrite(PIN_DIR, power > 0 ? HIGH : LOW);
    analogWrite(PIN_PWM, duty);
  } else if (power > 0) {
    // PWM mode forward: pulse IN1 with IN2 LOW (drive / coast).
    digitalWrite(PIN_DIR, LOW);
    analogWrite(PIN_PWM, duty);
  } else {
    // PWM mode reverse: D8 can't do PWM, so hold IN2 HIGH and pulse IN1
    // inverted (IN1 LOW = drive reverse, IN1 HIGH = brake). Duty 0 = brake.
    digitalWrite(PIN_DIR, HIGH);
    analogWrite(PIN_PWM, 255 - duty);
  }
}

// Both pins LOW stops the motor in either driver mode (brake in PH/EN mode,
// coast in PWM mode), so this is safe even if DRIVER_PH_EN_MODE is wrong.
void stopMotor() {
  digitalWrite(PIN_DIR, LOW);
  analogWrite(PIN_PWM, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  stopMotor();
  digitalWrite(PIN_LASER, LOW);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, RISING);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  }

  Serial.println("Ready: waiting for IR");
  prevLoopMs = millis();
}

void loop() {
  // Run at a fixed rate without blocking.
  unsigned long now = millis();
  if (now - prevLoopMs < LOOP_INTERVAL_MS) return;
  float dt = (now - prevLoopMs) / 1000.0;
  prevLoopMs = now;

  long pos = readEncoder();
  bool irSeen = anyIrDetected();
  if (irSeen) lastIrSeenMs = now;

  switch (state) {
    case WAITING_FOR_IR:
      if (irSeen) startTurn(now, pos);
      break;

    case TURNING:
      updateTurn(now, pos, dt);
      break;

    case WAITING_FOR_CLEAR:
      if (now - lastIrSeenMs >= REARM_MS) {
        state = WAITING_FOR_IR;
        Serial.println("Ready: waiting for IR");
      }
      break;
  }

  prevCounts = pos;
}

void startTurn(unsigned long now, long pos) {
  turnStartCounts = pos;
  targetCounts = pos + lround(TURN_DEG * COUNTS_PER_BODY_DEG);
  turnStartMs = now;
  settleStartMs = now;
  settleStartCounts = pos;
  integral = 0;
  state = TURNING;
  Serial.println("IR detected: turning");
}

void updateTurn(unsigned long now, long pos, float dt) {
  long error = targetCounts - pos;   // positive = wheel needs to count up

  // Safety: an error twice the size of the turn means the motor is driving
  // the wrong way; running past the timeout means it has stalled.
  if (labs(error) > 2 * labs(targetCounts - turnStartCounts)) {
    finishTurn("ABORTED: moving the wrong way. Check DRIVER_PH_EN_MODE, then swap ++/-- in encoderISR()", now, pos);
    return;
  }
  if (now - turnStartMs > TURN_TIMEOUT_MS) {
    finishTurn("ABORTED: timed out. If the wheel stopped short, raise MIN_PWM", now, pos);
    return;
  }

  // Finished once the wheel has sat inside the deadband for SETTLE_MS,
  // moving no more than 1 count (allows for encoder jitter).
  if (labs(error) > DEADBAND_COUNTS || labs(pos - settleStartCounts) > 1) {
    settleStartMs = now;
    settleStartCounts = pos;
  } else if (now - settleStartMs >= SETTLE_MS) {
    finishTurn("Turn complete", now, pos);
    return;
  }

  int power = 0;
  if (labs(error) > DEADBAND_COUNTS) {
    integral = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    // D uses wheel speed instead of the change in error, so setting a new
    // target doesn't cause a spike.
    float speed = (pos - prevCounts) / dt;   // counts per second
    float u = KP * error + KI * integral - KD * speed;

    power = (int)constrain(u, -(float)MAX_PWM, (float)MAX_PWM);

    // Push through the motor's dead zone so small errors still move it.
    if (power > 0 && power < MIN_PWM) power = MIN_PWM;
    if (power < 0 && power > -MIN_PWM) power = -MIN_PWM;
  } else {
    integral = 0;
  }

  setMotor(power);

  Serial.print("target:");
  Serial.print(targetCounts);
  Serial.print(",position:");
  Serial.print(pos);
  Serial.print(",pwm:");
  Serial.println(power);
}

void finishTurn(const char *result, unsigned long now, long pos) {
  stopMotor();
  state = WAITING_FOR_CLEAR;

  // "moved" is the number COUNTS_PER_BODY_DEG calibration needs.
  Serial.print(result);
  Serial.print(" | moved ");
  Serial.print(pos - turnStartCounts);
  Serial.print(" of ");
  Serial.print(targetCounts - turnStartCounts);
  Serial.print(" counts in ");
  Serial.print(now - turnStartMs);
  Serial.println(" ms");
}
