/*
  ENGG2000 satellite

  RUN_MODE picks what the sketch does:
    V1_BASELINE  Test Run 1: IR detected -> blind 1 second motor + laser burst
    V2_AIMING    Test Run 2: spin toward the IR bearing, then brake and fire
                 the laser once sensor 0 (0 deg) lines up
    STEP_TEST    bench test for the encoder PID: rotate 30 degrees, stop,
                 fire, repeat (see STEP TEST further down)

  Arduino Nano
  Pololu DRV8874 carrier  (PH/EN mode -- PMODE must be tied to GND)
  DFRobot FIT0186 gearmotor, 700 encoder counts per output-shaft revolution

  Library: "Encoder" by Paul Stoffregen (Library Manager)

  WIRING
    IR receivers: 0 deg -> D12, 60 deg -> A0, 120 deg -> D4,
                  180 deg -> D7, -120 deg -> D10, -60 deg -> D11
                  (0 and 60 deg used to be on D2/D3, but the encoder needs
                  the Nano's only two interrupt pins)
    IR emitter:   D5
    Encoder:      Vcc -> 5V, GND -> GND, A -> D2, B -> D3
    Driver:       EN/IN1 -> D9, PH/IN2 -> D8, nSLEEP -> 5V, PMODE -> GND
                  VIN -> 12V pack (+), GND -> pack (-) AND Arduino GND
                  OUT1/OUT2 -> motor red/black
    Laser:        signal -> D6 (HIGH = on)
    Do NOT power the motor from the Arduino's 5V rail.
    Battery ground, driver ground and Arduino ground must all be common.

  TELEMETRY
    V1/V2 record to EEPROM every 250 ms (about 64 s per run). To download a
    run, plug in the laptop and run logger.py before anything else: powering
    up without it starts a new recording over the old one.
    STEP_TEST doesn't record, so a bench test can't wipe a run. It prints to
    Tools > Serial Plotter instead (9600 baud).
*/

#include <EEPROM.h>
#include <Encoder.h>

// V1_BASELINE for Test Run 1, V2_AIMING for Test Run 2, STEP_TEST for the encoder PID bench test
enum RunMode { V1_BASELINE, V2_AIMING, STEP_TEST };
const RunMode RUN_MODE = V2_AIMING;

const int irEmitterPin = 5;

const int irPins[6] = {12, A0, 4, 7, 10, 11};
const int sensorAngles[6] = {0, 60, 120, 180, -120, -60};
bool irReadings[6] = {false, false, false, false, false, false};

const int motorPWM = 9;
const int motorDIR = 8;
const int laserPin = 6;
const int encoderPinA = 2;   // must be an interrupt pin
const int encoderPinB = 3;   // must be an interrupt pin

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

  if (RUN_MODE == STEP_TEST) {
    setupStepTest();
  } else {
    // No 'D' received -> We are on the hanging rig! Mark start of new recording
    EEPROM.update(0, 255);
  }
  Serial.println("System started");
}

void loop() {
  if (RUN_MODE == STEP_TEST) {
    runStepTest();
    return;
  }

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
  if (RUN_MODE == V1_BASELINE) {
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

// =====================================================================
//   STEP TEST: rotate, stop, fire, repeat
// =====================================================================
/*
  The motor turns STEP_ANGLE_DEGREES (30 degrees by default) and waits until
  it has come to a complete stop. Then the laser fires for LASER_ON_TIME_MS
  (1 second). As soon as the laser switches off the motor turns the next
  30 degrees, and this repeats forever.

  Each turn is smooth: instead of jumping straight to the new angle, a target
  speeds up, cruises, then slows down to land on it, and a PID controller
  drives the motor to follow that target. The gentle start and stop keeps a
  heavy flywheel from building up too much momentum and overshooting.

  If the motor spins non-stop at start-up, the encoder is counting the
  wrong way: swap the encoder A and B wires.

  THE SEQUENCE (repeats forever)
    ROTATING   the goal has moved on 30 degrees; the smooth target travels to it
    SETTLING   the target has arrived; wait for the shaft to stop on the goal
    FIRING     laser on for 1 second while the motor holds still, then laser
               off and back to ROTATING for the next turn

  WHAT runStepTest() DOES, IN ORDER
    1. Wait until it's time for the next update (every 10 ms).
    2. Read the encoder to see where the shaft is.
    3. Move the sequence along: spot when the shaft has stopped, switch the
       laser on and off, start the next turn.
    4. Move the smooth target a little closer to the goal.
    5. Work out the error (target - actual).
    6. Use PID to turn that error into a motor command.
    7. Send the command to the motor.
    8. Print goal, target and actual angle (optional).
*/

// ---- Rotate / stop / fire sequence ----
const float STEP_ANGLE_DEGREES = 30.0;         // how far each turn goes. Negative = reverse.
const unsigned long LASER_ON_TIME_MS = 1000;   // how long the laser fires at each stop
const unsigned long SETTLE_TIME_MS = 200;      // how long the shaft must sit still inside the
                                               // deadband before it counts as stopped

// ---- Motion limits: how the shaft gets to each new angle ----
// Heavier flywheel -> lower MAX_ACCELERATION, so the motor can always brake in time.
// With these values a 30 degree turn takes about 0.45 s and a 90 degree turn about 0.8 s.
// Set both very high to go back to jumping straight to the goal.
const float MAX_SPEED = 180.0;          // degrees per second
const float MAX_ACCELERATION = 720.0;   // degrees per second, per second

// ---- PID gains ----
// Gains only suit the load they were tuned with, so retune after changing the flywheel.
// Change one at a time, in small steps, and watch Tools > Serial Plotter:
//   overshoots or wobbles around the goal     -> raise DERIVATIVE_GAIN, or lower PROPORTIONAL_GAIN
//   stops short of the goal                   -> raise INTEGRAL_GAIN, or lower DERIVATIVE_GAIN
//   lags well behind the target while moving  -> raise PROPORTIONAL_GAIN
const float PROPORTIONAL_GAIN = 1.5;   // P: push harder the further we are from the target
const float INTEGRAL_GAIN = 0.4;       // I: push harder the longer we stay short of the target
const float DERIVATIVE_GAIN = 0.03;    // D: push to match the target's speed (brakes a runaway flywheel)
const float INTEGRAL_LIMIT = 300;      // stops the I term growing forever

// ---- Motor limits ----
const int DEADBAND_COUNTS = 4;   // "close enough" zone (4 counts ~ 2 degrees); motor rests inside it
const int MAX_PWM = 180;         // 0-255. Caps current. Raise if too weak.
const int MIN_PWM = 45;          // below this the motor buzzes but won't turn

// ---- Serial Plotter output ----
const bool DEBUG = true;
const unsigned long PRINT_INTERVAL_MS = 100;   // every 50 ms would nearly fill the 9600 baud link logger.py needs

// ---- Encoder and timing (only change if the motor changes) ----
const float ENCODER_COUNTS_PER_REVOLUTION = 700;   // FIT0186 output shaft
const float COUNTS_PER_DEGREE = ENCODER_COUNTS_PER_REVOLUTION / 360.0;

const unsigned long CONTROL_LOOP_INTERVAL_MS = 10;  // 100 updates per second

Encoder motorEncoder(encoderPinA, encoderPinB);

// ---------------- values that change while running ----------------
enum SequenceState { ROTATING, SETTLING, FIRING };   // see THE SEQUENCE above
SequenceState sequenceState = ROTATING;

float goalAngleDegrees = 0;     // where the shaft should end up (moves on one step per turn)
float targetAngleDegrees = 0;   // smooth target that travels towards the goal
float targetSpeed = 0;          // how fast the smooth target is moving, degrees per second
float integralAccumulator = 0;
long previousCounts = 0;
long restStartCounts = 0;            // where the shaft was when it last came to rest
unsigned long restStartMillis = 0;   // when the shaft last came to rest
unsigned long laserOnMillis = 0;     // when the laser was switched on
unsigned long previousLoopMillis = 0;
unsigned long lastPrintMillis = 0;

// ---------------- helpers ----------------
long degreesToCounts(float degrees) {
  return lround(degrees * COUNTS_PER_DEGREE);
}

float countsToDegrees(long counts) {
  return counts / COUNTS_PER_DEGREE;
}

// motorCommand: -255..255. Sign = direction, magnitude = effort.
void drive(int motorCommand) {
  if (motorCommand == 0) {
    analogWrite(motorPWM, 0);
    return;
  }
  digitalWrite(motorDIR, motorCommand > 0 ? HIGH : LOW);
  int magnitude = abs(motorCommand);
  if (magnitude < MIN_PWM) magnitude = MIN_PWM;
  if (magnitude > 255) magnitude = 255;
  analogWrite(motorPWM, magnitude);
}

void setupStepTest() {
  // ~31 kHz PWM on pins 9/10 -- above hearing, and well within the DRV8874's
  // range. Comment out if you'd rather keep the stock 490 Hz.
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  // Wherever the shaft is at start-up counts as 0 degrees.
  motorEncoder.write(0);

  // The first turn starts straight away.
  goalAngleDegrees = STEP_ANGLE_DEGREES;

  previousLoopMillis = millis();
}

void runStepTest() {
  // ===== 1. WAIT FOR THE NEXT UPDATE =====
  // runStepTest() is called thousands of times a second, but we only want to
  // update the motor every CONTROL_LOOP_INTERVAL_MS. Until then, leave early.
  unsigned long now = millis();
  if (now - previousLoopMillis < CONTROL_LOOP_INTERVAL_MS) return;
  float deltaTimeSeconds = (now - previousLoopMillis) / 1000.0;  // time since last update
  previousLoopMillis = now;

  // ===== 2. WHERE IS THE SHAFT? =====
  long currentCounts = motorEncoder.read();

  // ===== 3. ROTATE -> STOP -> FIRE, THEN REPEAT =====
  switch (sequenceState) {
    case ROTATING:
      // The smooth target is on its way. Once it has landed on the goal
      // (step 4 puts it there exactly), start waiting for the shaft to stop.
      if (targetSpeed == 0 && targetAngleDegrees == goalAngleDegrees) {
        restStartMillis = now;
        restStartCounts = currentCounts;
        sequenceState = SETTLING;
      }
      break;

    case SETTLING:
      // The shaft has stopped once it has stayed inside the deadband for
      // SETTLE_TIME_MS, moving no more than 1 count (allows for encoder jitter).
      // Leaving the deadband or moving further starts the wait again, so the
      // laser never fires while a heavy flywheel is still carrying the shaft.
      if (labs(degreesToCounts(goalAngleDegrees) - currentCounts) > DEADBAND_COUNTS
          || labs(currentCounts - restStartCounts) > 1) {
        restStartMillis = now;
        restStartCounts = currentCounts;
      } else if (now - restStartMillis >= SETTLE_TIME_MS) {
        digitalWrite(laserPin, HIGH);
        laserOnMillis = now;
        sequenceState = FIRING;
      }
      break;

    case FIRING:
      // The motor keeps holding the angle while the laser is on. When time is
      // up, switch the laser off and move the goal on to start the next turn.
      if (now - laserOnMillis >= LASER_ON_TIME_MS) {
        digitalWrite(laserPin, LOW);
        goalAngleDegrees += STEP_ANGLE_DEGREES;
        sequenceState = ROTATING;
      }
      break;
  }

  // ===== 4. MOVE THE SMOOTH TARGET TOWARDS THE GOAL =====
  // The target speeds up, cruises at MAX_SPEED, then slows down in time to stop
  // exactly on the goal. Its speed never changes faster than MAX_ACCELERATION.
  float distanceLeft = goalAngleDegrees - targetAngleDegrees;
  float speedChangePerUpdate = MAX_ACCELERATION * deltaTimeSeconds;

  // Fastest speed that still lets the target stop by the goal.
  // (From physics: stopping speed = sqrt(2 * acceleration * distance).
  //  The speedChangePerUpdate parts allow for the gap between updates.)
  float stoppingSpeed = sqrt(speedChangePerUpdate * speedChangePerUpdate
                             + 2 * MAX_ACCELERATION * fabs(distanceLeft))
                        - speedChangePerUpdate;

  // Wanted speed: as fast as allowed, in the direction of the goal.
  float wantedSpeed = min(MAX_SPEED, stoppingSpeed);
  if (distanceLeft < 0) wantedSpeed = -wantedSpeed;

  // Head towards the wanted speed, but only change speed by a limited amount per update.
  targetSpeed += constrain(wantedSpeed - targetSpeed, -speedChangePerUpdate, speedChangePerUpdate);

  // Move the target. If this move would reach (or pass) the goal, land exactly on it.
  float moveThisUpdate = targetSpeed * deltaTimeSeconds;
  if (fabs(moveThisUpdate) >= fabs(distanceLeft)) {
    targetAngleDegrees = goalAngleDegrees;
    targetSpeed = 0;
  } else {
    targetAngleDegrees += moveThisUpdate;
  }

  // ===== 5. HOW FAR IS THE SHAFT FROM THE TARGET? =====
  long targetCounts = degreesToCounts(targetAngleDegrees);
  long error = targetCounts - currentCounts;   // positive = shaft needs to move forward

  // ===== 6. PID: HOW HARD SHOULD THE MOTOR PUSH? =====
  int motorCommand = 0;

  if (targetSpeed == 0 && labs(error) <= DEADBAND_COUNTS) {
    // The target has stopped and we're close enough: leave the motor off, and
    // clear the I term so it doesn't carry over into the next step.
    // (While the target is still moving, the motor must keep following it,
    // even when the error is small.)
    integralAccumulator = 0;
  } else {
    // P: the further away we are, the harder we push.
    float pTerm = PROPORTIONAL_GAIN * error;

    // I: keep a running total of the error. If the shaft is stuck just short
    // of the target, this total grows until it's enough to push it the rest of the way.
    integralAccumulator += error * deltaTimeSeconds;
    integralAccumulator = constrain(integralAccumulator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float iTerm = INTEGRAL_GAIN * integralAccumulator;

    // D: compare the shaft's speed with the target's speed. If a heavy flywheel
    // is carrying the shaft along faster than the target, this pushes back to
    // slow it down. If the shaft is too slow, this helps push it along.
    float shaftSpeed = (currentCounts - previousCounts) / deltaTimeSeconds;   // counts per second
    float targetSpeedCounts = targetSpeed * COUNTS_PER_DEGREE;                // counts per second
    float dTerm = DERIVATIVE_GAIN * (targetSpeedCounts - shaftSpeed);

    float total = pTerm + iTerm + dTerm;
    motorCommand = (int)constrain(total, -(float)MAX_PWM, (float)MAX_PWM);
  }

  previousCounts = currentCounts;   // remembered for next update's speed calculation

  // ===== 7. DRIVE THE MOTOR =====
  drive(motorCommand);

  // ===== 8. PRINT (optional) =====
  // Open Tools > Serial Plotter to see three lines:
  //   goal (a staircase), target (smooth ramps), actual (should follow target).
  // The laser is on for the last second of each stair, after actual has levelled off.
  if (DEBUG && now - lastPrintMillis >= PRINT_INTERVAL_MS) {
    lastPrintMillis = now;
    Serial.print("goal:");
    Serial.print(goalAngleDegrees);
    Serial.print(",target:");
    Serial.print(targetAngleDegrees);
    Serial.print(",actual:");
    Serial.println(countsToDegrees(currentCounts));
  }
}
