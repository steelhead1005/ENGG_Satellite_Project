
## Wiring

### Arduino pin assignments

| Pin | Connected to | Direction | Notes |
|---|---|---|---|
| D0 / D1 | USB serial | — | Reserved for telemetry and logging |
| D2 | Motor encoder, Hall A | Input (`INPUT_PULLUP`) | Interrupt pin, triggers on rising edge |
| D3 | IR receiver, +30° (front right) | Input (`INPUT_PULLUP`) | Active LOW |
| D4 | Motor encoder, Hall B | Input (`INPUT_PULLUP`) | Read in the encoder interrupt to give direction |
| D5 | IR receiver, −30° (front left) | Input (`INPUT_PULLUP`) | Active LOW |
| D6 | Laser | Output | Active HIGH |
| D7 | IR receiver, −150° (back left) | Input (`INPUT_PULLUP`) | Active LOW |
| D8 | DRV8874 `PH/IN2` | Output | Direction (see driver mode below) |
| D9 | DRV8874 `EN/IN1` | Output (PWM) | Motor speed |
| D10 | IR receiver, +90° (right) | Input (`INPUT_PULLUP`) | Active LOW. Sensor is currently detached from the body |
| D11 | IR receiver, −90° (left) | Input (`INPUT_PULLUP`) | Active LOW |
| D12 | IR receiver, +150° (back right) | Input (`INPUT_PULLUP`) | Active LOW |
| D13 | — | — | Spare |
| A0–A5 | — | — | Spare |

IR receiver angles are measured from the laser boresight (0°); positive is clockwise.

### Motor driver (Pololu DRV8874 carrier)

| Driver pin | Connected to | Effect |
|---|---|---|
| `EN/IN1` | Arduino D9 | PWM input |
| `PH/IN2` | Arduino D8 | Direction input |
| `PMODE` | 5 V | Selects PWM (IN1/IN2) mode |
| `SLEEP` | GND (to be verified) | Low puts the driver to sleep and disables the outputs |
| `nFAULT` | Not connected | — |
| `IPROPI` | Not connected | — |
| `OUT1` / `OUT2` | FIT0186 gearmotor (12 V, 251 rpm, encoder) | Reaction wheel drive |

**Known issues:**
- **`PMODE` is tied to 5 V, which selects PWM (IN1/IN2) mode, but the firmware drives the motor as if it were in PH/EN mode.** In PWM mode, DIR HIGH with low PWM duty runs the motor in reverse rather than stopping it. Tie `PMODE` to GND to match the firmware.
- **`SLEEP` must be HIGH for the driver to run.** Measure its voltage with the board powered. If it reads 0 V, connect it to 5 V or to a spare pin driven HIGH in `setup()`.

## Targeting Process

MQ Sentinel operates as a closed-loop pointing system. 
The firmware runs a fixed-rate control loop (target: 100 Hz) driving a five-state machine. No
stage blocks; every state is evaluated once per tick.

### Overview

| State | Purpose | Exit condition |
|---|---|---|
| `SEARCHING` | Locate an active beacon | Beacon detected on one or more receivers |
| `ROTATING` | Rotate the body toward the estimated bearing | Pointing error within capture window |
| `SETTLING` | Damp residual motion and overshoot | Error and angular rate both below threshold |
| `DWELLING` | Laser on, accumulate the 2 s dwell | Dwell timer reaches 2000 ms |
| `LOST` | Beacon signal dropped mid-manoeuvre | Beacon reacquired, or search timeout elapses |

### 1. Detection

Infrared receivers are distributed around the sensor ring at a height of
40 mm. Each responds to light modulated at 38 kHz, so ambient and unmodulated
sources are rejected in hardware. The firmware polls all channels every tick
and records which receivers are asserting.

Exactly one beacon is active at any time during the demonstration; the active
transmitter is selected at random. Should more than one channel indicate a
valid signal, arbitration selects the strongest response and applies a
lockout to prevent oscillation between candidates.

### 2. Bearing estimation

A discrete ring of receivers cannot resolve the required pointing accuracy
directly. An 8 cm bullseye at a range of 1 m subtends approximately ±2.3°,
which is finer than the angular spacing between adjacent sensors.

Bearing is therefore estimated by interpolation. Where a beacon is visible to
two neighbouring receivers, the relative signal strength between them is used
to estimate an intermediate angle. This produces a coarse bearing sufficient
to initiate a rotation, refined during the settling phase.

Estimated bearing is expressed as a signed pointing error relative to the
laser boresight: the angular distance the body must rotate, and in which
direction.

### 3. Rotation

MQ Sentinel has no external propulsion. Rotation is produced by a reaction
wheel driven by the supplied DC gearmotor: accelerating the wheel imparts an
equal and opposite torque to the satellite body.

The controller commands wheel torque, not body angle. Firmware output is a
signed value; its sign sets motor direction and its magnitude sets PWM duty.
Because angular momentum persists after the command is removed, overshoot is
the default behaviour of the system rather than a fault condition, and is
managed by the controller rather than avoided.

Body attitude is not directly measured. It is inferred from motor encoder
counts (wheel speed and accumulated rotation) combined with the bearing
estimate from the sensor ring.

### 4. Settling

Once pointing error falls within the capture window, the controller
transitions to damping residual motion. The `DWELLING` state is not entered
until both pointing error and angular rate are below their respective
thresholds, ensuring the laser is stationary on target rather than passing
through it.

### 5. Dwell and scoring

The laser is enabled on entry to `DWELLING` and the dwell timer begins
accumulating.

The timer is a gate, not a delay. Pointing error is re-evaluated every tick;
if it exceeds the dwell threshold at any point, the timer resets. A scoring
event is registered only when 2000 ms of continuous on-target dwell is
accumulated. Scoring zones are concentric: 10 points for the 8 cm bullseye,
5 for the 16 cm ring, and 1 for the 24 cm outer ring, with the lesser value
applying if the laser crosses a zone boundary during the dwell period.

On completion, the system returns to `SEARCHING` to acquire the next beacon.