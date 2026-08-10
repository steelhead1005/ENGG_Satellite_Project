
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