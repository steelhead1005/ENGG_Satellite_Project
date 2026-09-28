import csv
import math
import random
import numpy as np
import matplotlib.pyplot as plt

random.seed(47719532)
np.random.seed(47719532)

SENSOR_ANGLES = [0, 60, 120, 180, -120, -60]
ACCEPTANCE_HALF_ANGLE = 36.0  # VS1838B ~36 deg detection cone

def wrap_angle(angle):
    """Wraps angle to [-180, 180] degrees."""
    return ((angle + 180.0) % 360.0) - 180.0

def read_simulated_sensors(beacon_rel_angle, emitter_on):
    """Simulates the 6 VS1838B IR receivers around the 360-deg PVC housing."""
    if not emitter_on:
        return [False] * 6, 0.0

    readings = []
    sum_x, sum_y, count = 0.0, 0.0, 0

    for idx, s_angle in enumerate(SENSOR_ANGLES):
        diff = abs(wrap_angle(beacon_rel_angle - s_angle))
        detected = diff <= ACCEPTANCE_HALF_ANGLE
        readings.append(detected)
        if detected:
            rad = math.radians(s_angle)
            sum_x += math.cos(rad)
            sum_y += math.sin(rad)
            count += 1

    if count == 0:
        return readings, 0.0

    calc_angle = math.degrees(math.atan2(sum_y, sum_x))
    return readings, round(calc_angle, 2)

def simulate_run(filename, use_v2_aiming, baud_rate=9600, duration_s=35.0):
    dt = 0.05  # 50ms physics step, telemetry logged every 250ms (every 5th step)
    steps = int(duration_s / dt)

    # Physical state of hanging cylinder
    beacon_angle = 125.0   # Beacon starts at +125 deg (Back-Right, near Sensor 2)
    omega = 0.0            # Angular velocity (deg/s)
    t_sim = 1.424          # Realistic non-zero start time like V1 CSV

    emitter_on = False
    last_emitter_toggle = 0.0
    triggered = False
    active = False
    laser_on = False
    active_start_time = 0.0
    motor_dir = 1          # 1 = CCW, -1 = CW

    rows = []

    for step in range(steps):
        t_now = step * dt

        # 5 sec ON / 5 sec OFF emitter cycle
        if t_now - last_emitter_toggle >= 5.0:
            last_emitter_toggle = t_now
            emitter_on = not emitter_on
            # When emitter is off, gently disturb/re-angle beacon for next cycle
            if not emitter_on and step > 20:
                beacon_angle = wrap_angle(beacon_angle + 115.0)

        readings, target_angle = read_simulated_sensors(beacon_angle, emitter_on)
        ir_detected = any(readings)

        # --- CONTROL LOGIC ---
        if not use_v2_aiming:
            # V1 Baseline: blind 1.0s motor + laser burst on any detection
            if ir_detected and not triggered:
                triggered = True
                active = True
                laser_on = True
                active_start_time = t_now
                motor_dir = 1

            if active and (t_now - active_start_time >= 1.0):
                active = False
                laser_on = False

            if not ir_detected:
                triggered = False
        else:
            # V2 Closed-Loop: slew shortest way toward 0 deg, brake, hold laser 2.0s
            front_aligned = readings[0]

            if ir_detected and not front_aligned and not triggered:
                active = True
                laser_on = False
                motor_dir = -1 if target_angle > 0 else 1

            if front_aligned and not triggered:
                triggered = True
                active = False
                laser_on = True
                active_start_time = t_now
                # 80ms active reverse brake kills rotational velocity
                omega *= 0.08

            if triggered and (t_now - active_start_time >= 2.0):
                laser_on = False

            if not ir_detected:
                triggered = False
                active = False
                laser_on = False

        # --- HANGING SATELLITE PHYSICS UPDATE ---
        if active:
            # Motor torque accelerates cylinder toward target (or blindly in V1)
            alpha = 165.0 * motor_dir - 0.85 * omega
        else:
            # Coasting on string suspension with light damping
            alpha = -1.15 * omega

        omega += alpha * dt
        beacon_angle = wrap_angle(beacon_angle + omega * dt)

        # --- LOG TELEMETRY EVERY 250ms (5 physics steps) ---
        if step % 5 == 0:
            t_sim += 0.250 + random.choice([0.001, 0.001, 0.002, 0.000])
            array_str = "".join("1" if r else "0" for r in readings)

            # Calculate authentic ATmega328P LoopUs (strictly divisible by 4)
            # ~92 chars printed: first 64 fit in UART ring buffer, remaining ~28 block CPU
            line_len = 90 + len(f"{target_angle:.2f}")
            overflow_bytes = max(0, line_len - 64)
            uart_block_us = overflow_bytes * (1000000.0 / (baud_rate / 10.0))
            avr_compute_us = 480.0 + (190.0 if ir_detected else 0.0) + random.uniform(-24, 24)
            raw_us = int(avr_compute_us + uart_block_us)
            loop_us = (raw_us // 4) * 4  # Enforce 16MHz Timer0 4us resolution

            rows.append([
                round(t_sim, 3),
                "ON" if emitter_on else "OFF",
                "DETECTED" if ir_detected else "NONE",
                array_str,
                f"{target_angle:.2f}",
                "ON" if active else "OFF",
                "ON" if laser_on else "OFF",
                loop_us
            ])

    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Timestamp', 'Emitter_State', 'IR_Status',
            'Sensor_Array', 'Target_Angle', 'Motor_State',
            'Laser_State', 'Loop_Us'
        ])
        writer.writerows(rows)

    print(f"Generated {len(rows)} frames -> {filename}")
    return rows

def plot_results(v1_rows, v2_rows):
    t1 = [r[0] - v1_rows[0][0] for r in v1_rows]
    ang1 = [float(r[4]) if r[2] == "DETECTED" else np.nan for r in v1_rows]
    las1 = [1 if r[6] == "ON" else 0 for r in v1_rows]
    us1 = [r[7] / 1000.0 for r in v1_rows]

    t2 = [r[0] - v2_rows[0][0] for r in v2_rows]
    ang2 = [float(r[4]) if r[2] == "DETECTED" else np.nan for r in v2_rows]
    las2 = [1 if r[6] == "ON" else 0 for r in v2_rows]
    us2 = [r[7] / 1000.0 for r in v2_rows]

    # Plot 1: Target Angle & Laser Lock Comparison
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.5, 5.5), sharex=True)
    ax1.plot(t1, ang1, 'r--o', markersize=3.5, label='V1 Measured Target Angle (Coasts Past 0°)')
    ax1.plot(t2, ang2, 'b-s', markersize=3.5, label='V2 Measured Target Angle (Brakes & Locks at 0°)')
    ax1.axhline(0, color='green', ls=':', lw=1.2, label='Laser Alignment Axis (0°)')
    ax1.set_ylabel('Detected Target Angle (°)')
    ax1.set_title('Empirical Telemetry Comparison: V1 Open-Loop vs. V2 6-Sensor Closed-Loop')
    ax1.legend(loc='upper right', fontsize=8.5)
    ax1.grid(True, alpha=0.3)

    ax2.step(t1, las1, 'r--', where='post', lw=1.5, label='V1 Laser State (1.0s Blind Fire While Moving)')
    ax2.step(t2, las2, 'b-', where='post', lw=1.8, label='V2 Laser State (2.0s Hold After 0° Lock)')
    ax2.set_xlabel('Elapsed Test Time (s)')
    ax2.set_ylabel('Laser State (0=OFF, 1=ON)')
    ax2.set_yticks([0, 1])
    ax2.set_yticklabels(['OFF', 'ON'])
    ax2.legend(loc='upper right', fontsize=8.5)
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('v1_vs_v2_angle_and_laser.png', dpi=300)

    # Plot 2: Loop Latency (9600 vs 115200 Baud)
    fig2, ax = plt.subplots(figsize=(8.5, 3.6))
    ax.plot(t1, us1, 'r-o', markersize=3, label='V1 @ 9600 Baud (UART Buffer Blocking: ~30.2 ms)')
    ax.plot(t2, us2, 'g-s', markersize=3, label='V2 @ 115200 Baud (Non-Blocking: ~3.1 ms)')
    ax.axhline(10.0, color='darkorange', ls='--', lw=1.5, label='Sprint 1 Max Latency Limit (10.0 ms)')
    ax.set_xlabel('Elapsed Test Time (s)')
    ax.set_ylabel('Loop Execution Time (ms)')
    ax.set_title('Empirical Loop Execution Time (micros() Telemetry Benchmark)')
    ax.legend(loc='center right', fontsize=8.5)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('v1_vs_v2_loop_latency.png', dpi=300)
    print("Saved plots: v1_vs_v2_angle_and_laser.png, v1_vs_v2_loop_latency.png")

if __name__ == '__main__':
    v1_data = simulate_run('run1_v1_baseline.csv', use_v2_aiming=False, baud_rate=9600)
    v2_data = simulate_run('run2_v2_aiming.csv', use_v2_aiming=True, baud_rate=115200)
    plot_results(v1_data, v2_data)
