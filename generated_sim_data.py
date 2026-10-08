import csv
import math
import random
import matplotlib.pyplot as plt

# Enforce reproducibility
random.seed(42)

def simulate_run(filename, is_v2, baud_rate):
    dt = 0.05  # 50ms physics step
    steps = int(20.0 / dt) # 20 second test run
    
    # Physics state
    angle = 175.0  # Start near the Back sensor (180 deg)
    omega = 0.0    # Angular velocity
    
    # Logic state
    front_detected = False
    back_detected = False
    active = False
    laser_on = False
    triggered = False
    active_start = 0.0
    
    t_sim = 1.032
    rows = []

    for step in range(steps):
        t_now = step * dt
        
        # Sensor Cones (+/- 35 deg field of view)
        front_detected = abs(angle) <= 35.0
        back_detected = abs(angle - 180.0) <= 35.0 or abs(angle + 180.0) <= 35.0
        ir_detected = front_detected or back_detected

        if not is_v2:
            # V1: Blind 1-second burst on ANY detection
            if ir_detected and not triggered:
                triggered = True
                active = True
                laser_on = True
                active_start = t_now
                
            if active and (t_now - active_start >= 1.0):
                active = False
                laser_on = False
        else:
            # V2: 2-Sensor Slew and Brake
            if back_detected and not active and not triggered:
                active = True
                laser_on = False
                
            if front_detected and not triggered:
                triggered = True
                active = False
                laser_on = True
                active_start = t_now
                omega *= 0.1 # 80ms Reverse Brake kills momentum
                
            if triggered and (t_now - active_start >= 2.0):
                laser_on = False

        # Physics Engine: Torque and String Damping
        if active:
            alpha = 150.0 - (0.8 * omega) # Motor torque accelerating
        else:
            alpha = -(1.2 * omega) # Coasting friction / string torsion
            
        omega += alpha * dt
        angle = ((angle + omega * dt + 180.0) % 360.0) - 180.0

        # Log Telemetry (Every 250ms / 5 steps)
        if step % 5 == 0:
            t_sim += 0.250 + random.choice([0.000, 0.001, -0.001])
            
            # Hardware Timing Calculation
            str_len = 75
            overflow = max(0, str_len - 64)
            uart_delay = overflow * (1000000.0 / (baud_rate / 10.0))
            cpu_compute = 480.0 + random.uniform(-12, 12)
            loop_us = int((cpu_compute + uart_delay) // 4) * 4 # Round to 4us Timer0

            rows.append([
                round(t_sim, 3),
                "DETECTED" if ir_detected else "NONE",
                "1" if front_detected else "0",
                "1" if back_detected else "0",
                round(angle, 1),
                "ON" if active else "OFF",
                "ON" if laser_on else "OFF",
                loop_us
            ])

    # Write to CSV
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'IR_Status', 'Front_D3', 'Back_D5', 'Actual_Angle', 'Motor', 'Laser', 'Loop_Us'])
        writer.writerows(rows)
    return rows

# Generate Data
v1_data = simulate_run('sim_v1_baseline.csv', is_v2=False, baud_rate=9600)
v2_data = simulate_run('sim_v2_optimized.csv', is_v2=True, baud_rate=115200)

# --- PLOT 1: Loop Latency ---
plt.figure(figsize=(8, 3.5))
t1 = [r[0] for r in v1_data]
us1 = [r[7] / 1000.0 for r in v1_data]
t2 = [r[0] for r in v2_data]
us2 = [r[7] / 1000.0 for r in v2_data]

plt.plot(t1, us1, 'r-', label='V1 (9600 Baud UART Blocking)')
plt.plot(t2, us2, 'b-', label='V2 (115200 Baud EEPROM)')
plt.axhline(10.0, color='orange', linestyle='--', label='Sprint 1 Limit (<10ms)')
plt.title('Simulated Pre-Build Loop Execution Latency')
plt.xlabel('Time (s)')
plt.ylabel('Execution Time (ms)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('fig1_loop_latency.png', dpi=300)

# --- PLOT 2: Target Locking & Laser Hold ---
plt.figure(figsize=(8, 4))
ang1 = [float(r[4]) for r in v1_data]
las1 = [1 if r[6]=='ON' else 0 for r in v1_data]
ang2 = [float(r[4]) for r in v2_data]
las2 = [1 if r[6]=='ON' else 0 for r in v2_data]

plt.plot(t1, ang1, 'r--', alpha=0.6, label='V1 Rotational Trajectory (Open Loop)')
plt.fill_between(t1, 0, [a * 45 for a in las1], color='red', alpha=0.2, label='V1 Laser Firing (While Moving)')

plt.plot(t2, ang2, 'b-', linewidth=2, label='V2 Rotational Trajectory (Active Brake)')
plt.fill_between(t2, 0, [a * 45 for a in las2], color='blue', alpha=0.3, label='V2 Laser Firing (Stationary Lock)')

plt.axhline(0, color='green', linestyle=':', label='Target Axis (0°)')
plt.title('Simulated Rotational Step-Response & Actuation Locking')
plt.xlabel('Time (s)')
plt.ylabel('Bearing to Target (°)')
plt.legend(loc='lower right')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('fig2_actuation_lock.png', dpi=300)
