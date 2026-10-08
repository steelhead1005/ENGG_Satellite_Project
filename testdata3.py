import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)
dt = 0.01
time_s = np.arange(0, 5, dt)
n_samples = len(time_s)

v1_angle = np.zeros(n_samples)
v1_omega = np.zeros(n_samples)
v2_angle = np.zeros(n_samples)
v2_omega = np.zeros(n_samples)
v2_laser = np.zeros(n_samples)

start_angle = -120.0
v1_angle[0] = start_angle
v2_angle[0] = start_angle

has_braked = False
brake_timer = 0.0

for i in range(1, n_samples):
    t = time_s[i]
    
    # V1 Coasting
    torque1 = 180.0 if 0.5 <= t <= 1.5 else 0.0
    alpha1 = torque1 - (1.2 * v1_omega[i-1])
    v1_omega[i] = v1_omega[i-1] + alpha1 * dt
    v1_angle[i] = v1_angle[i-1] + v1_omega[i] * dt
    
    # V2 Active Brake
    torque2 = 0.0
    if t >= 0.5:
        if not has_braked and v2_angle[i-1] < -8.0:
            torque2 = 180.0
        elif not has_braked and v2_angle[i-1] >= -8.0:
            brake_timer = 0.08
            has_braked = True
            
        if brake_timer > 0:
            # We need a torque that exactly cancels the momentum, or close to it
            torque2 = -1000.0 # Make it stronger
            brake_timer -= dt
            
    # Apply static friction simulation
    alpha2 = torque2 - (1.2 * v2_omega[i-1])
    
    v2_omega[i] = v2_omega[i-1] + alpha2 * dt
    
    # Static friction threshold: if motor is off and velocity is low, it stops completely
    if torque2 == 0 and has_braked and abs(v2_omega[i]) < 5.0:
        v2_omega[i] = 0
        
    v2_angle[i] = v2_angle[i-1] + v2_omega[i] * dt
    
    if has_braked and brake_timer <= 0 and abs(v2_omega[i]) < 0.1:
        v2_laser[i] = 1.0

# --- UPDATED PLOTTING SECTION ---
plt.figure(figsize=(10, 4.5), dpi=300)

plt.plot(time_s, v1_angle, 'r--', linewidth=2, label='V1 Baseline (1000ms Motor Pulse + Coasting)')
plt.plot(time_s, v2_angle, 'b-', linewidth=2.5, label='V2 Optimized (FSM Slew + 80ms Active Brake)')

plt.axhline(0, color='darkgreen', linestyle='-', linewidth=1.5, label='Target Alignment Axis (0°)')

plt.fill_between(time_s, -130, 40, where=(v2_laser == 1.0), color='blue', alpha=0.1, label='V2 Laser Firing (Post-Brake Lock)')

# Pre-Build Simulation Watermark
plt.text(2.5, -45, 'PRE-BUILD SIMULATION', fontsize=35, color='gray', 
         alpha=0.15, ha='center', va='center', rotation=15, weight='bold')

plt.title('Kinematic Plant Simulation: V2 Active Braking & Safety Interlock')
plt.xlabel('Elapsed Test Time (s)')
plt.ylabel('Bearing to Target (°)')
plt.ylim(-130, 40)

plt.legend(loc='lower right')
plt.grid(True, alpha=0.3)
plt.tight_layout()

plt.savefig('test_graph_watermarked.png')
print(f"Max V2 Angle: {np.max(v2_angle):.2f}°")
print(f"Final V2 Angle: {v2_angle[-1]:.2f}°")
print("Plot saved to test_graph_watermarked.png")
