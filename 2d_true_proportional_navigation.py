import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
N = 4              # Navigation constant
dt = 0.01          # Time step (seconds)
max_time = 100     # Maximum simulation time (seconds)
intercept_dist = 1 # Intercept distance threshold (meters)

# Initial conditions
r_t = np.array([0.0, 0.0])       # Target initial position (m)
v_t = np.array([100.0, 0.0])     # Target velocity (m/s, constant)
r_m = np.array([0.0, 10000.0])   # Missile initial position (m)
v_m = np.array([200.0, 0.0])     # Missile initial velocity (m/s)

# Lists to store trajectories
positions_m = [r_m.copy()]
positions_t = [r_t.copy()]

# Simulation loop
t = 0
while t < max_time:
    # Compute relative position and velocity
    r = r_t - r_m
    v = v_t - v_m
    r_norm = np.linalg.norm(r)
    
    # Check for intercept
    if r_norm < intercept_dist:
        print(f"Intercept at time {t:.2f} s")
        break
    
    if r_norm > 0:
        # Compute LOS rate (sigma_dot) and closing velocity (V_c)
        r_dot_v = r[0] * v[0] + r[1] * v[1]
        r_cross_v = r[0] * v[1] - r[1] * v[0]
        sigma_dot = r_cross_v / r_norm**2
        V_c = -r_dot_v / r_norm
        
        # LOS unit vector and perpendicular direction
        e_LOS = r / r_norm
        n = np.array([-e_LOS[1], e_LOS[0]])  # Rotate 90° counterclockwise
        
        # Acceleration command
        a_m = N * V_c * sigma_dot * n
        
        # Update missile state
        v_m += a_m * dt
        r_m += v_m * dt
        
        # Update target state (constant velocity)
        r_t += v_t * dt
        
        # Store positions
        positions_m.append(r_m.copy())
        positions_t.append(r_t.copy())
        
        t += dt
    else:
        break

# Plot trajectories
positions_m = np.array(positions_m)
positions_t = np.array(positions_t)
plt.plot(positions_t[:, 0], positions_t[:, 1], label='Target')
plt.plot(positions_m[:, 0], positions_m[:, 1], label='Missile')
plt.legend()
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('True Proportional Navigation (TPN)')
plt.grid(True)
plt.axis('equal')
plt.show()