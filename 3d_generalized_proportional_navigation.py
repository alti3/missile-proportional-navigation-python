import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Simulation parameters
N = 4              # Navigation constant
dt = 0.01          # Time step (seconds)
max_time = 100     # Maximum simulation time (seconds)
intercept_dist = 1 # Intercept distance threshold (meters)

# Initial conditions
r_t = np.array([0.0, 0.0, 0.0])       # Target initial position (m)
v_t = np.array([100.0, 0.0, 0.0])     # Target velocity (m/s, constant)
r_m = np.array([0.0, 10000.0, 0.0])   # Missile initial position (m)
v_m = np.array([200.0, 0.0, 0.0])     # Missile initial velocity (m/s)

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
        # Compute LOS rate (sigma_dot)
        r_cross_v = np.cross(r, v)
        sigma_dot = r_cross_v / r_norm**2
        
        # Relative velocity magnitude and direction
        V_r = np.linalg.norm(v)
        if V_r > 0:
            e_v = v / V_r
            # Perpendicular to relative velocity
            n = np.cross(e_v, sigma_dot)
            # Acceleration command
            a_m = N * V_r * n
        else:
            a_m = np.array([0.0, 0.0, 0.0])
        
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

# Convert to numpy arrays for plotting
positions_m = np.array(positions_m)
positions_t = np.array(positions_t)

# 3D Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions_t[:, 0], positions_t[:, 1], positions_t[:, 2], label='Target')
ax.plot(positions_m[:, 0], positions_m[:, 1], positions_m[:, 2], label='Missile')
ax.legend()
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Generalized Proportional Navigation')
plt.show() 