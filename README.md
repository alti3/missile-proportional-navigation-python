# Missile Proportional Navigation in Python

This repository provides Python implementations of various missile proportional navigation guidance laws. These simulations model missile intercepts in both 2D and 3D space using different proportional navigation techniques.

## Overview

Proportional navigation is a guidance method used by missiles to intercept moving targets. The basic principle is that the missile applies acceleration proportional to the rate of rotation of the line-of-sight (LOS) to the target.

The repository includes implementations of four different proportional navigation methods in both 2D and 3D:

1. [**Pure Proportional Navigation (PPN)**](pure_proportional_navigation.py) / [3D PPN](3d_pure_proportional_navigation.py)
2. [**True Proportional Navigation (TPN)**](true_proportional_navigation.py) / [3D TPN](3d_true_proportional_navigation.py)
3. [**Augmented Proportional Navigation**](augmented_proportional_navigation.py) / [3D Augmented PN](3d_augmented_proportional_navigation.py)
4. [**Generalized Proportional Navigation**](generalized_proportional_navigation.py) / [3D Generalized PN](3d_generalized_proportional_navigation.py)

## Navigation Methods

### Pure Proportional Navigation (PPN)
In Pure Proportional Navigation, the missile's acceleration is applied perpendicular to its velocity vector, with a magnitude proportional to the line-of-sight (LOS) rate and the missile's speed. This method adjusts the missile's heading to nullify the LOS rate.

### True Proportional Navigation (TPN)
In True Proportional Navigation, the acceleration is applied perpendicular to the LOS vector, with a magnitude proportional to the LOS rate and the closing velocity. This method is commonly used in missile guidance systems.

### Augmented Proportional Navigation
Augmented Proportional Navigation extends TPN by adding a term to compensate for target maneuvers. Here, the acceleration includes an additional component proportional to the target's acceleration, assumed to be known (e.g., through sensor data).

### Generalized Proportional Navigation
Generalized Proportional Navigation is a broader framework where the acceleration direction can differ from being strictly perpendicular to the LOS or missile velocity. For this simulation, the acceleration is applied perpendicular to the relative velocity vector (target velocity minus missile velocity), with a magnitude proportional to the LOS rate and the relative speed.

## Common Features

- **Libraries**: `numpy` for vector operations and `matplotlib.pyplot` for plotting. 3D versions use `mpl_toolkits.mplot3d`.
- **Parameters**: Navigation constant \( N = 4 \), time step \( dt = 0.01 \) s, max time 100 s, intercept distance 1 m.
- **Initial Conditions**: 
  - 2D: Target starts at (0, 0) moving right at 100 m/s; missile starts at (0, 10000) moving right at 200 m/s.
  - 3D: Target starts at (0, 0, 0) moving right at 100 m/s; missile starts at (0, 10000, 0) moving right at 200 m/s.
  - For Augmented PN, the target accelerates upward at 10 m/s².
- **Simulation**: Updates positions and velocities using Euler integration, stops when the missile is within 1 m of the target or time exceeds 100 s.
- **Output**: Plots the missile and target trajectories with labels and a grid. 3D versions include z-axis visualization.

## Specific Guidance Laws

1. **PPN**: 
   - Acceleration: `$\mathbf{a}_m = N \cdot V_m \cdot \dot{\sigma} \cdot \mathbf{m}$`
   - `$V_m = |\mathbf{v}_m|$`, `$\mathbf{m}$` is perpendicular to `$\mathbf{v}_m$`.
   - Adjusts missile heading based on its own velocity.

2. **TPN**: 
   - Acceleration: `$\mathbf{a}_m = N \cdot V_c \cdot \dot{\sigma} \cdot \mathbf{n}$`
   - `$V_c$` is the closing velocity, `$\mathbf{n}$` is perpendicular to the LOS.
   - Widely used due to its effectiveness against non-maneuvering targets.

3. **Augmented PN**: 
   - Acceleration: `$\mathbf{a}_m = N \cdot V_c \cdot \dot{\sigma} \cdot \mathbf{n} + \frac{N}{2} \cdot \mathbf{a}_t$`
   - Adds a term for target acceleration `$\mathbf{a}_t$`, improving performance against maneuvering targets.

4. **Generalized PN**: 
   - Acceleration: `$\mathbf{a}_m = N \cdot V_r \cdot \dot{\sigma} \cdot \mathbf{n}$`
   - `$V_r = |\mathbf{v}_t - \mathbf{v}_m|$`, `$\mathbf{n}$` is perpendicular to the relative velocity.
   - Interpreted as a variant where acceleration direction is more flexible, here chosen as perpendicular to relative velocity.

## Key Calculations

- **LOS Rate (`$\dot{\sigma}$`)**: 
  - 2D: `$\dot{\sigma} = \frac{r_x v_y - r_y v_x}{r_x^2 + r_y^2}$`
  - 3D: `$\dot{\sigma} = \frac{\mathbf{r} \times \mathbf{v}}{|\mathbf{r}|^2}$`
- **Closing Velocity (`$V_c$`)**: `$V_c = -\frac{\mathbf{r} \cdot \mathbf{v}}{|\mathbf{r}|}$`, negative rate of range change.
- **Direction Vectors**: 
  - 2D: Computed by rotating the reference vector 90° counterclockwise
  - 3D: Computed using cross products

## Notes

- The target has constant velocity in PPN, TPN, and Generalized PN, but constant acceleration in Augmented PN to demonstrate its capability.
- The missile's speed may change since acceleration isn't constrained to be purely lateral, though in practice, speed might be constant with only direction changing.
- Initial conditions ensure the missile can intercept within the simulation time, but adjustments might be needed for different scenarios.
- Generalized PN's implementation is one interpretation; the Wikipedia article notes it allows varied acceleration directions, so alternatives (e.g., biased PN) could also be valid.

These programs provide a practical demonstration of proportional navigation principles in both 2D and 3D space, suitable for educational purposes or as a starting point for more complex simulations.
