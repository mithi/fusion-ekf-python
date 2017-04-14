This is an extended Kalman Filter implementation in C++ for fusing lidar and radar sensor measurements.
A Kalman filter can be used anywhere you have uncertain information about some dynamic system, 
and you want to make an educated guess about what the system is going to do next. 

**In this case, we have two 'noisy' sensors:**
- A lidar sensor that measures our position in cartesian-coordinates `(x, y)`
- A radar sensor that measures our position and velocity in polar coordinates `(rho, phi, drho)`

**We want to predict our position, and how fast we are going in what direction at any point in time:**
- In essence: the position and velocity of the system in cartesian coordinates: `(x, y, vx, vy)`
- Note that we are assuming a **constant velocity model (CV)** for this particular system

**This extended kalman filter does just that.** 

- Check the Jupyter notebooks to see sample usage and same visualization usage
- Please use the following format for your input file:

```
[L(for lidar)] [m_x] [m_y] [t] [r_x] [r_y] [r_vx] [r_vy]
[R(for radar)] [m_rho] [m_phi] [m_drho] [t] [r_px] [r_py] [r_vx] [r_vy]

Where:
(m_x, m_y) - measurements by the lidar
(m_rho, m_phi, m_dho) - measurements by the radar in polar coordinates
(t) - timestamp in unix/epoch time the measurements were taken
(r_x, r_y, r_vx, r_vy) - the real ground truth state of the system

Example:
R 8.60363 0.0290616 -2.99903  1477010443399637  8.6 0.25  -3.00029  0
L 8.45  0.25  1477010443349642  8.45  0.25  -3.00027  0 
```

# [Here's the repository for the C++ Version](https://github.com/mithi/Fusion-EKF-CPP)
