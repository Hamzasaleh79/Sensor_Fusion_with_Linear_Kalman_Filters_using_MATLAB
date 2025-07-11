# Sensor Fusion with Linear Kalman Filters using MATLAB

## Overview
This repository contains the MATLAB code for Assignment Mobile Robots . The assignment focuses on processing mobile sensor data from an accelerometer and gyroscope to estimate roll and pitch angles using a Linear Kalman Filter (LKF) for sensor fusion.

## Assignment Description
The goal of the assignment is to:
1. **Gather and Plot IMU Data**: Collect accelerometer and gyroscope data from a mobile device and plot the x, y, z components for each sensor.
2. **Accelerometer-Based Angle Estimation**: Compute roll and pitch angles using accelerometer data.
3. **Gyroscope-Based Angle Rate Estimation**: Integrate gyroscope data to estimate roll and pitch angle rates.
4. **Sensor Fusion with LKF**: Implement a Linear Kalman Filter to fuse accelerometer and gyroscope data for improved roll and pitch angle estimation.



## Files
- **process_mobile_sensor_data.m**: MATLAB script implementing the tasks outlined in the assignment.
- **sensorlog_last.mat**: Sample sensor data file containing accelerometer and gyroscope measurements (not included in the repository due to file path specificity).

## Code Structure
The MATLAB script (`process_mobile_sensor_data.m`) is divided into four tasks:

### Task 1: Gathering and Plotting IMU Data
- Loads sensor data from `sensorlog_last.mat`.
- Extracts acceleration and angular velocity data for x, y, z axes.
- Plots the components over a 20-second time vector.

### Task 2: Accelerometer-Based Angle Estimation
- Computes roll and pitch angles using accelerometer data with the `atan2` function to avoid division by zero.
- Equations used:
  - Roll: \(\hat{\phi} = \tan^{-1} \frac{a_y}{\sqrt{a_x^2 + a_z^2}}\)
  - Pitch: \(\hat{\theta} = \tan^{-1} \frac{-a_x}{\sqrt{a_y^2 + a_z^2}}\)

### Task 3: Gyroscope-Based Angle Rate Estimation
- Extracts angular velocities (p, q, r) from gyroscope data.
- Computes roll and pitch rates using the Euler rate equations.
- Integrates the rates over time to estimate roll and pitch angles.
- Equation for roll rate: \(\dot{\hat{\phi}} = p + \sin(\phi) \tan(\theta) q + \cos(\phi) \tan(\theta) r\)
- Equation for pitch rate: \(\dot{\hat{\theta}} = \cos(\phi) q - \sin(\phi) r\)

### Task 4: Sensor Fusion with Linear Kalman Filter
- Implements a Linear Kalman Filter to fuse accelerometer-based angles with gyroscope-based angle rates.
- Uses constant matrices (A, u, C, R, Q) for simplicity.
- Performs prediction and correction steps for both roll and pitch angles.
- Plots Gaussian probability density curves for selected time steps (k = 50, 100, 150, 200).



## License
This project is for educational purposes only and is not licensed for commercial use.