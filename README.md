# LSM6DSOX Kalman Filter Sensor Fusion
[![Rust](https://img.shields.io/badge/language-Rust-orange.svg)](https://www.rust-lang.org/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/blob/main/LICENSE)
[![Sensor Fusion](https://img.shields.io/badge/sensor%20fusion-Kalman%20Filter-green.svg)](https://en.wikipedia.org/wiki/Kalman_filter)
[![Embedded Systems](https://img.shields.io/badge/platform-STM32-darkred.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)

> Kalman Filter implementation on IMU 

This repo entails an implementation of the Discrete Kalman Filter on the 6-Axis IMU LSM6DSOX. The goal is to output an angle that can avoid the instabilities or inaccuracies of the gyroscope and accelerometer by itself.  

### Introduction

This project aims to develop a system capable of generating precise pitch and roll angles. While these angles can be obtained from the accelerometer or gyroscope in the LSMD6SOX IMU used for this project, both sensors produce noisy data. Specifically:
- The gyroscope is susceptible to gyro drift, leading to a gradual increase in angle due to the integration of errors.
- Accelerometers, although accurate at rest, become noisy when exposed to vibrations or motion.

Considering the mentioned flaws, it becomes evident that the gyroscope provides accurate short-term data, whereas the accelerometer offers accurate long-term data. With this understanding, a Discrete Kalman Filter can be employed to fuse the two datasets, yielding a more accurate output.

The Kalman filter is an algorithm used for estimating the state of a system in the presence of noisy measurements. It works by iteratively predicting the next state based on a dynamic model and comparing it with actual measurements. The filter optimally combines predictions and measurements, dynamically adjusting their contributions based on their uncertainties. This allows the Kalman filter to provide a more accurate and stable estimate of the system's true state, making it widely used in applications like sensor fusion for robotics, navigation, and control systems.

### Structure


### Results

The following graphs displays the values of the roll with the given parameters. The first image corresponds to a slow increase in the IMU's roll motion, while the second image corresponds to slight vibrations of the imu. 

```
q_ang = 0.001,
q_gyro = 0.1,
r = 0.1,
```

<div style="display: flex; flex-direction: row;">
<img width="456" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/5cf48dee-8b5a-482d-b488-c18932f0f97c">
<img width="494" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/4abd0d1a-05e9-48a4-9bb3-df7ef5ce02d0">
<div/>

```
q_ang = 0.001,
q_gyro = 0.01,
r = 0.03,
```

<div style="display: flex; flex-direction: row;">
<img width="439" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/0934f054-b475-4a26-a8d6-6e48a05af947">
<img width="496" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/7093be91-651d-4a53-96f8-32ab725c882a">
<div/>


### References

- https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp

- An Introduction to the Kalman Filter, <br />
  Greg Welch and Gary Bishop 1994, <br />
  https://api.semanticscholar.org/CorpusID:9209711



