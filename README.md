# LSM6DSOX Kalman Filter Sensor Fusion
> Kalman Filter implementation on IMU 

This repo entails an implementation of the Discrete Kalman Filter on the 6-Axis IMU LSM6DSOX. 



### Results

The following graphs displays the values of the roll with the given parameters. The first image corresponds to a slow increase in the IMU's roll motion, while the second image corresponds to slight vibrations of the imu. 

```
q_ang = 0.001,
q_gyro = 0.1,
r = 0.1,
```

<img width="456" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/5cf48dee-8b5a-482d-b488-c18932f0f97c">
<img width="494" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/4abd0d1a-05e9-48a4-9bb3-df7ef5ce02d0">


```
q_ang = 0.001,
q_gyro = 0.01,
r = 0.03,
```

<img width="439" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/0934f054-b475-4a26-a8d6-6e48a05af947">
<img width="496" alt="image" src="https://github.com/KG-Drone-Project/LSM6DSOX-Kalman-Filter/assets/25258108/7093be91-651d-4a53-96f8-32ab725c882a">



### References

- https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp

- An Introduction to the Kalman Filter, <br />
  Greg Welch and Gary Bishop 1994, <br />
  https://api.semanticscholar.org/CorpusID:9209711



