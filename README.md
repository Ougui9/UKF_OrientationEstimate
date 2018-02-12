# Orientation Estimation with Unscented Kalman Filters (UKF)

Use data separatly from the gyroscope and accelerometer on ArduIMU+ V2, estimate orientation based on UKF. The data from Vicon serves as the ground truth. To visualize the performance, this project also provides with a function to generate panaramic images.

## Getting Started
1. Before starting, two folder containing imu data(gyro, accelerometer and timestamp) and vicon data respectively should be under the root.
2. Just run try.py to start.
3. This project turns down the panarama by default. To switch it on, just change the variable plotPanarama as True at the beginning of try.py.
```
plotPanarama = True

```

### Prerequisites

Python 3

## test Results

### Orientation comparison
####result of dataset 1
![alt text](https://github.com/Ougui9/UKF_OrientaionEstimate/blob/master/res/weightedUKF1.png)
####result of dataset 2
![alt text](https://github.com/Ougui9/UKF_OrientaionEstimate/blob/master/res/weightedUKF2.png)
####result of dataset 3
![alt text](https://github.com/Ougui9/UKF_OrientaionEstimate/blob/master/res/weightedUKF8.png)
####result of dataset 4
![alt text](https://github.com/Ougui9/UKF_OrientaionEstimate/blob/master/res/weightedUKF4.png)
### Panarama
![alt text](https://github.com/Ougui9/UKF_OrientaionEstimate/blob/master/res/panDs1.jpg)
