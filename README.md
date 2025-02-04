# ROS2_iAHRS_Dynamixel


## Dynamixel IMU Motor Control

This package enables motor control based on IMU sensor data using the Dynamixel SDK.
It subscribes to IMU data from the /imu/data topic and adjusts the motor speed accordingly.

---

### 1. Reference Packages
This package is built upon the following references:

**IMU Driver** : iahrs_driver_ros2 (<https://github.com/wookbin/iahrs_driver_ros2>)
Please refer to the original repository for details on setting up and running the IMU driver.

**Dynamixel SDK** : ROBOTIS DynamixelSDK (<https://github.com/ROBOTIS-GIT/DynamixelSDK>)
This package uses the Dynamixel SDK to communicate with EX-106 motors via Protocol 1.0.

Please **git clone and build** the SDK from the provided link before proceeding.
---

### 2. System Requirements
OS: Ubuntu 20.04
ROS2: Foxy
Motors: 2 × Dynamixel EX-106
Protocol: Dynamixel Protocol 1.0

#### Step 0: Build the Dynamixel SDK
Dynamixel SDK must be built before using this package.

📌 Follow the instructions from the official repository: <https://github.com/ROBOTIS-GIT/DynamixelSDK>


#### Step 1: Build the IMU Driver
The iahrs_driver_ros2 package must be built first, ensuring that the interface files are compiled beforehand.

cd ~/ros2_ws/src

git clone https://github.com/wookbin/iahrs_driver_ros2

cd ~/ros2_ws

colcon build --packages-select interfaces  # Build interface files first 

colcon build --packages-select iahrs_driver

source install/setup.bash

#### Step 2: Build the Motor Control Package

cd ~/ros2_ws/src

git clone https://github.com/whatny/ROS2_iAHRS_Dynamixel

cd ~/ros2_ws

colcon build --packages-select ROS2_iAHRS_Dynamixel

source install/setup.bash

---

#### Running the System
Once both packages are built, follow these steps to launch them.

#### Step 1: Launch the IMU Driver

ros2 launch iahrs_driver iahrs_driver.py

#### Step 2: Verify IMU Data

ros2 topic echo /imu/data

#### Step 3: Run the Motor Control Node

ros2 run ROS2_iAHRS_Dynamixel dynamixel_imu


Important Notes
IMU Sensor Data Anomalies
If the IMU sensor provides abnormally high values, the calculated motor speed might exceed the valid range for Dynamixel motors.
To prevent unexpected behavior, refer to the Dynamixel e-Manual to check the valid speed range and adjust accordingly.

### If you have any problems, please feel free to contact me in any way



https://github.com/user-attachments/assets/ebe525db-4845-44c2-90af-84de3d895cb3



https://github.com/user-attachments/assets/42f45f03-20c6-442e-8659-b2fe13dd3a44


Since the speed increment per IMU reading is relatively small, the change may not be visually noticeable.
If you need a more visible effect, you can modify the speed increment factor in the source code.
