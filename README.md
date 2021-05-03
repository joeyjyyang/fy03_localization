# fy03_localization
**A ROS package containing an outdoor localization system that fuses UWB and IMU sensor data through a Particle Filter.**

## Overview
This repository contains a UWB-IMU sensor fusion algorithm developed for the purposes of affordable outdoor localization. Specifically, absolute positioning data from Decawave's DWM1001 UWB sensors and relative motion data from Adafruit's BNO055 IMU sensors are fused using a Particle Filter; the output of the process is a continuous best-estimate of pose (position and orientation). The entire system is built within the ROS ecosystem, and is deployed on the Raspberry Pi 4 Model B (4 GB RAM). The drivers for the BNO055 IMU and DWM1001 UWB sensors are not contained within this repository; they need to be installed individually. Upon setting up the UWB perimeter and connecting the IMU and UWB (tag) to the Raspberry Pi, real-time localization can be performed using this package. 

## Prerequisites
### Software
- ROS Kinetic

### Hardware
- Raspberry Pi 4 Model B (4 GB RAM recommended)
- Adafruit's BNO055 Absolute Orientation Sensor
- Decawave's DWM1001 Development Kit (x5 recommended: 1 tag, 4 anchors)
- Android Smartphone

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/fy03_localization.git # Localization package
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_dwm1001.git # UWB package
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_bno055.git # IMU package
cd .. 
sudo apt-get install -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make
source devel/setup.bash
rospack profile
```

## Setup
## Raspberry Pi Setup
1. Install ROS Kinetic either [on default Raspberry Pi OS](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi), or [using Raspberry Pi image containing Ubuntu 16.04 and ROS Kinetic](https://downloads.ubiquityrobotics.com/pi.html)
2. [Enable software I2C on Raspberry Pi](https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md)

### UWB Setup
1. Install [Decawave Android application](https://www.decawave.com/source-code-for-the-android-application/) onto Android smartphone
2. Connect DWM1001 UWB (tag) to Raspberry Pi over USB (Micro B to USB)
3. Configure UWB system (3-4 anchors, 1 tag) using [Decawave's Quickstart](https://www.decawave.com/mdek1001/quickstart/)
4. Launch [DWM1001 ROS driver node](https://github.com/joeyjyyang/ros_dwm1001) to see UWB data in ROS

### IMU Setup
1. Wire IMU to Raspberry Pi over GPIO (I2C) pins
2. Launch [BNO055 ROS driver node](https://github.com/joeyjyyang/ros_bno055) to see IMU data in ROS

## Usage
### Example
```
roslaunch fy03_localization localization_online.launch
```

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang