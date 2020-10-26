# fy03_localization
**A ROS package containing an outdoor localization system that fuses UWB and IMU sensor data.**

## Overview

## Prerequisites
### Software

### Hardware

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/fy03_localization.git # Localization package
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_bno055.git # IMU package
cd .. 
sudo apt-get install -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make
source devel/setup.bash
rospack profile
```

## Setup

## Usage

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang

