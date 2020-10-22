#!/usr/bin/env python

'''
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    ROS node that subscribes to the IMU topic published to by the BNO055 ROS Node, and estimates position purely based on linear acceleration data.
'''

import rospy
from sensor_msgs.msg import Imu

def imuCallback(imu_msg):
  rospy.loginfo("X: %f", imu_msg.linear_acceleration.x)
  rospy.loginfo("Y: %f", imu_msg.linear_acceleration.y)
  rospy.loginfo("Z: %f", imu_msg.linear_acceleration.z)

if __name__ == '__main__':

  rospy.init_node("imu_position_estimation_node") 
    
  rospy.Subscriber("bno055_node/imu", Imu, imuCallback)

  rospy.spin()




