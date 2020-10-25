#!/usr/bin/env python

'''
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    ROS node that subscribes to the IMU topic published to by the BNO055 ROS Node, and estimates position purely based on linear acceleration data.
'''

import rospy
from sensor_msgs.msg import Imu

position = [0, 0, 0]
previous_time = 0
total_time = 0
accuracy_5cm = True
accuracy_10cm = True

def accuracyTest5Cm():
  global position, total_time, accuracy_5cm
  if (accuracy_5cm and (abs(position[0]) >= 0.05 or abs(position[1]) >= 0.05 or abs(position[2]) >= 0.05)):
    accuracy_5cm = False
    rospy.logwarn("Failed 5cm accuracy at in %f seconds. POOP", total_time)

def accuracyTest10Cm():
  global position, total_time, accuracy_10cm
  if (accuracy_10cm and (abs(position[0]) >= 0.10 or abs(position[1]) >= 0.10 or abs(position[2]) >= 0.10)):
    accuracy_10cm = False
    rospy.logwarn("Failed 10cm accuracy at in %f seconds.", total_time)

def imuCallback(imu_msg):
  global position, previous_time, total_time, accuracy_5cm, accuracy_10cm
  
  message_time = imu_msg.header.stamp.to_sec()
  # rospy.loginfo("Message sequence: %f, Message time: %f", imu_msg.header.seq, message_time)
  time_difference = message_time - previous_time
  total_time += time_difference 
  # rospy.loginfo("Time difference: %f", time_difference)
  # Set previous_time to (current) message_time for next callback.
  previous_time = message_time

  acceleration_x = imu_msg.linear_acceleration.x
  acceleration_y = imu_msg.linear_acceleration.y
  acceleration_z = imu_msg.linear_acceleration.z

  velocity_x = acceleration_x * time_difference
  velocity_y = acceleration_y * time_difference
  velocity_z = acceleration_z * time_difference

  displacement_x = velocity_x * time_difference
  displacement_y = velocity_y * time_difference
  displacement_z = velocity_z * time_difference

  position[0] += displacement_x
  position[1] += displacement_y
  position[2] += displacement_z

  # Determine time taken to fail 5cm accuracy due to IMU drift.
  accuracyTest5Cm()
  # Determine time taken to fail 10cm accuracy due to IMU drift.
  accuracyTest10Cm()

  rospy.loginfo("X: %f, Y: %f, Z: %f", position[0], position[1], position[2])

if __name__ == '__main__':
 
  rospy.init_node("imu_position_estimation_node") 
  rospy.Subscriber("bno055_node/imu", Imu, imuCallback)

  rospy.loginfo("Initial X: %f, Y: %f, Z: %f", position[0], position[1], position[2])
  # Set starting time stamp to current time.
  previous_time = rospy.get_time()
  rospy.loginfo("Initial time: %f:", previous_time)
 
  rospy.spin()




