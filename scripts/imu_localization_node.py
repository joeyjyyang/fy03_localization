#!/usr/bin/env python

'''
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    ROS node that subscribes to the IMU topic published to by the BNO055 ROS Node, and estimates position purely based on linear acceleration data.
'''

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

position = {
  "x": 0,
  "y": 0,
  "z": 0
}
previous_time = 0
total_time = 0
accuracy_5cm = True
accuracy_10cm = True
start = False

odom_msg = Odometry()
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)


"""
def accuracyTest5Cm():
  global position, total_time, accuracy_5cm
  if (accuracy_5cm and (abs(position["x"]) >= 0.05 or abs(position["y"]) >= 0.05 or abs(position["z"]) >= 0.05)):
    accuracy_5cm = False
    rospy.logwarn("Failed 5cm accuracy at in %f seconds.", total_time)

def accuracyTest10Cm():
  global position, total_time, accuracy_10cm
  if (accuracy_10cm and (abs(position["x"]) >= 0.10 or abs(position["y"]) >= 0.10 or abs(position["z"]) >= 0.10)):
    accuracy_10cm = False
    rospy.logwarn("Failed 10cm accuracy at in %f seconds.", total_time)

def getRotationMatrix(quaternion):
  # Normalization
  s = (x**2 + y**2 + z**2 + w**2)**0.5 
  
  r_11 = 1 - 2*s*(y**2 + z**2) 
  r_12 = 2*s*(x*y - z*w)
  r_13 = 2*s*(x*z + y*w)
  r_21 = 2*s*(x*y + z*w)
  r_22 = 1 - 2*s*(x**2 + z**2)
  r_23 = 2*s*(y*z - x*w)
  r_31 = 2*s*(x*z - y*w)
  r_32 = 2*s*(y*z + x*w)
  r_33 = 1 - 2*s*(x**2 + y**2)
  
  rotation_matrix = [[r_11, r_12, r_13],
  		    [r_21, r_22, r_23],
  		    [r_31, r_32, r_33]]
  
  return rotation_matrix

"""

def imuCallback(imu_msg):
  global position, previous_time, total_time, accuracy_5cm, accuracy_10cm, start, odom_pub, odom_msg
  
  if not start:
   # Set starting time stamp to current time.
    previous_time = imu_msg.header.stamp.to_sec()
    rospy.loginfo("Initial time: %f:", previous_time)
    start = True
  
  else:
    message_time = imu_msg.header.stamp.to_sec()
    #rospy.loginfo("Message sequence: %f, Message time: %f", imu_msg.header.seq, message_time)
    time_difference = message_time - previous_time
    #total_time += time_difference 
    #rospy.loginfo("Time difference: %f", time_difference)
    # Set previous_time to (current) message_time for next callback.
    previous_time = message_time

    linear_acceleration_x = imu_msg.linear_acceleration.x
    linear_acceleration_y = imu_msg.linear_acceleration.y
    linear_acceleration_z = imu_msg.linear_acceleration.z

    linear_velocity_x = linear_acceleration_x * time_difference
    linear_velocity_y = linear_acceleration_y * time_difference
    linear_velocity_z = linear_acceleration_z * time_difference

    #linear_displacement_x = linear_velocity_x * time_difference + 0.5 * linear_acceleration_x * time_difference * time_difference
    #linear_displacement_y = linear_velocity_y * time_difference + 0.5 * linear_acceleration_y * time_difference * time_difference
    #linear_displacement_z = linear_velocity_z * time_difference + 0.5 * linear_acceleration_z * time_difference * time_difference
    linear_displacement_x = linear_velocity_x * time_difference
    linear_displacement_y = linear_velocity_y * time_difference
    linear_displacement_z = linear_velocity_z * time_difference

    position["x"] += linear_displacement_x
    position["y"] += linear_displacement_y
    position["z"] += linear_displacement_z

    odom_msg.header.stamp = rospy.get_rostime()
    odom_msg.header.frame_id = "imu_link"

    odom_msg.pose.pose.position.x = position["x"]
    odom_msg.pose.pose.position.y = position["y"]
    odom_msg.pose.pose.position.z = position["z"]
 
    odom_pub.publish(odom_msg)

if __name__ == '__main__':
 
    rospy.init_node("imu_localization_node") 
    rospy.loginfo("Initial position X: %f, Y: %f, Z: %f", position["x"], position["y"], position["z"])
 
    imu_sub = rospy.Subscriber("/imu", Imu, imuCallback)

    rospy.spin()
