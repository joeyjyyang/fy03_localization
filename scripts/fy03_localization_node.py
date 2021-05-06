#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped

from matplotlib.pyplot import *
from random import *
from math import *
import scipy as scipy
import scipy.stats
from numpy import *
import threading

class LocalizationNode:
    
	def __init__(self):

		# Constants
		self.x_tag = 0 # Tag x coord from UWB
		self.y_tag = 0 # Tag y coord from UWB
		self.x_tag_prev = 0 # Remembers previous x_coord
		self.y_tag_prev = 0 # Remembers previous y_coord
		self.x_fused = 0 # Fusion x coord from PF
		self.y_fused = 0 # Fusion y coord from PF
		self.linear_velocity_x = 0 # IMU dead-reckoning linear velocity x value
		self.linear_velocity_y = 0 # IMU dead-reckoning linear velocity y value
		self.linear_velocity_x_matrix = [0] # Tracks up to 10 most recent velocity vals
		self.linear_velocity_y_matrix = [0] # Tracks up to 10 most recent velocity vals
		self.coordinate_angle_offset = 0 # Tracks up the offset angle to covert UWB coordinates to IMU ones
		
		# ROS
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCallback, queue_size = 1)
		self.uwb_tag_sub = rospy.Subscriber("/tag", Marker, self.tagCallback, queue_size = 1)
		self.fused_pose_pub = rospy.Publisher('/fused_pose', Odometry, queue_size = 1)
		self.fused_pose_msg = Odometry()
		self.odom_base_link_br = tf2_ros.TransformBroadcaster()
		self.odom_base_link_tf = TransformStamped()

	def imuCallback(self, imu_msg):
		# Populate orientation data (not estimated through particle filter)	
		self.fused_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
		self.fused_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
		self.fused_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
		self.fused_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

		# Populate orientation data for transform broadcaster
		self.odom_base_link_tf.transform.rotation.x = imu_msg.orientation.x
		self.odom_base_link_tf.transform.rotation.y = imu_msg.orientation.y
		self.odom_base_link_tf.transform.rotation.z = imu_msg.orientation.z
		self.odom_base_link_tf.transform.rotation.w = imu_msg.orientation.w
    	
		# Send acceleration data to particle filter
		linear_acceleration_x = imu_msg.linear_acceleration.x
		linear_acceleration_y = imu_msg.linear_acceleration.y
		u = [linear_acceleration_x, linear_acceleration_y]
		self.IMU_data(u)
	
	def tagCallback(self, tag_msg):
		self.x_tag = tag_msg.pose.position.x
		self.y_tag = tag_msg.pose.position.y
		self.UWB_data()


	# Prediction step.

	#The IMU runs 100 times per second
	#The UWB runs 10 times per second
	#The IMU runs 10 times per UWB data point



	def IMU_data(self, u, dt = 0.01):						##consider replacing dt with an actual time step from the system clock
		length = len(self.linear_velocity_x_matrix)
		#current velocity = previous velocity + (acceleration * time step)
		self.linear_velocity_x = self.linear_velocity_x_matrix[length-1] + u[0]*dt
		self.linear_velocity_y = self.linear_velocity_y_matrix[length-1] + u[1]*dt
		#add curent velocity to list of velocities
		self.linear_velocity_x_matrix.append(self.linear_velocity_x)
		self.linear_velocity_y_matrix.append(self.linear_velocity_y)

	# Update step.
	# Get observation of position from UWB and update weights of particles.

	#Calculate the offset angle for axis alignment
	def calc_offset_angle(self, UWBx, UWBy, IMUx, IMUy):	
		theta1 = atan2(IMUy, IMUx)		
		theta2 = atan2(UWBy, UWBx)
		self.coordinate_angle_offset = theta1 - theta2

	def UWB2IMU(self, UWBx, UWBy):
		magnitude = sqrt(UWBx*UWBx + UWBy*UWBy)
		theta = atan2(UWBy, UWBx)
		new_theta = theta + self.coordinate_angle_offset
		return cos(new_theta)*magnitude, sin(new_theta)*magnitude
	
	#happens 10 times per second
	#after 10 IMU data points

	def UWB_data(self, dt = 0.1):						##consider replacing dt with an actual time step from the system clock
		#current velocity = dx/dt
		UWB_vel_x = (self.x_tag-self.x_tag_prev)/dt
		UWB_vel_y = (self.y_tag-self.y_tag_prev)/dt
	

		print("IMU_vel x,y: " + str(self.linear_velocity_x) + ", " + str(self.linear_velocity_y) + " UWB_vel x,y: " + str(UWB_vel_x) + ", " + str(UWB_vel_y) + "\n")
				
			##add self correction step where last n (prob 50-100) UWB time steps are compared with IMU velocity to determine IMU velocity factor in real time



		UWB_vel_x, UWB_vel_y = self.UWB2IMU(UWB_vel_x, UWB_vel_y) 
		self.linear_velocity_x = (self.linear_velocity_x+UWB_vel_x)/2		##could replace with sophisticated EKF step
		self.linear_velocity_y = (self.linear_velocity_y+UWB_vel_y)/2		##alternatively could use tuning parameters to weigh the average (e.g 0.7x+0.3y)	

		#length = len(self.linear_velocity_x_matrix)
		#x_vel_ave = 0
		#y_vel_ave = 0
		#for i in range(length):
		#	x_vel_ave = x_vel_ave + self.linear_velocity_x_matrix[i]   	##v1 * t1 + v2 * t2 + v3 * t3 = x
		#	y_vel_ave = y_vel_ave + self.linear_velocity_y_matrix[i]	##t1 = t2 = t3 = t
				##(v1 + v2 + v3)t = x
											
		
		#x_vel_ave = x_vel_ave/length
		#y_vel_ave = y_vel_ave/length

		self.x_fused = 0.9*self.x_tag + 0.1*(self.x_tag_prev + self.linear_velocity_x*dt)	##could replace with sophisticated EKF step
		self.y_fused = 0.9*self.y_tag + 0.1*(self.y_tag_prev + self.linear_velocity_y*dt)

		#print("x: " + str(self.x_fused) + "y: " + str(self.y_fused))

		self.linear_velocity_x_matrix = [self.linear_velocity_x]
		self.linear_velocity_y_matrix = [self.linear_velocity_y]

		self.x_tag_prev = self.x_tag
		self.y_tag_prev = self.y_tag

		self.publishFusedPose()


		##add axis allignment step every 15-30 seconds, use x,y accel/vel from IMU and UWB displacement to allign.

	# Publish best approximation to ROS.    
	def publishFusedPose(self):
		time_stamp = rospy.get_rostime()
		frame_id = "odom"
		child_frame_id = "base_link"

		# Populate fused pose message
		self.fused_pose_msg.header.stamp = time_stamp
		self.fused_pose_msg.header.frame_id = frame_id
		self.fused_pose_msg.child_frame_id = child_frame_id

		self.fused_pose_msg.pose.pose.position.x = self.x_fused
		self.fused_pose_msg.pose.pose.position.y = self.y_fused

		rospy.loginfo("x: %f, y: %f" % (self.fused_pose_msg.pose.pose.position.x, self.fused_pose_msg.pose.pose.position.y))

		# Populate odom->base_link transform
		self.odom_base_link_tf.header.stamp = time_stamp
		self.odom_base_link_tf.header.frame_id = frame_id
		self.odom_base_link_tf.child_frame_id = child_frame_id

		self.odom_base_link_tf.transform.translation.x = self.x_fused
		self.odom_base_link_tf.transform.translation.y = self.y_fused
            
		# Publish and broadcast  
		self.odom_base_link_br.sendTransform(self.odom_base_link_tf)
		self.fused_pose_pub.publish(self.fused_pose_msg)

if __name__ == '__main__':
	rospy.init_node("fy03_localization_node")
	localization_node = LocalizationNode()
  
	#rospy.on_shutdown(localization_node.clean)

	try:
		rospy.spin() 
	except rospy.ROSInterruptException:
		pass

