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
# sudo apt-get install python-sklearn
from sklearn.cluster import DBSCAN
import threading
from fusion_algorithm_class import *
# from old_fusion_algorithm_class import *

class LocalizationNode:
   	def __init__(self):
		self.x_tag = 0
		self.y_tag = 0
		self.x_tag_prev = 0
		self.y_tag_prev = 0
		self.fusion = FusionAlgorithm()
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
		linear_acceleration_x = imu_msg.linear_acceleration.x if abs(imu_msg.linear_acceleration.x) > 0.03 else 0
		linear_acceleration_y = imu_msg.linear_acceleration.y if abs(imu_msg.linear_acceleration.y) > 0.03 else 0
		u = [linear_acceleration_x, linear_acceleration_y]

		self.fusion.take_avg(u)
	
	# def anomaly_detection(self, u):
	# 	# DBSCAN to detect anomaly
	# 	if len(self.accum) < 1000:
	# 		self.accum.append(u)
	# 	elif len(self.accum) == 1000:
	# 		outlier_detection = DBSCAN(min_samples = 100, eps = 5)
	# 		clusters = outlier_detection.fit_predict(self.accum)
	# 		print(list(clusters).count(-1) == 0)
	# 		self.accum.pop(0)
	# 		self.accum.append(u)

		# 1.5 IQR to detect anomaly
		# if len(self.accum) < 1000:
		# 	self.accum.append(u[0])
		# elif len(self.accum) == 1000:
		# 	Q1 = percentile(self.accum, 25)
		# 	Q3 = percentile(self.accum, 75)
		# 	IQR = Q3 - Q1
		# 	if u[0] < (Q1 - IQR*4.5) or u[1] > (Q3 + 4.5*IQR):
		# 		print("WARNING WARNING WARNING")
		# 	self.accum.pop(0)
		# 	self.accum.append(u[0])

	def tagCallback(self, tag_msg, dt = 0.1):
		self.x_tag = tag_msg.pose.position.x
		self.y_tag = tag_msg.pose.position.y
        	UWB_vel_x = (self.x_tag-self.x_tag_prev)/dt
		UWB_vel_y = (self.y_tag-self.y_tag_prev)/dt
		fused_x, fused_y = self.fusion.UWB_data([self.x_tag_prev, self.y_tag_prev], [self.x_tag, self.y_tag], [UWB_vel_x, UWB_vel_y])
    		self.x_tag_prev = self.x_tag
		self.y_tag_prev = self.y_tag
        	self.publishFusedPose(fused_x, fused_y)


	# Publish best approximation to ROS.    
	def publishFusedPose(self, fused_x, fused_y):
		time_stamp = rospy.get_rostime()
		frame_id = "odom"
		child_frame_id = "base_link"

		# Populate fused pose message
		self.fused_pose_msg.header.stamp = time_stamp
		self.fused_pose_msg.header.frame_id = frame_id
		self.fused_pose_msg.child_frame_id = child_frame_id

		self.fused_pose_msg.pose.pose.position.x = fused_x
		self.fused_pose_msg.pose.pose.position.y = fused_y

		rospy.loginfo("x: %f, y: %f" % (self.fused_pose_msg.pose.pose.position.x, self.fused_pose_msg.pose.pose.position.y))

		# Populate odom->base_link transform
		self.odom_base_link_tf.header.stamp = time_stamp
		self.odom_base_link_tf.header.frame_id = frame_id
		self.odom_base_link_tf.child_frame_id = child_frame_id

		self.odom_base_link_tf.transform.translation.x = fused_x
		self.odom_base_link_tf.transform.translation.y = fused_y
            
		# Publish and broadcast  
		self.odom_base_link_br.sendTransform(self.odom_base_link_tf)
		self.fused_pose_pub.publish(self.fused_pose_msg)
