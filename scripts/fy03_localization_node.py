#!/usr/bin/env python

import rospy
from ros_dwm1001.msg import Tag, Anchor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from matplotlib.pyplot import *
from random import *
from math import *
import scipy as scipy
#import scipy.stats
from numpy import *

class LocalizationNode:
    def __init__(self):
        #
        # Instantiate Localization object.
        # self.localization = Localization()
        #

        self.tag_msg_ = Tag()
        self.anchor_msg_ = Anchor()
        self.imu_msg_ = Imu()
        self.fused_pose_msg_ = Odometry()
        
        self.uwb_tag_sub_ = rospy.Subscriber("/tag", Tag, self.tagCallback)
        self.uwb_anchor_sub_ = rospy.Subscriber("/anchor1", Anchor, self.anchor1Callback)
        self.uwb_anchor_sub_ = rospy.Subscriber("/anchor2", Anchor, self.anchor2Callback)
        self.uwb_anchor_sub_ = rospy.Subscriber("/anchor3", Anchor, self.anchor3Callback)
        self.uwb_anchor_sub_ = rospy.Subscriber("/anchor4", Anchor, self.anchor4Callback)
        self.imu_sub_ = rospy.Subscriber("/imu", Imu, self.imuCallback)
        self.fused_pose_pub_ = rospy.Publisher('/fused_pose', Odometry, queue_size = 1)

	# Particle Filter.	
	self.num_particles = 1
    	self.x_range = (0,10)
    	self.y_range = (0,10)
    	self.x_vals = []
    	self.y_vals = []
    	self.weights = []
    	self.current_estimate = ()
    	self.normalized_weights = []
    	self.UWB_covariance = 0.3
        self.x_anchors = [0, 0, 0, 0]
        self.y_anchors = [0, 0, 0, 0]

	# Create initial particles.
    	self.initialize_particles()
    	
    def tagCallback(self, tag_msg):
	#rospy.loginfo("Tag X: %f, Tag Y: %f", tag_msg.x, tag_msg.y)

	tag_observation_x = tag_msg.x
	tag_observation_y = tag_msg.y
	# quality_factor = tag_msg.quality_factor	

	z = [tag_observation_x, tag_observation_y]
	self.update(z)

    def anchor1Callback(self, anchor1_msg):
        self.x_anchors[0] = anchor1_msg.x
        self.y_anchors[0] = anchor1_msg.y
    
    def anchor2Callback(self, anchor2_msg):
        self.x_anchors[1] = anchor2_msg.x
        self.y_anchors[1] = anchor2_msg.y
    
    def anchor3Callback(self, anchor3_msg):
        self.x_anchors[2] = anchor3_msg.x
        self.y_anchors[2] = anchor3_msg.y
    
    def anchor4Callback(self, anchor4_msg):
        self.x_anchors[3] = anchor4_msg.x
        self.y_anchors[3] = anchor4_msg.y

    def imuCallback(self, imu_msg):
 	#rospy.loginfo("ACC X: %f, ACC Y: %f", imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y)
   	
	#message_time = imu_msg.header.stamp.to_sec()
    	#rospy.loginfo("Message sequence: %f, Message time: %f", imu_msg.header.seq, message_time)

    	linear_acceleration_x = imu_msg.linear_acceleration.x
    	linear_acceleration_y = imu_msg.linear_acceleration.y
	
	u = [linear_acceleration_x, linear_acceleration_y]	
	self.predict(u)

    # Prediction step.
    # Get motion data (control inputs) from IMU and move particles.
    def predict(self, u, dt = 0.01): #std: float):
    	linear_acceleration_x = u[0]
	linear_acceleration_y = u[1]
    
    	linear_velocity_x = linear_acceleration_x * dt
    	linear_velocity_y = linear_acceleration_y * dt

    	linear_displacement_x = linear_velocity_x * dt
    	linear_displacement_y = linear_velocity_y * dt

    	for i in range(self.num_particles):
	    self.x_vals[i] += linear_displacement_x
	    self.y_vals[i] += linear_displacement_y

    # Update step.
    # Get observation of position from UWB and update weights of particles.
    def update(self, z):
	# self.UWB_covariance
	pass

    # Initial generation of particle based on uniform distribution.
    def initialize_particles(self):
	for i in range(self.num_particles):
            self.x_vals.append(uniform(self.x_range[0], self.x_range[1]))
            self.y_vals.append(uniform(self.y_range[0], self.y_range[1]))
    
    def publishFusedPose(self, fused_pose):
        self.fused_pose_msg.pose.pose.position.x = fused_pose[0]
        self.fused_pose_msg.pose.pose.position.y = fused_pose[1]
        self.fused_pose_msg.pose.pose.position.z = fused_pose[2]

        self.fused_pose_pub_.publish(self.fused_pose_msg_)
 
if __name__ == '__main__':
    rospy.init_node("fy03_localization_node")
    localization_node = LocalizationNode()

    try:
  	rospy.spin() 
    except rospy.ROSInterruptException:
        pass

