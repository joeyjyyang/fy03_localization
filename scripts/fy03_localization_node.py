#!/usr/bin/env python

import rospy
from ros_dwm1001.msg import Tag, Anchor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from matplotlib.pyplot import *
from random import *
from math import *
import scipy as scipy
import scipy.stats
from numpy import *

class LocalizationNode:
    def __init__(self):
        #
        # Instantiate Localization object.
        # self.localization = Localization()
        #

	# ROS.
        self.imu_msg = Imu()
        self.tag_msg = Tag()
        self.anchor_msg = Anchor()
        self.fused_pose_msg = Odometry()
        
	self.imu_sub = rospy.Subscriber("/imu", Imu, self.imuCallback)
        self.uwb_tag_sub = rospy.Subscriber("/tag", Tag, self.tagCallback)
        self.uwb_anchor_sub = rospy.Subscriber("/anchor1", Anchor, self.anchor1Callback)
        self.uwb_anchor_sub = rospy.Subscriber("/anchor2", Anchor, self.anchor2Callback)
        self.uwb_anchor_sub = rospy.Subscriber("/anchor3", Anchor, self.anchor3Callback)
        self.uwb_anchor_sub = rospy.Subscriber("/anchor4", Anchor, self.anchor4Callback)
        self.fused_pose_pub = rospy.Publisher('/fused_pose', Odometry, queue_size = 1)

	# Particle Filter.	
	self.num_particles = 5
    	self.x_range = (0,10)
    	self.y_range = (0,10)
    	self.x_vals = [0] * self.num_particles
    	self.y_vals = [0] * self.num_particles
    	self.weights = [0] * self.num_particles
    	self.current_estimate = ()
    	self.normalized_weights = [0] * self.num_particles
    	self.UWB_covariance = 0.3
        self.x_anchors = [0, 0, 0, 0]
        self.y_anchors = [0, 0, 0, 0]
	self.x_fused = 0
	self.y_fused = 0

	# Create initial particles.
    	self.initializeParticles()
     
    def imuCallback(self, imu_msg):
 	#rospy.loginfo("ACC X: %f, ACC Y: %f", imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y)
   	
	#message_time = imu_msg.header.stamp.to_sec()
    	#rospy.loginfo("Message sequence: %f, Message time: %f", imu_msg.header.seq, message_time)

    	linear_acceleration_x = imu_msg.linear_acceleration.x
    	linear_acceleration_y = imu_msg.linear_acceleration.y
	
	u = [linear_acceleration_x, linear_acceleration_y]	
	self.predict(u)
	
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
	distance = 0
	temp_weight = 0
	for i in range(self.num_particles):
	    distance = self.p2pDistance(z, self.x_vals[i], self.y_vals[i])
	    temp_weight = (scipy.stats.norm.pdf(distance, 0, self.UWB_covariance))
	    if temp_weight == 1:
		rospy.loginfo("true")
		temp_weight = 0
	    self.weights[i] = temp_weight
	    self.weights[i] += 0.00000001

	self.normalizeWeights()

	#rospy.loginfo("Weights: ")
	#rospy.loginfo(self.weights)
   
    # Initial generation of particle based on uniform distribution.
    def initializeParticles(self):
	for i in range(self.num_particles):
            self.x_vals[i] = uniform(self.x_range[0], self.x_range[1])
            self.y_vals[i] = uniform(self.y_range[0], self.y_range[1])

	rospy.loginfo("Initial particles: ")
	rospy.loginfo(self.x_vals)
	rospy.loginfo(self.y_vals)

    # Normalize weights.
    def normalizeWeights(self):
	total_weight = 0
	
	for weight in self.weights:
	    total_weight += weight
	
	for i in range(self.num_particles):
	    self.normalized_weights[i] = self.weights[i] / total_weight    

	self.getFusedPose()

	#rospy.loginfo("Normalized Weights: ")
	#rospy.loginfo(self.normalized_weights)

    # Calculates the distance between 2 points.
    # Specifically, the distance between each particle,
    # and the UWB measurement of position.
    def p2pDistance(self, z, x_particle, y_particle):
	x_measurement = z[0]
	y_measurement = z[1]
	x_diff = float(x_measurement - x_particle)
	y_diff = float(y_measurement - y_particle)
	distance = sqrt((x_diff**2) + (y_diff**2))

	return distance

    # Resample step.
    # Regenerate particles with no weights.
    def resample(self):
	cum_sum = 0
	counter = 0
	prev_x_vals = self.x_vals
	prev_y_vals = self.y_vals
	self.x_vals = [0] * self.num_particles
	self.y_vals = [0] * self.num_particles
	self.weights = [0] * self.num_particles
	
	for i in range(self.num_particles):
	    cum_sum += self.normalized_weights[i] * float(self.num_particles)
	    
	    while (counter < cum_sum):
	        self.x_vals[i] = random.normal(prev_x_vals[i], self.UWB_covariance)
		self.y_vals[i] = random.normal(prev_y_vals[i], self.UWB_covariance)
		counter += 1

	for i in range(self.num_particles):
	    self.weights[i] = 1

	#rospy.loginfo(self.x_vals)
	#rospy.loginfo(self.y_vals)
	#rospy.loginfo(self.weights)

    # Determine best approximation based on weighted particles.
    def getFusedPose(self):
	self.x_fused = 0
	self.y_fused = 0

	for i in range(self.num_particles):
	    self.x_fused += self.x_vals[i] * self.weights[i]
 	    self.y_fused += self.y_vals[i] * self.weights[i]

	self.publishFusedPose()
	self.resample()
	
    # Publish best approximation to ROS.    
    def publishFusedPose(self):
	time_stamp = rospy.get_rostime()
	self.fused_pose_msg.header.stamp = time_stamp

        self.fused_pose_msg.pose.pose.position.x = self.x_fused
        self.fused_pose_msg.pose.pose.position.y = self.y_fused
        #self.fused_pose_msg.pose.pose.position.z = fused_pose[2]

        self.fused_pose_pub.publish(self.fused_pose_msg)
 
if __name__ == '__main__':
    rospy.init_node("fy03_localization_node")
    localization_node = LocalizationNode()

    try:
  	rospy.spin() 
    except rospy.ROSInterruptException:
        pass

