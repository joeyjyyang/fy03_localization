#!/usr/bin/env python

import rospy
import tf2_ros
from ros_dwm1001.msg import Tag, Anchor
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

class LocalizationNode:
    def __init__(self):
        #
        # Instantiate Localization object.
        # self.localization = Localization()
        #
	
	# Sensor initialization flags	
	self.imu_init = False
	self.uwb_init = False

	# Particle Filter.	
	self.num_particles = 1000
    	self.x_range = (0, 2.0) # Size of 
    	self.y_range = (0, 2.0)
    	self.x_vals = [0] * self.num_particles # Initialize x coords of particles
    	self.y_vals = [0] * self.num_particles # Initialize y coords of particles
    	self.weights = [0] * self.num_particles # Initialize weights of particles
    	self.current_estimate = ()
    	self.normalized_weights = [0] * self.num_particles # Initialize normalized weights of particles
    	self.UWB_covariance = 0.03 #tuneable variable
	self.x_tag = 0 # Tag x coord from UWB
	self.y_tag = 0 # Tag y coord from UWB
	self.x_fused = 0 # Fusion x coord from PF
	self.y_fused = 0 # Fusion y coord from PF
	self.linear_velocity_x = 0 # IMU dead-reckoning linear velocity x value
	self.linear_velocity_y = 0 # IMU dead-reckoning linear velocity y value
        
        # Zero Velocity Detector
        self.vel_tolerance = 1.0 #tuneable variable
        self.low_vel = 0
        self.zero_vel_counter = 0
        self.zero_vel_matrix = []
        self.zero_vel = 0
        self.zero_vel_sensitivity_parameter = 2    
        self.zero_vel_confirmed = 0

	# ROS
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCallback)
        self.uwb_tag_sub = rospy.Subscriber("/tag", Marker, self.tagCallback)
        self.fused_pose_pub = rospy.Publisher('/fused_pose', Odometry, queue_size = 1)
	self.fused_pose_msg = Odometry()
       	self.odom_base_link_br = tf2_ros.TransformBroadcaster()
        self.odom_base_link_tf = TransformStamped()
	
	# Create initial particles.
    	self.initializeParticles()
     
    def imuCallback(self, imu_msg):
	# Toggle upon first message.
	if not self.imu_init:
	    self.imu_init = True

	if self.checkSensorsInit():
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
            self.predict(u)
	
    def tagCallback(self, tag_msg):
	# Toggle upon first message.
	if not self.uwb_init:
	    self.uwb_init = True

	# Only publish when IMU and UWB both initialized.
	if self.checkSensorsInit():
	    self.x_tag = tag_msg.pose.position.x
	    self.y_tag = tag_msg.pose.position.y
	    # quality_factor = tag_msg.quality_factor	
	    z = [self.x_tag, self.y_tag]
	    self.update(z)

    # Check that IMU and UWB sensors are initialized.
    def checkSensorsInit(self):
	if self.imu_init and self.uwb_init:
	    return True
	else:
	    return False

    # Prediction step.
    # Get motion data (control inputs) from IMU and move particles.
    #
    # Also performs zero velocity check if linear velocity readings.
    def predict(self, u, dt = 0.01): #std: float):
    	linear_acceleration_x = u[0]
	linear_acceleration_y = u[1]
    
    	self.linear_velocity_x = self.linear_velocity_x + (linear_acceleration_x * dt)
    	self.linear_velocity_y = self.linear_velocity_y + (linear_acceleration_y * dt)
	"""
        linear_velocity_magnitude = sqrt((self.linear_velocity_x**2) + (self.linear_velocity_y**2))
        
        # If velocity is under the tolerance, check for zero velocity!
        if linear_velocity_magnitude < self.vel_tolerance:
            self.zero_vel, zero_vel_confirmed = zero_vel_check(zero_vel_matrix, current_estimate, UWB_covariance, zero_vel_sensitivity_parameter)
    	
        if(zero_vel_confirmed == 1):
	    x_vel, y_vel = 0, 0
    	elif(zero_vel == 0):
	    zero_vel_matrix = []
	"""
    	linear_displacement_x = self.linear_velocity_x * dt
    	linear_displacement_y = self.linear_velocity_y * dt

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
			temp_weight = 0
	    self.weights[i] = temp_weight
	    self.weights[i] += 0.00000001

	self.normalizeWeights()

    # Initial generation of particle based on uniform distribution.
    def initializeParticles(self):
	for i in range(self.num_particles):
            self.x_vals[i] = uniform(self.x_range[0], self.x_range[1])
            self.y_vals[i] = uniform(self.y_range[0], self.y_range[1])
    """
    # Triggerd by potentially low velocity reading from IMU.
    # Verifies the current instance as an instance of truly zero velocity.
    def zeroVelocityCheck(self):
        zero_vel = False

        # Compare last read (current) UWB tag position with previous UWB tag positions
        for previous_tag_pose in self.zero_vel_matrix:
            if((previous_tag_pose[0] - self.UWB_covariance < self.x_tag) &
               (previous_tag_pose[0] + self.UWB_covariance > self.x_tag) &
               (previous_tag_pose[1] - self.UWB_covariance < self.y_tag) &
               (previous_tag_pose[1] + self.UWB_covariance > self.y_tag)):
            else:
                zero_vel = 0
                zero_vel_confirmed  = 0

        # If instance of zero velocity, add current tag pose to matrix.
        if self.zero_vel == 1:
            self.zero_vel_matrix.append([self.x_tag, self.y_tag])

        # If number of zero velocity instances exceed threshold, notify zero velocity.
        if len(self.zero_vel_matrix) < self.sensitivity_parameter:
            zero_vel = False
        else:
            zero_vel = True
        
        return zero_vel
    
    def zero_vel_check(zero_vel_matrix: list, current_estimate: tuple, UWB_covariance: float, sensitivity_parameter: int):
        zero_vel = 1
        zero_vel_confirmed = 1
        debug_counter = 0
        for i in(self.zero_vel_matrix):
            if((i[0] - self.UWB_covariance < current_estimate[0])&
               (i[0] + self.UWB_covariance > current_estimate[0])&
               (i[1] - self.UWB_covariance < current_estimate[1])&
               (i[1] + self.UWB_covariance > current_estimate[1])):
                debug_counter += 1
            else:
                zero_vel = 0
                zero_vel_confirmed  = 0
            #print(debug_counter)
        if(zero_vel == 1):
            zero_vel_matrix.append(current_estimate)
        if(len(zero_vel_matrix) < sensitivity_parameter):
            zero_vel_confirmed = 0
        return(zero_vel, zero_vel_confirmed)
    """
    # Normalize weights.
    def normalizeWeights(self):
	total_weight = 0
	
	for weight in self.weights:
	    total_weight += weight
	
	for i in range(self.num_particles):
	    self.normalized_weights[i] = self.weights[i] / total_weight    

	self.getFusedPose()

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

    # Determine best approximation based on weighted particles.
    def getFusedPose(self):
	self.x_fused = 0
	self.y_fused = 0

	for i in range(self.num_particles):
	    self.x_fused += self.x_vals[i] * self.normalized_weights[i]
	    self.y_fused += self.y_vals[i] * self.normalized_weights[i]

	self.publishFusedPose()
	self.resample()
	
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

        self.fused_pose_pub.publish(self.fused_pose_msg)

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

    try:
  	rospy.spin() 
    except rospy.ROSInterruptException:
        pass

