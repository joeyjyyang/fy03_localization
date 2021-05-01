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
import threading

class LocalizationNode:
    
    def __init__(self):
	# Particle Filter.	
        # Tuneable
	self.num_particles = 3000 #5000
	self.IMU_messages_per_move = 10 
	#This parameter relates to self.linear_displacement_counter
	#This defines the number of IMU steps stored before moving all points
    	
	self.x_range = (0, 3.5) # (0, 2.85)
    	self.y_range = (0, 6.5) #(0, 5.5)
        # Used in update step

        # Too high -> all particles get same weight
        # Too low -> one particle gets all the weight
        # Extremely sensitive
        self.distance_std_dev = 0.3 #0.2 to 0.5 (ideally)

        # Too high -> steady state error too high on average
        # Too low -> will not converge quickly
        # Used in resample step
        self.UWB_covariance = 1.5 #5.0

        # Constants
	self.x_vals = [0] * self.num_particles # Initialize x coords of particles
    	self.y_vals = [0] * self.num_particles # Initialize y coords of particles
    	self.weights = [0] * self.num_particles # Initialize weights of particles
    	self.current_estimate = ()
    	self.normalized_weights = [0] * self.num_particles # Initialize normalized weights of particles
	self.x_tag = 0 # Tag x coord from UWB
	self.y_tag = 0 # Tag y coord from UWB
	self.x_tag_prev = 0 # Remembers previous x_coord
	self.y_tag_prev = 0 # Remembers previous y_coord
	self.x_fused = 0 # Fusion x coord from PF
	self.y_fused = 0 # Fusion y coord from PF
	self.linear_velocity_x = 0 # IMU dead-reckoning linear velocity x value
	self.linear_velocity_y = 0 # IMU dead-reckoning linear velocity y value
	self.linear_displacement_x = 0 # Saves displacement over short periods of time
	self.linear_displacement_y = 0 # Saves displacement over short periods of time
	self.linear_displacement_counter = 0 #IMU dead-reckoning accumulation counter

        # Zero Velocity Detector
        self.vel_tolerance = 0.1 #tuneable variable (m/s)
        self.zero_vel_offset = 0.3 #tuneable variable (m)
        self.zero_vel_matrix = [] # Stores previous tag positions in instances of zero velocity
        self.zero_vel_sensitivity_parameter = 3 #tuneable variable 

	# High Velocity Detector
	self.high_velocity_multiplier = 2 # By what factor can valocity be over before we reset it?

	# ROS
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCallback, queue_size = 1)
        self.uwb_tag_sub = rospy.Subscriber("/tag", Marker, self.tagCallback, queue_size = 1)
        self.fused_pose_pub = rospy.Publisher('/fused_pose', Odometry, queue_size = 1)
	self.fused_pose_msg = Odometry()
       	self.odom_base_link_br = tf2_ros.TransformBroadcaster()
        self.odom_base_link_tf = TransformStamped()

	# Create initial particles.
    	self.initializeParticles()
        
        # Start thread.
        self.update_thread = threading.Thread(target = self.runUpdate)
        self.update_thread.daemon = True
        self.update_thread.start()

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
        
        self.predict(u)
	
    def tagCallback(self, tag_msg):
        self.x_tag = tag_msg.pose.position.x
        self.y_tag = tag_msg.pose.position.y
        # quality_factor = tag_msg.quality_factor	
    
        #self.update()

    # Prediction step.
    # Get motion data (control inputs) from IMU and move particles.
    #
    # Also performs zero velocity check if linear velocity readings.	self.linear_displacement_counter = self.linear_displacement_counter +1
    def predict(self, u, dt = 0.01):
    	linear_acceleration_x = u[0]
	linear_acceleration_y = u[1]
    
    	self.linear_velocity_x = self.linear_velocity_x + (linear_acceleration_x * dt)
    	self.linear_velocity_y = self.linear_velocity_y + (linear_acceleration_y * dt)
	
        #linear_velocity_magnitude = sqrt((self.linear_velocity_x**2) + (self.linear_velocity_y**2))
        
        # If velocity is under the tolerance, check for zero velocity!
        #if linear_velocity_magnitude < self.vel_tolerance:
    	if self.zeroVelocityCheck():
	    self.linear_velocity_x = 0
	    self.linear_velocity_y = 0   
 	
    	self.linear_displacement_x = self.linear_displacement_x + self.linear_velocity_x * dt
    	self.linear_displacement_y = self.linear_displacement_y + self.linear_velocity_y * dt

	if(1):
	    	for i in range(self.num_particles):
		    	self.x_vals[i] += self.linear_displacement_x
		    	self.y_vals[i] += self.linear_displacement_y
			self.linear_displacement_x = 0
			self.linear_displacement_y = 0
			self.linear_displacement_counter = 0


    def runUpdate(self):
        while True:
            self.update()

    # Update step.
    # Get observation of position from UWB and update weights of particles.
    def update(self):
	self.highVelocityCheck()
	distance = 0
	temp_weight = 0

	for i in range(self.num_particles):
	    distance = self.p2pDistance(self.x_vals[i], self.y_vals[i])
	    temp_weight = (scipy.stats.norm.pdf(distance, 0, self.distance_std_dev))
	    if temp_weight == 1:
		temp_weight = 0
            self.weights[i] = temp_weight
	    self.weights[i] += 0.00000001

	self.normalizeWeights()
	self.x_tag_prev = self.x_tag # These will be used next iteration by highVelocityCheck()
	self.y_tag_prev = self.y_tag 

    # Initial generation of particle based on uniform distribution.
    def initializeParticles(self):
	for i in range(self.num_particles):
            self.x_vals[i] = uniform(self.x_range[0], self.x_range[1])
            self.y_vals[i] = uniform(self.y_range[0], self.y_range[1])

    # Triggerd by potentially low velocity reading from IMU.
    # Verifies the current instance as an instance of truly zero velocity.
    def zeroVelocityCheck(self):
        zero_vel = True
	zero_vel_confirm = True

        # Compare last read (current) UWB tag position with previous UWB tag positions
        for previous_tag_pose in self.zero_vel_matrix:
	    # If last read (current) UWB tag position is NOT within range of all previous UWB tag positions,
	    # then it is NOT an instance of zero velocity.
            if not ((previous_tag_pose[0] - self.zero_vel_offset < self.x_tag) &
               (previous_tag_pose[0] + self.zero_vel_offset > self.x_tag) &
               (previous_tag_pose[1] - self.zero_vel_offset < self.y_tag) &
               (previous_tag_pose[1] + self.zero_vel_offset > self.y_tag)):
		zero_vel = False

        # If instance of zero velocity, add current tag pose to matrix.
        if zero_vel:
            self.zero_vel_matrix.append([self.x_tag, self.y_tag])
	else:
	    self.zero_vel_matrix = []

        # If number of zero velocity instances exceed threshold, notify zero velocity.
        if len(self.zero_vel_matrix) < self.zero_vel_sensitivity_parameter:
            zero_vel_confirm = False
        
        return zero_vel_confirm

    def highVelocityCheck(self):
	velocity_x = self.x_tag-self.x_tag_prev
	velocity_y = self.y_tag-self.y_tag_prev

	if(velocity_x > self.high_velocity_multiplier*self.linear_velocity_x):
		self.linear_velocity_x = velocity_x
	if(velocity_y > self.high_velocity_multiplier*self.linear_velocity_y):
		self.linear_velocity_y = velocity_y
		

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
    def p2pDistance(self, x_particle, y_particle):
	x_diff = float(self.x_tag - x_particle)
	y_diff = float(self.y_tag - y_particle)
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
	
        #rospy.loginfo(self.x_vals)

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

    def clean(self):
        rospy.loginfo("Cleaning up threads.")
        self.update_thread.join()

if __name__ == '__main__':
    rospy.init_node("fy03_localization_node")
    localization_node = LocalizationNode()
  
    #rospy.on_shutdown(localization_node.clean)

    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass

