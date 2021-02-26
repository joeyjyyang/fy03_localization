#!/usr/bin/env python

import rospy
from ros_dwm1001.msg import Tag, Anchor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from matplotlib import pyplot as plt

class PlotterNode:
    def __init__(self):
        #
        # Instantiate Localization object.
        # self.localization = Localization()
        #
	
	self.true_positions = [[0, 0], [1, 1], [2, 2]]
	self.tag_positions_x = []
	self.tag_positions_y = []
	self.anchor1_position = [0, 0]
	self.anchor2_position = [0, 0]
	self.anchor3_position = [0, 0]
	self.anchor4_position = [0, 0]
	self.fused_positions_x = []	
	self.fused_positions_y = []

	#self.imu_position_x = 0
	#self.imu_position_y = 0

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
        self.fused_pose_sub = rospy.Subscriber('/fused_pose', Odometry, self.fusedPoseCallback)

    	plt.ion()
	plt.show()

    def imuCallback(self, imu_msg):
	plt.plot(1, 5, '*')
	plt.axis("equal")
	plt.draw()
	plt.pause(0.01)

    def tagCallback(self, tag_msg):
	self.tag_positions_x.append(tag_msg.x)
    	self.tag_positions_y.append(tag_msg.y)
 
    def anchor1Callback(self, anchor1_msg):
	self.anchor1_position[0] = anchor1_msg.x 
    	self.anchor1_position[1] = anchor1_msg.y 
    
    def anchor2Callback(self,  anchor2_msg):
    	self.anchor2_position[0] = anchor2_msg.x 
    	self.anchor2_position[1] = anchor2_msg.y 
    
    def anchor3Callback(self, anchor3_msg):
    	self.anchor3_position[0] = anchor3_msg.x 
    	self.anchor3_position[1] = anchor3_msg.y 

    def anchor4Callback(self, anchor4_msg):
	self.anchor4_position[0] = anchor4_msg.x 
    	self.anchor4_position[1] = anchor4_msg.y 

    def fusedPoseCallback(self, fused_pose_msg):
	self.fused_positions_x.append(fused_pose_msg.pose.pose.position.x)
	self.fused_positions_y.append(fused_pose_msg.pose.pose.position.y)
	
def plot(msg):
	plt.plot(1, 5, '*')
	plt.axis("equal")
	plt.draw()
	plt.pause(0.01)

if __name__ == '__main__':
    rospy.init_node("plotter_node")
    plotter_node = PlotterNode()

    #imu_sub = rospy.Subscriber("/imu", Imu, plot)
 
    #rospy.on_shutdown(CLEANUP_FUNCTION)

    try:
  	rospy.spin() 
    except rospy.ROSInterruptException:
        pass

