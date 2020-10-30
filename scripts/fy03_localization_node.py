#!/usr/bin/env python

import rospy
# from ros_dwm1001.msg import Tag, Anchor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class LocalizationNode(object):
    def __init__(self):
        #
        # Instantiate Localization object.
        # self.localization = Localization()
        #

        # self.tag_msg_ = Tag()
        # self.anchor_msg_ = Anchor()
        self.imu_msg_ = Imu()
        self.fused_pose_msg_ = Odometry()
        
        # self.uwb_tag_sub_ = rospy.Subscriber("/uwb_tag", Tag, self.tagCallback)
        # self.uwb_anchor_sub_ = rospy.Subscriber("/uwb_anchor", Anchor, self.anchorCallback)
        self.imu_sub_ = rospy.Subscriber("/imu", Imu, self.imuCallback)
        self.fused_pose_pub_ = rospy.Publisher('/fused_pose', Odometry, queue_size = 1)

    def run(self):
        rospy.spin()
    
    '''
    def tagCallback(self, tag_msg):
        self.publishFusedPose(fused_pose)

    def anchorCallback(self, anchor_msg):
        self.publishFusedPose(fused_pose)
        '''

    def imuCallback(self, imu_msg):
        # self.publishFusedPose(fused_pose)
	pass

    def publishFusedPose(self, fused_pose):
        # self.fused_pose_pub_.publish(self.fused_pose_msg_)
        pass

if __name__ == '__main__':
    rospy.init_node("fy03_localization_node")
    localization_node = LocalizationNode()
    
    try:
        localization_node.run()
    except rospy.ROSInterruptException:
        pass

