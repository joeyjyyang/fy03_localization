#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    
    rospy.init_node("fy03_localization_node")
    rate = rospy.Rate(15)
    
    try:
        while not rospy.is_shutdown():
            rate.sleep()
   
    except rospy.ROSInterruptException:
        pass

