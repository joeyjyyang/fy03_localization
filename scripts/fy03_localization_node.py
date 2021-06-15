#!/usr/bin/env python

from localization_class import *

if __name__ == '__main__':
	rospy.init_node("fy03_localization_node")
	localization_node = LocalizationNode()

	#rospy.on_shutdown(localization_node.clean)

	try:
		rospy.spin() 
	except rospy.ROSInterruptException:
		pass
		

