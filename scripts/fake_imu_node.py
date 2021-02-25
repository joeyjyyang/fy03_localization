#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
 
imu_pub = rospy.Publisher("/imu", Imu, queue_size=1)
imu_msg = Imu()

def publishData():
  imu_msg.orientation.x = 0

  imu_pub.publish(imu_msg)

if __name__ == "__main__":
  rospy.init_node("fake_imu_node")
  rate = rospy.Rate(100)

  try:
    while not rospy.is_shutdown():
      publishData()
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
