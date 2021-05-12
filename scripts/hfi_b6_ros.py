#!/usr/bin/env python
# -*- coding:utf-8 -*-

from RosImu import *


if __name__ == '__main__':
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    try:
        RosImu().imu_publisher(ImuB(), port)
    except rospy.ROSInterruptException as e:
        print(e)
