#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from HandsFreeImu import *


class RosImu:
    def __init__(self):
        pass

    def imu_publisher(self, classImu, port_):

        imu = classImu
        imu_ser = SerialTransport()
        imu_ser.openSerial(port_, 921600, 1)
        imu_ser.start(imu.handleSerialData)
        imu_pub = rospy.Publisher("handsfree/imu", Imu, queue_size=10)
        mag_pub = rospy.Publisher("handsfree/mag", MagneticField, queue_size=10)
        # t0 = time.time()
        while not rospy.is_shutdown():
            if [imu.finish_round[i] for i in range(0, len(imu.finish_round))] == [True for i in range(0, len(imu.finish_round))]:
                # print("用时%s" % (time.time() - t0))
                # t0 = time.time()
                stamp = rospy.get_rostime()
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "base_link"

                # 调用 eul_to_qua , 将欧拉角转四元数
                imu_msg.orientation = eul_to_qua(imu.euler_angle)

                imu_msg.angular_velocity.x = imu.angular_velocity[0]
                imu_msg.angular_velocity.y = imu.angular_velocity[1]
                imu_msg.angular_velocity.z = imu.angular_velocity[2]

                imu_msg.linear_acceleration.x = imu.acceleration[0]
                imu_msg.linear_acceleration.y = imu.acceleration[1]
                imu_msg.linear_acceleration.z = imu.acceleration[2]

                imu_pub.publish(imu_msg)

                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = "base_link"
                mag_msg.magnetic_field.x = imu.magnetometer[0]
                mag_msg.magnetic_field.y = imu.magnetometer[1]
                mag_msg.magnetic_field.z = imu.magnetometer[2]

                mag_pub.publish(mag_msg)
                imu.finish_round = [False for i in range(0, len(imu.finish_round))]
