#!/usr/bin/env python
# -*- coding:utf-8 -*-

from HandsFreeImu import *


def main(classImu,port):
    imu = classImu
    imu_ser = SerialTransport()
    imu_ser.openSerial(port, 921600, 1)
    imu_ser.start(imu.handleSerialData)
    t0 = time.time()
    while True:
        if [imu.finish_round[i] for i in range(0, len(imu.finish_round))] == [True for i in range(0, len(imu.finish_round))]:
            # print("用时%s" % (time.time() - t0))
            t0 = time.time()

            print(
                '''
加速度(m/s²)：
    x轴：%.2f
    y轴：%.2f
    z轴：%.2f

角速度(rad/s)：
    x轴：%.2f
    y轴：%.2f
    z轴：%.2f

欧拉角(°)：
    x轴：%.2f
    y轴：%.2f
    z轴：%.2f

磁场：
    x轴：%.2f
    y轴：%.2f
    z轴：%.2f
                ''' % (
            imu.acceleration[0], imu.acceleration[1], imu.acceleration[2], imu.angular_velocity[0],
            imu.angular_velocity[1], imu.angular_velocity[2], imu.euler_angle[0], imu.euler_angle[1], imu.euler_angle[2],
            imu.magnetometer[0], imu.magnetometer[1], imu.magnetometer[2]))
            imu.finish_round = [False for i in range(0, len(imu.finish_round))]


if __name__ == '__main__':
    main(ImuB(),'COM3')

