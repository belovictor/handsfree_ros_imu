#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler


# Find ttyUSB* devices
def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))


# crc check
def checkSum(list_data, check_data):
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])


# 16 Convert base to ieee floating point
def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str =hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
        if python_version == '2':
            ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
        if python_version == '3':
            ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data


# Process serial data
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, data_right_count

    if data_right_count > 200000:
        print("The device transmits data error, exit")
        exit(0)


    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0xaa:
        data_right_count += 1
        key = 0
        return
    if key < 3:
        return
    if buff[1] != 0x55:
        key = 0
        return
    if key < buff[2] + 5:  # According to the judgment of the data length bit, to obtain the corresponding length data
        return

    else:
        data_right_count = 0
        data_buff = list(buff.values())  # Get a dictionary so value

        if buff[2] == 0x2c and pub_flag[0]:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                data = hex_to_ieee(data_buff[7:47])
                angularVelocity = data[1:4]
                acceleration = data[4:7]
                magnetometer = data[7:10]
            else:
                print('Validation failed')
            pub_flag[0] = False
        elif buff[2] == 0x14 and pub_flag[1]:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                angle_degree = data[1:4]
            else:
                print('Validation failed')
            pub_flag[1] = False
        else:
            print("The data processing class does not provide the " + str(buff[2]) + " Analysis of")
            print("or data error")
            buff = {}
            key = 0

        buff = {}
        key = 0
        #if pub_flag[0] == True or pub_flag[1] == True:
        #    return
        pub_flag[0] = pub_flag[1] = True
        stamp = rospy.get_rostime()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "base_link"

        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = "base_link"

        angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        qua = quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])

        imu_msg.orientation.x = qua[0]
        imu_msg.orientation.y = qua[1]
        imu_msg.orientation.z = qua[2]
        imu_msg.orientation.w = qua[3]

        imu_msg.angular_velocity.x = angularVelocity[0]
        imu_msg.angular_velocity.y = angularVelocity[1]
        imu_msg.angular_velocity.z = angularVelocity[2]
        
        acc_k = math.sqrt(acceleration[0] ** 2 + acceleration[1] ** 2 + acceleration[2] ** 2)
        if acc_k == 0:
           acc_k = 1
           
        if gra_normalization:
            imu_msg.linear_acceleration.x = acceleration[0] * -9.8 / acc_k
            imu_msg.linear_acceleration.y = acceleration[1] * -9.8 / acc_k
            imu_msg.linear_acceleration.z = acceleration[2] * -9.8 / acc_k
        else:
            imu_msg.linear_acceleration.x = acceleration[0] * -9.8
            imu_msg.linear_acceleration.y = acceleration[1] * -9.8
            imu_msg.linear_acceleration.z = acceleration[2] * -9.8

        mag_msg.magnetic_field.x = magnetometer[0]
        mag_msg.magnetic_field.y = magnetometer[1]
        mag_msg.magnetic_field.z = magnetometer[2]

        imu_pub.publish(imu_msg)
        mag_pub.publish(mag_msg)


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True]
data_right_count = 0


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    rospy.init_node("imu")
    rospy.loginfo("Starting handsfree IMU node")
    port = rospy.get_param("~port", "/dev/imu")
    baudrate = rospy.get_param("~baudrate", 921600)
    gra_normalization = rospy.get_param("~gra_normalization", True)
    imu_msg = Imu()
    mag_msg = MagneticField()
    try:
        hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=5.0)
        if hf_imu.isOpen():
            rospy.loginfo("Serial port opened successfully")
        else:
            hf_imu.open()
            rospy.loginfo("Serial port opened successfully")
    except Exception as e:
        print(e)
        rospy.loginfo("Failed to open serial port")
        exit(0)
    else:
        imu_pub = rospy.Publisher("handsfree/imu", Imu, queue_size=10)
        mag_pub = rospy.Publisher("handsfree/mag", MagneticField, queue_size=10)

        while not rospy.is_shutdown():
            try:
                byte = hf_imu.read(1)
                buff_count = hf_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("Lost IMU connection")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = hf_imu.read(buff_count)
                    handleSerialData(byte[0])
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
    rospy.loginfo("Shutting down handsfree IMU node")
