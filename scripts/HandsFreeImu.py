# -*- coding:utf-8 -*-
import math
import serial
import struct
import threading
import platform
import time
import numpy

from geometry_msgs.msg import Quaternion



class Tool:
    def __init__(self):
        pass

    error = ['\033[1;31m', '\033[0m']
    warn = ['\033[1;32m', '\033[0m']
    hint = ['\033[1;32m', '\033[0m']

    def strError(self, str_):
        return self.error[0] + '[error] ' + str_ + self.error[1]

    def strWarn(self, str_):
        return self.warn[0] + '[warn]' + str_ + self.warn[1]

    def strHint(self, str_):
        return self.warn[0] + '[hint]' + str_ + self.warn[1]


t = Tool()


def eul_to_qua(Eular):
    eular_div = [0, 0, 0]
    eular_div[0], eular_div[1], eular_div[2] = Eular[0] / 2.0, Eular[1] / 2.0, Eular[2] / 2.0

    ca, cb, cc = math.cos(eular_div[0]), math.cos(eular_div[1]), math.cos(eular_div[2])
    sa, sb, sc = math.sin(eular_div[0]), math.sin(eular_div[1]), math.sin(eular_div[2])

    x = sa * cb * cc - ca * sb * sc
    y = ca * sb * cc + sa * cb * sc
    z = ca * cb * sc - sa * sb * cc
    w = ca * cb * cc + sa * sb * sc

    orientation = Quaternion()
    orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
    return orientation


class SerialTransport():
    def __init__(self):
        pass

    open_flag = False
    read_flag = False
    count_timeout = 10000
    read_timeout = 2000
    read_hint_flag = True

    def openSerial(self, port, baudrate, timeout):
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            if not self.serial.isOpen():
                self.serial.open()
            self.open_flag = True
            print(t.strHint('设备串口打开成功'))
        except Exception as e:
            self.open_flag = False
            print("SerialTransport-openSerial-exception:" + str(e))
            print(t.strHint("设备串口打开异常，请检查设备和电脑的 USB 通讯线是否连接"))
            exit(0)

    # 串口读数据处理
    def readHandler(self, methodB):
        while self.open_flag:
            if self.count_timeout > 0:
                count = self.serial.inWaiting()
                if count > 0:
                    self.count_timeout = 10000
                    try:
                        raw_buffer = self.serial.read(count)
                    except Exception as e:
                        print('SerialTransport-readHandler-exception:' + str(e))
                        print(t.strHint('设备读取异常，请查看上一行 exception 提示'))
                        self.closeSerial()
                        exit(0)
                    else:
                        for i in range(0, count):
                            self.read_flag = methodB(raw_buffer[i])
                        if self.read_timeout > 0:
                            self.read_timeout = 2000
                            if self.read_flag:
                                if self.read_hint_flag:
                                    print('设备读取到数据，并且数据能够解析成功。')
                                    self.read_hint_flag = False
                            else:
                                self.read_timeout -= 1
                        else:
                            print('设备读取到数据，但是数据未能解析成功。')
                            self.closeSerial()
                            exit(0)
                else:
                    self.count_timeout -= 1
            else:
                print('串口打开成功，但是未读取到任何数据')
                self.closeSerial()
                exit(0)


    def writeHandler(self, data):
        serial.time.sleep(0.01)
        self.serial.write(data)
        self.serial.flush()

    # 串口丢弃写入写出缓存数据后，关闭串口
    def closeSerial(self):
        self.serial.flushInput()  # 丢弃接收缓存中的所有数据。
        self.serial.flushOutput()  # 终止当前写操作，并丢弃发送缓存中的数据。
        print(t.strHint("清理串口并关闭串口"))
        self.serial.close()
        serial.time.sleep(0.5)

    # 开启线程读取数据处理方法
    def start(self, methodB):
        read = threading.Thread(target=self.readHandler, args=(methodB,))
        read.setDaemon(True)
        read.start()


#
def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        # 转字符串
        data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[
                                                                                         4:6] + hex(
            raw_data[i + 3] | 0xff00)[4:6]
        # 转ieee
        if platform.python_version()[0] == '3':
            ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str)))
        else:
            ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])

    ieee_data.reverse()
    return ieee_data


# crc 校验
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


class ImuA(object):
    def __init__(self):
        self.key = 0
        self.flag = 0
        self.buffer = {}

        # 转换的规定数据
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.euler_angle = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]

        # 原始数据
        self.raw_acceleration = [0.0, 0.0, 0.0]
        self.raw_angular_velocity = [0.0, 0.0, 0.0]
        self.raw_euler_angle = [0.0, 0.0, 0.0]
        self.raw_magnetometer = [0.0, 0.0, 0.0]

        self.finish_round = [False, False]

    def handleSerialData(self, raw_data):

        if platform.python_version()[0] == '3':
            self.buffer[self.key] = raw_data
        else:
            self.buffer[self.key] = ord(raw_data)

        self.key += 1

        if self.buffer[0] != 0xaa:
            self.buffer = {}
            self.key = 0
            return

        if self.key < 2:
            return

        if self.buffer[1] != 0x55:
            self.key = 0
            self.buffer = {}
            return

        if self.key < 3:
            return
        if self.key < self.buffer[2] + 5:
            return
        else:
            data_buffer = list(self.buffer.values())

            if self.buffer[2] == 0x2c:
                if checkSum(data_buffer[2:47], data_buffer[47:49]):

                    data = hex_to_ieee(data_buffer[7:47])

                    self.raw_acceleration = data[4:7]
                    self.raw_angular_velocity = data[1:4]
                    self.raw_magnetometer = data[7:10]

                    self.finish_round[0] = True
                else:
                    # print('check sum fail!')
                    self.key = 0
                    self.buffer = {}

            elif self.buffer[2] == 0x14:
                if checkSum(data_buffer[2:23], data_buffer[23:25]):
                    data = hex_to_ieee(data_buffer[7:23])
                    self.raw_euler_angle = data[1:4]
                    self.finish_round[1] = True
                else:
                    # print('check sum fail!')
                    self.key = 0
                    self.buffer = {}
            else:
                # print("该数据处理类没有提供该 " + str(self.buffer[2]) + " 的解析")
                # print("或数据错误")
                pass

        if self.finish_round[0] == self.finish_round[1] == True:
            self.acceleration = [self.raw_acceleration[i] * 9.8 for i in range(0, 3)]
            self.angular_velocity = self.raw_angular_velocity
            self.euler_angle = [self.raw_euler_angle[0] * math.pi / 180, self.raw_euler_angle[1] * math.pi / -180, numpy.sign(self.raw_euler_angle[2] - 180) * (abs(self.raw_euler_angle[2] - 180) % -180) * math.pi / -180 ]
            self.magnetometer = self.raw_magnetometer
            time.sleep(0.0001)
            self.buffer = {}
            self.key = 0
            return True
        else:
            self.buffer = {}
            self.key = 0
            return False


class ImuB():
    # 初始化
    def __init__(self):
        self.key = 0
        self.buffer = [0 for i in range(11)]

        # 转换的规定数据
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.euler_angle = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]

        # 原始数据
        self.raw_acceleration = [0.0, 0.0, 0.0]
        self.raw_angular_velocity = [0.0, 0.0, 0.0]
        self.raw_euler_angle = [0.0, 0.0, 0.0]
        self.raw_magnetometer = [0.0, 0.0, 0.0]

        self.finish_round = [False for i in range(0, 4)]

    # 处理串行数据
    def handleSerialData(self, raw_data):
        if platform.python_version()[0] == '3':
            self.buffer[self.key] = raw_data
        else:
            self.buffer[self.key] = ord(raw_data)

        self.key += 1
        if self.buffer[0] != 0x55:
            self.key = 0
            return
        else:
            pass

        if self.key < 11:
            return
        else:
            if self.buffer[1] == 0x51:
                if self.buffer[10] == sum(self.buffer[0:10]) & 0xff:
                    cope_data = list(struct.unpack("hhhh", bytearray(self.buffer[2:10])))
                    self.raw_acceleration = [cope_data[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                    self.finish_round[0] = True
                else:
                    pass
            elif self.buffer[1] == 0x52:
                if self.buffer[10] == sum(self.buffer[0:10]) & 0xff:
                    cope_data = list(struct.unpack("hhhh", bytearray(self.buffer[2:10])))
                    self.raw_angular_velocity = [cope_data[i] / 32768.0 * 2000 for i in range(0, 3)]
                    self.finish_round[1] = True
                else:
                    pass
            elif self.buffer[1] == 0x53:
                if self.buffer[10] == sum(self.buffer[0:10]) & 0xff:
                    cope_data = list(struct.unpack("hhhh", bytearray(self.buffer[2:10])))
                    self.raw_euler_angle = [cope_data[i] / 32768.0 * 180 for i in range(0, 3)]
                    self.finish_round[2] = True
                else:
                    pass
            elif self.buffer[1] == 0x54:
                if self.buffer[10] == sum(self.buffer[0:10]) & 0xff:
                    cope_data = list(struct.unpack("hhhh", bytearray(self.buffer[2:10])))
                    self.raw_magnetometer = cope_data
                    self.finish_round[3] = True
                else:
                    pass
            else:
                if (self.buffer[1] & 0x50) == 0x50:
                    print("没有提供该 " + str(self.buffer[1]) + " 的解析")
                else:
                    print("数据错误")
            self.key = 0

            if self.finish_round[0] == self.finish_round[1] == self.finish_round[2] == self.finish_round[3] == True:
                self.acceleration = self.raw_acceleration
                self.angular_velocity = [self.raw_angular_velocity[i] * math.pi / 180 for i in range(0, 3)]
                self.euler_angle = [self.raw_euler_angle[i] * math.pi / 180 for i in range(0, 3)]
                self.magnetometer = self.raw_magnetometer
                time.sleep(0.001)
                return True
            else:
                return False
