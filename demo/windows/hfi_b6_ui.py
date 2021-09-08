#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import platform
import serial.tools.list_ports
import time
import threading
# 查找 ttyUSB* 设备
def find_ttyUSB():
    print("imu default serial port is COM3")
    posts = [port.device for port in serial.tools.list_ports.comports() if 'COM' in port.device]
    print("current computer connect {} ,have {} : {}".format('COM', len(posts), posts))


# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value

        if buff[1] == 0x51 and pub_flag[0]:
            if checkSum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print("0x51 check fail")
            pub_flag[0] = False

        elif buff[1] == 0x52 and pub_flag[1]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 for i in range(0, 3)]

            else:
                print("0x52 check fail")
            pub_flag[1] = False

        elif buff[1] == 0x53 and pub_flag[2]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
            else:
                print("0x53 check fail")
            pub_flag[2] = False

        else:
            print("The data processing class does not provide the resolution of the" + str(buff[2]))
            print("Or data error")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True or pub_flag[2] == True:
            return
        pub_flag[0] = pub_flag[1] = pub_flag[2] = True

        text = '''acceleration(m/s²):
    x-axis:%.2f
    y-axis:%.2f
    z-axis:%.2f

angular velocity(rad/s):
    x-axis:%.2f
    y-axis:%.2f
    z-axis:%.2f

Euler angle(°):
    x-axis:%.2f
    y-axis:%.2f
    z-axis:%.2f

''' % (acceleration[0] * -9.8, acceleration[1] * -9.8, acceleration[2] * -9.8,
       angularVelocity[0], angularVelocity[1], angularVelocity[2],
       angle_degree[0], angle_degree[1], angle_degree[2]
      )
        showText(text)


def startUI():
    window.mainloop()


def showText(text):
    show_text.delete(0.0, tk.END)  # 删除
    show_text.insert(tk.INSERT, text)  # 插入


def loopData(hf_imu):
    while True:
        try:
            time.sleep(0.035)
            buff_count = hf_imu.inWaiting()
        except Exception as e:
            print("exception:" + str(e))
            print("imu lost connection,poor contact or broken wire")
            exit(0)
        else:
            if buff_count > 0:
                buff_data = hf_imu.read(buff_count)
                for i in range(0, buff_count):
                    handleSerialData(buff_data[i])


def threadLoopData(imu_ser):
    # 开启数据解析线程
    t = threading.Thread(target=loopData, args=[imu_ser,])
    # 将当前线程设为子线程t的守护线程,这样一来,当前线程结束时会强制子线程结束
    t.setDaemon(True)
    t.start()


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True, True]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    # ui 库版本判断
    if python_version == '2':
        import Tkinter as tk
    if python_version == '3':
        import tkinter as tk

    # init UI
    window = tk.Tk()
    window.title('handsfree imu')
    window.geometry('640x360')
    show_frame = tk.Frame(window)
    show_frame.config(height=345, width=625)
    show_frame.place(x=5, y=5)
    show_text = tk.Text(show_frame, height=700, bg='white', font=('Arial', 12))
    show_text.place(x=4, y=4)

    find_ttyUSB()
    port = "COM3"
    baudrate = 921600

    try:
        hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if hf_imu.isOpen():
            print("serial open success...")
        else:
            hf_imu.open()
            print("serial open success...")
    except Exception as e:
        print("Exception:"+str(e))
        print("serial open fail")
        exit(0)
    else:
        threadLoopData(hf_imu)
        startUI()
