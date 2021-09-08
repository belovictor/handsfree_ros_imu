#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import platform
import serial.tools.list_ports


# 查找 ttyUSB* 设备
def find_ttyUSB():
    print("imu default serial port is COM3")
    posts = [port.device for port in serial.tools.list_ports.comports() if 'COM' in port.device]
    print("current computer connect {} ，have {} : {}".format('COM', len(posts), posts))


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


# 16 进制转 ieee 浮点数
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


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0xaa:
        key = 0
        return
    if key < 3:
        return
    if buff[1] != 0x55:
        key = 0
        return
    if key < buff[2] + 5:  # 根据数据长度位的判断, 来获取对应长度数据
        return

    else:
        data_buff = list(buff.values())  # 获取字典所以 value

        if buff[2] == 0x2c and pub_flag[0]:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                data = hex_to_ieee(data_buff[7:47])
                angularVelocity = data[1:4]
                acceleration = data[4:7]
                magnetometer = data[7:10]
            else:
                print("check fail")
            pub_flag[0] = False
        elif buff[2] == 0x14 and pub_flag[1]:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                angle_degree = data[1:4]
            else:
                print("check success")
            pub_flag[1] = False
        else:
            print("The data processing class does not provide the resolution of the" + str(buff[2]))
            print("Or data error")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True:
            return
        pub_flag[0] = pub_flag[1] = True

        text = '''
acceleration(m/s²)：
    x-axis：%.2f
    y-axis：%.2f
    z-axis：%.2f

angular velocity(rad/s)：
    x-axis：%.2f
    y-axis：%.2f
    z-axis：%.2f

Euler angle(°)：
    x-axis：%.2f
    y-axis：%.2f
    z-axis：%.2f

magnetic field：
    x-axis：%.2f
    y-axis：%.2f
    z-axis：%.2f

''' % (acceleration[0] * -9.8, acceleration[1] * -9.8, acceleration[2] * -9.8,
       angularVelocity[0], angularVelocity[1], angularVelocity[2],
       angle_degree[0], angle_degree[1], angle_degree[2],
       magnetometer[0], magnetometer[1], magnetometer[2]
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
            buff_count = hf_imu.inWaiting()
        except Exception as e:
            print("exception:" + str(e))
            print("imu 失去连接，接触不良，或断线")
            exit(0)
        else:
            if buff_count > 0:
                buff_data = hf_imu.read(buff_count)
                for i in range(0, buff_count):
                    handleSerialData(buff_data[i])


def threadLoopData(imu_ser):
    import threading
    # 开启数据解析线程
    t = threading.Thread(target=loopData, args=[imu_ser,])
    # 将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
    t.setDaemon(True)
    t.start()


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True]


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
            print("serial open fail...")
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
