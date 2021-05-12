# -*- coding:utf-8 -*-
from datetime import datetime

from HandsFreeImu import *
import Tkinter as tk


# 图形界面类
class MyUI:
    def __init__(self):
        # 创建显示传感器数据的窗口
        self.window = tk.Tk()
        self.window.title('HFI-A9')
        self.window.geometry('640x360')

        self.frameBoy = tk.Frame(self.window)
        self.frameBoy.config(height=345, width=625)
        self.frameBoy.place(x=5, y=5)

        self.textBox = tk.Text(self.frameBoy, height=700, bg='white', font=('Arial', 12))
        self.textBox.place(x=4, y=4)

        self.working = False

    # 开启UI
    def start(self):
        t = threading.Thread(target=self.updateThread)
        self.working = True
        t.setDaemon(True)
        t.start()
        # 开启窗口主循环
        self.window.mainloop()

    # 显示文本
    def showText(self, text):
        self.textBox.delete(0.0, tk.END)
        self.textBox.insert(tk.INSERT, text)

    def updateThread(self):
        t0 = time.time()
        while self.working:
            if [imu.finish_round[i] for i in range(0, len(imu.finish_round))] == [True for i in
                                                                                  range(0, len(imu.finish_round))]:
                t0 = time.time()

                def output():
                    text = '系统时间：' + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\r\n"
                    text += '相对时间：' + "%.3f" % (time.time() - t0) + "\r\n\r\n"

                    text += 'x轴加速度：' + "%.2f m/s²" % imu.acceleration[0] + "\r\n"
                    text += 'y轴加速度：' + "%.2f m/s²" % imu.acceleration[1] + "\r\n"
                    text += 'z轴加速度：' + "%.2f m/s²" % imu.acceleration[2] + "\r\n\r\n"

                    text += 'x轴角速度：' + "%.2f rad/s" % imu.angular_velocity[0] + "\r\n"
                    text += 'y轴角速度：' + "%.2f rad/s" % imu.angular_velocity[1] + "\r\n"
                    text += 'z轴角速度：' + "%.2f rad/s" % imu.angular_velocity[2] + "\r\n\r\n"

                    text += 'x轴角度：  ' + "%.2f rad" % imu.euler_angle[0] + "\r\n"
                    text += 'y轴角度：  ' + "%.2f rad" % imu.euler_angle[1] + "\r\n"
                    text += 'z轴角度：  ' + "%.2f rad" % imu.euler_angle[2] + "\r\n\r\n"

                    text += 'x轴磁场： ' + "%.2f mG" % imu.magnetometer[0] + "\r\n"
                    text += 'y轴磁场： ' + "%.2f mG" % imu.magnetometer[1] + "\r\n"
                    text += 'z轴磁场： ' + "%.2f mG" % imu.magnetometer[2] + "\r\n\r\n"

                    self.showText(text)

                output()

                imu.finish_round = [False for i in range(0, len(imu.finish_round))]


# 主线程
if __name__ == '__main__':
    imu = ImuA()
    imu_ser = SerialTransport()
    imu_ser.openSerial('/dev/ttyUSB0', 921600, 1)
    imu_ser.start(imu.handleSerialData)

    ui = MyUI()
    ui.start()

