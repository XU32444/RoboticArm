import sys, time, threading
import serial
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

from DM_CAN import *

# ==== 初始化电机 ====
PORT = "COM4"
BAUD = 921600
serial_device = serial.Serial(PORT, BAUD, timeout=0.5)
J6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)
MotorControl1 = MotorControl(serial_device)
MotorControl1.addMotor(J6)
MotorControl1.switchControlMode(J6, Control_Type.VEL)
time.sleep(0.05)
MotorControl1.enable(J6)

# ==== GUI 类 ====
class MotorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("达妙 J6 电机速度监控")
        self.resize(800, 600)

        # 状态数据
        self.vel_data = []
        self.time_data = []
        self.start_time = time.time()

        # 绘图相关
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)

        # 按钮
        self.btn_start = QPushButton("启动电机")
        self.btn_stop = QPushButton("停止电机")
        self.btn_start.clicked.connect(self.start_motor)
        self.btn_stop.clicked.connect(self.stop_motor)

        # 布局
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.btn_start)
        layout.addWidget(self.btn_stop)
        self.setLayout(layout)

        # 状态更新定时器（用于读取速度并画图）
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # 每 100 ms 更新一次图

        # 电机控制线程
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def control_loop(self):
        while self.running:
            if hasattr(self, 'vel_cmd') and self.vel_cmd != 0.0:
                MotorControl1.control_Vel(J6, self.vel_cmd)
            time.sleep(0.01)

    def update_plot(self):
        MotorControl1.refresh_motor_status(J6)
        vel = J6.getVelocity()
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.vel_data.append(vel)
        self.time_data = self.time_data[-200:]
        self.vel_data = self.vel_data[-200:]
        self.ax.clear()
        self.ax.plot(self.time_data, self.vel_data, label='J6 Velocity (rad/s)')
        self.ax.set_ylabel("速度 rad/s")
        self.ax.set_xlabel("时间 s")
        self.ax.legend()
        self.canvas.draw()

    def start_motor(self):
        self.vel_cmd = 10.0  # 设定速度
        print("启动电机...")

    def stop_motor(self):
        self.vel_cmd = 0.0
        MotorControl1.control_Vel(J6, 0.0)
        print("停止电机...")

    def closeEvent(self, event):
        self.running = False
        MotorControl1.control_Vel(J6, 0.0)
        MotorControl1.disable(J6)
        serial_device.close()
        event.accept()

# ==== 启动应用 ====
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = MotorGUI()
    gui.show()
    sys.exit(app.exec_())
