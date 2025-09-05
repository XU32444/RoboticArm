import sys
import time
import math
import serial
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QMessageBox, QGroupBox
)
from DM_CAN import *

PORT = "COM4"
BAUD = 921600

class MotorControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("电机控制界面")
        self.motor_control = None
        self.motor_list = []

        self.init_serial_and_scan()
        self.initUI()

    def init_serial_and_scan(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.5)
            self.motor_control = MotorControl(self.ser)
            print("串口打开成功")

            # 尝试扫描 ID 从 0x01 到 0x10
            for sid in range(1, 17):
                try:
                    motor = Motor(DM_Motor_Type.DM4310, sid, sid + 0x10)
                    self.motor_control.addMotor(motor)
                    self.motor_control.refresh_motor_status(motor)
                    _ = motor.getPosition()  # 如果能获取，说明有效
                    self.motor_list.append(motor)
                    print(f"检测到电机 ID: {sid}")
                except:
                    continue
        except Exception as e:
            QMessageBox.critical(self, "错误", f"无法打开串口：{e}")
            sys.exit(1)

    def initUI(self):
        layout = QVBoxLayout()

        if not self.motor_list:
            layout.addWidget(QLabel("未检测到电机"))
        else:
            for motor in self.motor_list:
                box = QGroupBox(f"电机 ID: {motor.SlaveID}")
                box_layout = QHBoxLayout()

                # 设为零点按钮
                zero_btn = QPushButton("设为0点")
                zero_btn.clicked.connect(lambda _, m=motor: self.set_zero(m))

                # 输入角度和发送按钮
                angle_input = QLineEdit()
                angle_input.setPlaceholderText("输入角度(°)")
                move_btn = QPushButton("转到角度")
                move_btn.clicked.connect(lambda _, m=motor, a=angle_input: self.move_motor(m, a))

                box_layout.addWidget(zero_btn)
                box_layout.addWidget(angle_input)
                box_layout.addWidget(move_btn)
                box.setLayout(box_layout)

                layout.addWidget(box)

        self.setLayout(layout)

    def set_zero(self, motor):
        try:
            self.motor_control.set_zero_position(motor)
            QMessageBox.information(self, "成功", f"电机 {motor.SlaveID} 零点设置成功")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"设置零点失败: {e}")

    def move_motor(self, motor, angle_input):
        try:
            angle_deg = float(angle_input.text())
            angle_rad = math.radians(angle_deg)
            self.motor_control.control_Pos_Vel(motor, angle_rad, 2.0)
        except ValueError:
            QMessageBox.warning(self, "输入错误", "请输入有效的角度（数字）")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"控制失败: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorControlUI()
    window.show()
    sys.exit(app.exec_())
