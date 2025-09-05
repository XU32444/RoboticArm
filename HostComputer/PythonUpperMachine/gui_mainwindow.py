# gui_mainwindow.py
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QHBoxLayout, QComboBox
from PyQt5.QtCore import QTimer, Qt
from motor_widget import MotorWidget
from DM_CAN import *
import serial

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("达妙电机上位机 GUI")
        self.resize(800, 600)

        self.central = QWidget()
        self.layout = QVBoxLayout()

        # 串口输入和连接按钮
        self.port_input = QLineEdit("COM4")
        self.connect_btn = QPushButton("连接串口")
        self.connect_btn.clicked.connect(self.try_connect_serial)
        self.layout.addWidget(QLabel("串口号:"))
        self.layout.addWidget(self.port_input)
        self.layout.addWidget(self.connect_btn)

        # 电机连接状态栏
        self.status_layout = QHBoxLayout()
        self.motor_status_labels = []
        for i in range(6):
            label = QLabel(f"J{i+1} 未连接")
            label.setStyleSheet("color: red")
            self.motor_status_labels.append(label)
            self.status_layout.addWidget(label)
        self.layout.addLayout(self.status_layout)

        # 初始化后隐藏以下控件，连接成功后再显示
        self.motor_select = QComboBox()
        self.motor_select.currentIndexChanged.connect(self.change_motor_selection)
        self.enable_btn = QPushButton("使能电机")
        self.disable_btn = QPushButton("失能电机")
        self.enable_btn.clicked.connect(self.enable_selected_motor)
        self.disable_btn.clicked.connect(self.disable_selected_motor)
        self.layout.addWidget(QLabel("选择电机 (J1-J6):"))
        self.layout.addWidget(self.motor_select)
        self.layout.addWidget(self.enable_btn)
        self.layout.addWidget(self.disable_btn)

        self.motor_select.hide()
        self.enable_btn.hide()
        self.disable_btn.hide()
        self.layout.itemAt(self.layout.count() - 4).widget().hide()  # QLabel("选择电机")
        self.motor_widget = None

        self.central.setLayout(self.layout)
        self.setCentralWidget(self.central)

    def try_connect_serial(self):
        port_name = self.port_input.text()
        try:
            self.serial_device = serial.Serial(port_name, 921600, timeout=0.5)
            self.controller = MotorControl(self.serial_device)

            self.motors = []
            for i in range(6):
                slave_id = 0x01 + i
                master_id = 0x11 + i
                motor = Motor(DM_Motor_Type.DM4310, slave_id, master_id)
                self.controller.addMotor(motor)
                self.controller.enable(motor)
                self.controller.refresh_motor_status(motor)
                pos = motor.getPosition()
                if abs(pos) > 1e-4:
                    self.motor_status_labels[i].setText(f"J{i+1} 已连接")
                    self.motor_status_labels[i].setStyleSheet("color: green")
                    self.motors.append(motor)
                    self.motor_select.addItem(f"J{i+1}")
                else:
                    self.motor_status_labels[i].setText(f"J{i+1} 未连接")
                    self.motor_status_labels[i].setStyleSheet("color: red")

            if not self.motor_widget and self.motors:
                selected_index = self.motor_select.currentIndex()
                selected_motor = self.motors[selected_index] if selected_index >= 0 else self.motors[0]
                self.motor_widget = MotorWidget(selected_motor, self.controller)
                self.layout.addWidget(self.motor_widget)

            self.timer = QTimer()
            self.timer.timeout.connect(self.update_selected_motor_status)
            self.timer.start(100)

            self.motor_select.show()
            self.enable_btn.show()
            self.disable_btn.show()
            self.layout.itemAt(self.layout.count() - 4).widget().show()
            QMessageBox.information(self, "连接成功", f"已连接到 {port_name}")

        except Exception as e:
            QMessageBox.critical(self, "连接失败", f"串口打开失败: {str(e)}")

    def change_motor_selection(self, index):
        if self.motor_widget and 0 <= index < len(self.motors):
            selected_motor = self.motors[index]
            self.motor_widget.set_motor(selected_motor)
            current_mode = self.controller.read_motor_param(selected_motor, DM_variable.CTRL_MODE)
            self.motor_widget.update_mode_ui(current_mode)

    def update_selected_motor_status(self):
        index = self.motor_select.currentIndex()
        if 0 <= index < len(self.motors):
            self.motor_widget.send_mit_command()
            self.motor_widget.update_status()

    def enable_selected_motor(self):
        index = self.motor_select.currentIndex()
        if 0 <= index < len(self.motors):
            self.controller.enable(self.motors[index])

    def disable_selected_motor(self):
        index = self.motor_select.currentIndex()
        if 0 <= index < len(self.motors):
            self.controller.disable(self.motors[index])
