from PyQt5.QtWidgets import QWidget, QLabel, QComboBox, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGroupBox
from PyQt5.QtCore import Qt
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import time

from DM_CAN import *

class MotorWidget(QWidget):
    def __init__(self, motor, controller):
        super().__init__()
        self.motor = motor
        self.controller = controller
        self.data_history = []
        self.max_points = 200

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # 模式选择
        mode_box = QGroupBox("控制模式")
        mode_layout = QHBoxLayout()
        self.mode_select = QComboBox()
        self.mode_select.addItems(["MIT", "POS_VEL", "VEL"])
        self.mode_btn = QPushButton("切换模式")
        self.mode_btn.clicked.connect(self.change_mode)
        mode_layout.addWidget(self.mode_select)
        mode_layout.addWidget(self.mode_btn)
        mode_box.setLayout(mode_layout)



        # 状态显示
        status_box = QGroupBox("电机状态")
        status_layout = QHBoxLayout()
        self.pos_label = QLabel("POS: 0.00")
        self.vel_label = QLabel("VEL: 0.00")
        self.tau_label = QLabel("TAU: 0.00")
        status_layout.addWidget(self.pos_label)
        status_layout.addWidget(self.vel_label)
        status_layout.addWidget(self.tau_label)
        status_box.setLayout(status_layout)

        # 实时图像
        self.graph = PlotWidget()
        self.graph.setYRange(-15, 15)
        self.pos_curve = self.graph.plot(pen='r')
        self.vel_curve = self.graph.plot(pen='g')
        self.tau_curve = self.graph.plot(pen='b')

        # 添加到主布局
        layout.addWidget(mode_box)
        layout.addWidget(status_box)
        layout.addWidget(self.graph)
        self.setLayout(layout)

        # MIT 控制区
        self.control_box = QGroupBox("MIT 控制输入")
        control_layout = QHBoxLayout()

        self.pd_input = QLineEdit("0")
        self.vd_input = QLineEdit("0")
        self.tau_input = QLineEdit("0")
        self.kp_input_mit = QLineEdit("0")
        self.kd_input_mit = QLineEdit("0")

        control_layout.addWidget(QLabel("Pd [-12.5~12.5]")); control_layout.addWidget(self.pd_input)
        control_layout.addWidget(QLabel("Vd [-30~30]")); control_layout.addWidget(self.vd_input)
        control_layout.addWidget(QLabel("Tau [-10~10]")); control_layout.addWidget(self.tau_input)
        control_layout.addWidget(QLabel("Kp [0~500]")); control_layout.addWidget(self.kp_input_mit)
        control_layout.addWidget(QLabel("Kd [0~5]")); control_layout.addWidget(self.kd_input_mit)

        self.send_btn = QPushButton("发送控制帧")
        self.send_btn.clicked.connect(self.send_mit_command)
        control_layout.addWidget(self.send_btn)

        self.control_box.setLayout(control_layout)
        layout.addWidget(self.control_box)


    def change_mode(self):
        mode_str = self.mode_select.currentText()
        mode_map = {"MIT": Control_Type.MIT, "POS_VEL": Control_Type.POS_VEL, "VEL": Control_Type.VEL}
        success = self.controller.switchControlMode(self.motor, mode_map[mode_str])
        if success:
            print(f"切换到 {mode_str} 成功")
            self.update_mode_ui(mode_map[mode_str])

        else:
            print(f"切换到 {mode_str} 失败")

    def update_status(self):
        self.controller.refresh_motor_status(self.motor)
        pos = self.motor.getPosition()
        vel = self.motor.getVelocity()
        tau = self.motor.getTorque()
        self.pos_label.setText(f"POS: {pos:.2f}")
        self.vel_label.setText(f"VEL: {vel:.2f}")
        self.tau_label.setText(f"TAU: {tau:.2f}")

        self.data_history.append((time.time(), pos, vel, tau))
        if len(self.data_history) > self.max_points:
            self.data_history = self.data_history[-self.max_points:]

        times = [t - self.data_history[0][0] for t, _, _, _ in self.data_history]
        poses = [p for _, p, _, _ in self.data_history]
        vels = [v for _, _, v, _ in self.data_history]
        taus = [t for _, _, _, t in self.data_history]

        self.pos_curve.setData(times, poses)
        self.vel_curve.setData(times, vels)
        self.tau_curve.setData(times, taus)
    def send_mit_command(self):
        try:
            pd = float(self.pd_input.text())
            vd = float(self.vd_input.text())
            tau = float(self.tau_input.text())
            kp = float(self.kp_input_mit.text())
            kd = float(self.kd_input_mit.text())

            # 安全范围限制
            if not (-12.5 <= pd <= 12.5):
                raise ValueError("Pd 超出范围 [-12.5, 12.5]")
            if not (-30 <= vd <= 30):
                raise ValueError("Vd 超出范围 [-30, 30]")
            if not (-10 <= tau <= 10):
                raise ValueError("Tau 超出范围 [-10, 10]")
            if not (0 <= kp <= 500):
                raise ValueError("Kp 超出范围 [0, 500]")
            if not (0 <= kd <= 5):
                raise ValueError("Kd 超出范围 [0, 5]")

            self.controller.controlMIT(self.motor, kp, kd, pd, vd, tau)

        except ValueError as ve:
            print(f"[MIT 控制失败]: {ve}")
        except Exception as e:
            print(f"[MIT 控制错误]: {e}")

    def update_mode_ui(self, mode):
        if isinstance(mode, str):
            mode = Control_Type[mode]  # 从字符串变成枚举值
            print("当前模式为：", mode)

        # 控制输入框使能状态
        if mode == Control_Type.MIT:
            self.pd_input.setEnabled(True)
            self.vd_input.setEnabled(True)
            self.tau_input.setEnabled(True)
            self.kp_input_mit.setEnabled(True)
            self.kd_input_mit.setEnabled(True)

        elif mode == Control_Type.POS_VEL:
            self.pd_input.setEnabled(True)
            self.vd_input.setEnabled(True)
            self.tau_input.setEnabled(False)
            self.kp_input_mit.setEnabled(False)
            self.kd_input_mit.setEnabled(False)
            self.tau_input.setText("0")
            self.kp_input_mit.setText("0")
            self.kd_input_mit.setText("0")

        elif mode == Control_Type.VEL:
            self.pd_input.setEnabled(False)
            self.vd_input.setEnabled(True)
            self.tau_input.setEnabled(False)
            self.kp_input_mit.setEnabled(False)
            self.kd_input_mit.setEnabled(False)
            self.pd_input.setText("0")
            self.tau_input.setText("0")
            self.kp_input_mit.setText("0")
            self.kd_input_mit.setText("0")

        else:
            # 默认禁用全部，防止意外模式
            self.pd_input.setEnabled(False)
            self.vd_input.setEnabled(False)
            self.tau_input.setEnabled(False)
            self.kp_input_mit.setEnabled(False)
            self.kd_input_mit.setEnabled(False)
            self.pd_input.setText("0")
            self.vd_input.setText("0")
            self.tau_input.setText("0")
            self.kp_input_mit.setText("0")
            self.kd_input_mit.setText("0")
