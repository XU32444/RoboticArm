import time
from DM_CAN import *          
import serial                 


# ====== 用户可调参数 ======
PORT        = "COM4"          # 串口号
BAUD        = 921600
SPEED_RAD   = 3             # 目标速度 (rad/s)
RUN_SECONDS = 10              # 每个方向持续时间
# ==========================

serial_device = serial.Serial(PORT, BAUD, timeout=0.5)

J6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)
# J6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)


MotorControl1=MotorControl(serial_device)

MotorControl1.addMotor(J6)


MotorControl1.enable(J6)

MotorControl1.switchControlMode(J6, Control_Type.VEL)

time.sleep(0.05)  # Give some delay

MotorControl1.enable(J6)



for _ in range(1000):  # 10 秒 @ 100 Hz
    MotorControl1.control_Vel(J6, 10.0)  # 匀速 5 rad/s
    time.sleep(0.01)

# 8. 停止电机
MotorControl1.control_Vel(J6, 0.0)
MotorControl1.disable(J6)
serial_device.close()

