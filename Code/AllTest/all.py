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

J1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
J2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
J3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
J4 = Motor(DM_Motor_Type.DM4310, 0x04, 0x14)
J5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
J6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)


MotorControl1=MotorControl(serial_device)

MotorControl1.addMotor(J1)
MotorControl1.addMotor(J2)
MotorControl1.addMotor(J3)
MotorControl1.addMotor(J4)
MotorControl1.addMotor(J5)
MotorControl1.addMotor(J6)

MotorControl1.enable(J1)
MotorControl1.enable(J2)
MotorControl1.enable(J3)
MotorControl1.enable(J4)
MotorControl1.enable(J5)
MotorControl1.enable(J6)

# 所有电机的列表
motors = [J1, J2, J3, J4, J5, J6]


for m in [J1, J2, J3, J4, J5, J6]:
    MotorControl1.switchControlMode(m, Control_Type.VEL)
    time.sleep(0.05)  # Give some delay
    MotorControl1.enable(m)


print("Start forward rotation...")
start_time = time.time()
while time.time() - start_time < RUN_SECONDS:
    for m in motors:
        MotorControl1.control_Vel(m, SPEED_RAD)
    time.sleep(0.01)

print("Start reverse rotation...")
start_time = time.time()
while time.time() - start_time < RUN_SECONDS:
    for m in motors:
        MotorControl1.control_Vel(m, -SPEED_RAD)
    time.sleep(0.01)

print("Stopping all motors...")
for m in motors:
    MotorControl1.control_Vel(m, 0)
    MotorControl1.disable(m)

print("Done.")