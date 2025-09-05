"""
DM4340 10 s 正转 → 10 s 反转 demo
---------------------------------
前置条件：
* USB-CAN/串口适配器已连接，波特率 921 600
* 电机在 ID = 0x01，主机 ID = 0x11
* 固件支持 VEL 模式
"""

import time
from DM_CAN import *          # 达妙官方库
import serial                 # pyserial

# ====== 用户可调参数 ======
PORT        = "COM4"          # 串口号
BAUD        = 921600
SPEED_RAD   = 10.0             # 目标速度 (rad/s)
RUN_SECONDS = 10              # 每个方向持续时间
# ==========================

# 1. 打开串口并注册电机
serial_device = serial.Serial(PORT, BAUD, timeout=0.5)
motor = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
ctrl  = MotorControl(serial_device)
ctrl.addMotor(motor)

try:
    # 2. 切 VEL 模式并使能
    if not ctrl.switchControlMode(motor, Control_Type.VEL):
        raise RuntimeError("切换到 VEL 模式失败")
    time.sleep(0.2)           # 给驱动器留响应时间
    ctrl.enable(motor)

    # 3. 正转 10 s
    print(f"→ Forward  {SPEED_RAD} rad/s  for {RUN_SECONDS} s")
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        ctrl.control_Vel(motor,  SPEED_RAD)
        time.sleep(0.01)

    # 4. 反转 10 s
    print(f"← Reverse {-SPEED_RAD} rad/s  for {RUN_SECONDS} s")
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        ctrl.control_Vel(motor, -SPEED_RAD)
        time.sleep(0.01)

finally:
    # 5. 善后：失能 + 关串口
    print("\nDisabling motor and closing serial…")
    try:
        ctrl.disable(motor)
    except Exception:
        pass
    serial_device.close()
    print("Done.")
