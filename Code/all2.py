import time
from DM_CAN import *
import serial

# ====== 用户可调参数 ======
PORT        = "COM4"
BAUD        = 921600
SPEED_RAD   = 5.0
RUN_SECONDS = 30
# ==========================

# 1. 打开串口 & 控制器
serial_device = serial.Serial(PORT, BAUD, timeout=0.5)
ctrl = MotorControl(serial_device)

# 2. 电机对象列表（混合 4340 和 4310）
motors = [
    Motor(DM_Motor_Type.DM4340, 0x01, 0x11),
    Motor(DM_Motor_Type.DM4340, 0x02, 0x12),
    Motor(DM_Motor_Type.DM4340, 0x03, 0x13),
    Motor(DM_Motor_Type.DM4310, 0x04, 0x14),
    Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
    Motor(DM_Motor_Type.DM4310, 0x06, 0x16),
]

# 3. 添加电机
for m in motors:
    ctrl.addMotor(m)

try:
    # 4. 切换模式并使能
    for m in motors:
        if not ctrl.switchControlMode(m, Control_Type.VEL):
            raise RuntimeError(f"电机 {hex(m.getSlaveID())} 切 VEL 模式失败")
        time.sleep(0.1)
        ctrl.enable(m)
        time.sleep(0.1)

    # 5. 正转
    print(f"→ Forward {SPEED_RAD} rad/s")
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        for m in motors:
            ctrl.control_Vel(m, SPEED_RAD)
        time.sleep(0.01)

    # 6. 反转
    print(f"← Reverse {-SPEED_RAD} rad/s")
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        for m in motors:
            ctrl.control_Vel(m, -SPEED_RAD)
        time.sleep(0.01)

finally:
    # 7. 善后处理
    print("Disabling motors and closing serial…")
    for m in motors:
        try:
            ctrl.control_Vel(m, 0)
            ctrl.disable(m)
        except Exception:
            pass
    serial_device.close()
    print("Done.")
