"""
达妙 DM-系列 —— MIT 模式恒扭矩示例
-------------------------------------------------
功能 : kp = kd = 0，持续输出 tau = +0.5 N·m
循环 : 200 Hz（可视需要调高/调低）
安全 : 上电后 3 s 再使能；结束请 Ctrl-C
"""

import time, serial
from DM_CAN import MotorControl, Motor, DM_Motor_Type, Control_Type   # 已上传的库

# ------------------- 参数区（根据实际情况修改） -------------------
PORT       = "COM4"         # 串口号 /dev/ttyUSB0 ...
BAUD       = 921600          # 波特率，必须与驱动板一致
SLAVE_ID   = 0x01            # 电机 CAN-ID
MASTER_ID  = 0x11            # PC 端 ID，随便非 0
MOTOR_TYPE = DM_Motor_Type.DM4310  # 根据实际电机型号选
TORQUE_NM  = 0.5             # 恒定输出扭矩
LOOP_HZ    = 200             # 控制循环频率
# -----------------------------------------------------------------

# 1️⃣ 打开串口、创建电机对象
ser   = serial.Serial(PORT, BAUD, timeout=0.5)
mc    = MotorControl(ser)                                           
m0    = Motor(MOTOR_TYPE, SLAVE_ID, MASTER_ID)
mc.addMotor(m0)

# 2️⃣ 切到 MIT 模式（多数固件默认就是 MIT，可省略）
mc.switchControlMode(m0, Control_Type.MIT)                           

# 3️⃣ 使能 & 归零
print("等待 3 s……请确认电机可以自由转动")
time.sleep(3)
mc.enable(m0)                                                       
mc.set_zero_position(m0)

# 4️⃣ 循环发送恒扭矩
kp = 0.0; kd = 0.0; q_des = 0.0; dq_des = 0.0
dt = 1.0 / LOOP_HZ
print(f"开始输出 {TORQUE_NM} N·m（{LOOP_HZ} Hz），Ctrl-C 停止")
try:
    while True:
        mc.controlMIT(m0, kp, kd, q_des, dq_des, TORQUE_NM)       
        time.sleep(dt)
except KeyboardInterrupt:
    print("\n已手动停止，失能电机")
    mc.disable(m0)
