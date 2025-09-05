import time, signal, serial
from DM_CAN import MotorControl, Motor, DM_Motor_Type, Control_Type

# ---------- 参数 ----------
PORT       = "COM4"
BAUD       = 921600
SLAVE_ID   = 0x04
MASTER_ID  = 0x14
MOTOR_TYPE = DM_Motor_Type.DM4310
KSPRING    = 1.8     # N·m / rad
B          = 0.021    # N·m·s / rad
LOOP_HZ    = 1000
TAU_MAX    = 3.0     # 限制最大力矩
# --------------------------

# 初始化串口和电机
ser = serial.Serial(PORT, BAUD, timeout=0.5)
ctrl = MotorControl(ser)
motor = Motor(MOTOR_TYPE, SLAVE_ID, MASTER_ID)
ctrl.addMotor(motor)
ctrl.enable_old(motor, Control_Type.MIT)

dt = 1.0 / LOOP_HZ
running = True

def stop_handler(signum, frame):
    global running
    running = False
    print("Stopping...")

signal.signal(signal.SIGINT, stop_handler)

print("Spring mode running. Press Ctrl+C to stop.")

while running:
    # 刷新状态，获取当前位置和速度
    ctrl.refresh_motor_status(motor)
    pos = motor.getPosition()
    vel = motor.getVelocity()

    # 计算弹簧+阻尼力矩
    tau = -KSPRING * pos - B * vel
    tau = max(-TAU_MAX, min(TAU_MAX, tau))

    # 发出MIT控制指令（力矩前馈）
    ctrl.controlMIT(motor, 0, 0, 0, 0, tau)

    time.sleep(dt)

# 停止电机
ctrl.controlMIT(motor, 0, 0, 0, 0, 0)
ctrl.disable(motor)
print("Exited gracefully.")
