"""
MIT, 纯力矩外环版 —— 可随手调软硬
-------------------------------------------------
滚轮 / ↑↓ 改刚度；←→ 改阻尼
"""
import time, serial
from pynput import keyboard, mouse
from DM_CAN import MotorControl, Motor, DM_Motor_Type, Control_Type

# ---参数---
PORT, BAUD = "COM4", 921600
ID, MASTER = 0x01, 0x11
MOTOR_TYPE = DM_Motor_Type.DM4310
KSPRING, B = 0.20, 0.02      # 初始 (N·m/rad), (N·m·s/rad)
TAU_LIM     = 1.0            # 人手能轻松扭动的扭矩上限
LOOP_HZ     = 500
STEP_K, STEP_B = 0.05, 0.01  # 每次微调步长
# --------------

# ➜ 初始化
ser = serial.Serial(PORT, BAUD, timeout=0.5)
mc  = MotorControl(ser)
m0  = Motor(MOTOR_TYPE, ID, MASTER)
mc.addMotor(m0)
mc.switchControlMode(m0, Control_Type.MIT)
time.sleep(2); mc.enable(m0); mc.set_zero_position(m0)

# ➜ 键鼠实时调参
def on_kbd(key):
    global KSPRING, B
    if key == keyboard.Key.up:   KSPRING = max(0, KSPRING + STEP_K)
    if key == keyboard.Key.down: KSPRING = max(0, KSPRING - STEP_K)
    if key == keyboard.Key.right: B = max(0, B + STEP_B)
    if key == keyboard.Key.left:  B = max(0, B - STEP_B)
    print(f"kp={KSPRING:.2f}, kd={B:.3f}")

def on_scroll(x,y,dx,dy):
    on_kbd(keyboard.Key.up if dy>0 else keyboard.Key.down)

keyboard.Listener(on_press=on_kbd).start()
mouse.Listener(on_scroll=on_scroll).start()

# ➜ 主循环
dt = 1/LOOP_HZ
print("开始：←→ 调 kd，↑↓ 调 kp，Ctrl-C 退出")
try:
    while True:
        mc.refresh_motor_status(m0)
        theta = m0.getPosition()     # rad
        omega = m0.getVelocity()     # rad/s
        tau_cmd = -KSPRING*theta - B*omega
        tau_cmd = max(min(tau_cmd, TAU_LIM), -TAU_LIM)  # 软限幅
        mc.controlMIT(m0, 0, 0, 0, 0, tau_cmd)
        time.sleep(dt)
except KeyboardInterrupt:
    mc.disable(m0)
    print("\n已关闭")
