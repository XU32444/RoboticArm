"""
Single motor functional test for 达妙 DM4340
------------------------------------------
* Switches motor to POS_VEL mode
* Enables motor
* Drives the motor with a smooth ±6 rad sine‑sweep for 10 s
* Prints live position / velocity / torque every second
* Safely disables the motor and closes the serial port on exit

Usage:
    python DM_SingleMotor_Test.py
Make sure the COM port and CAN IDs match your hardware.


python ./Code/1.py

"""

import math
import time
import signal
from DM_CAN import *
import serial

# ====== User‑adjustable settings ======
PORT = 'COM4'          # Serial port name
SLAVE_ID = 0x01        # Motor CAN ID
MASTER_ID = 0x11       # Host ID (do NOT set to 0x00)
SWEEP_AMPLITUDE = 6.0  # rad, must be ≤ PMAX (12.5 rad for DM4340)
SWEEP_FREQ = 0.2       # Hz
TEST_DURATION = 10.0   # seconds
VELOCITY = 5            
# =====================================

# Initialize serial and motor control
serial_device = serial.Serial(PORT, 921600, timeout=0.5)
motor = Motor(DM_Motor_Type.DM4340, SLAVE_ID, MASTER_ID)
ctrl = MotorControl(serial_device)
ctrl.addMotor(motor)

# Optional: change to position‑velocity mode so we can specify desired position
print("Switching control mode to POS_VEL…")
if not ctrl.switchControlMode(motor, Control_Type.POS_VEL):
    print("⚠️  Failed to switch control mode; continuing anyway.")

# Enable the motor after a short pause
time.sleep(0.5)
print("Enabling motor…")
ctrl.enable(motor)

# Graceful shutdown handler ----------------------------------------------------
running = True
def _sigint_handler(signum, frame):
    global running
    running = False
signal.signal(signal.SIGINT, _sigint_handler)
# ---------------------------------------------------------------------------

print(f"Running sine‑sweep for {TEST_DURATION} s — press Ctrl‑C to abort")
start = time.time()
last_print = start

try:
    while running and (time.time() - start) < TEST_DURATION:
        now = time.time()
        t = now - start
        position_cmd = SWEEP_AMPLITUDE * math.sin(2 * math.pi * SWEEP_FREQ * t)
        velocity_cmd = VELOCITY  # we let the controller track position; vel=0
        ctrl.control_Pos_Vel(motor, position_cmd, velocity_cmd)

        # Print feedback roughly once per second
        if now - last_print >= 1.0:
            ctrl.refresh_motor_status(motor)
            print(f"[{t:5.2f}s]  pos={motor.getPosition():6.2f}  "
                  f"vel={motor.getVelocity():6.2f}  tau={motor.getTorque():6.2f}")
            last_print = now

        time.sleep(0.01)  # 10 ms loop
finally:
    print("\nDisabling motor and closing serial port…")
    try:
        ctrl.disable(motor)
    except Exception:
        pass
    serial_device.close()
    print("Done.")
