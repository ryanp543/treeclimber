import body_controller
import math
import time

# Message IDs for different commands
# 0x1 = wheel position (value, P, I, D)
# 0x2 = wheel velocity
# 0x3 = turret motor position
# 0x4 = left tendon [torque setpoint, pos setpoint or time(s), kp, ki]
# 0x5 = right tendon
# 0x6 = (facing from back tail counterweight) left = 2, right = 3 servo roll
# 0x7 = clippers
# 0xa = enable motor (i.e. reset encoders)
# 0xb = get drive encoder data

def main():
    # Create my_controller object
    my_controller = body_controller.MyController()

    # Wind up right arm tendon using position command based on matlab Kinematics.m 
    # Finish with torque 2.9850
    # my_controller.send_command(5, [0, 2.9850, 0, 0])
    # my_controller.send_command(5, [2, 3, 1, 2.2])
    time.sleep(1)
    # my_controller.send_command(5, [0, -3.5, 0, 0])

    # my_controller.send_command(3, [-math.pi/2, 25, 2, 5])

    # Shutdown controller object
    my_controller.shutdown()

if __name__ == "__main__":
    main()