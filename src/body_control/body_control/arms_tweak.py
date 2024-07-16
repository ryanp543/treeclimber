import math
import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from body_controller import MyController

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

def main(args=None):
    
    # Create my_controller object
    my_controller = MyController()

    # Wind up right arm tendon using position command based on matlab Kinematics.m 
    # Finish with torque. Positive value: windup
    # my_controller.send_command(5, [0, -0.3, 0, 0])
    time.sleep(3)

    # Wind up left arm tendon using position command, then torque command
    # Negative value: windup (-15.1398)
    my_controller.send_command(4, [0, -0.6, 0, 0])

    # Shutdown controller object
    my_controller.shutdown()

if __name__ == "__main__":
    main()