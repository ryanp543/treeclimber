import math
import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
# from .body_controller import MyController
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
    # Create ROS nodes
    rclpy.init(args=args)
    arms_node = rclpy.create_node('arm_publisher')
    arms_pub = arms_node.create_publisher(String, 'body_control_topic', 10)
    
    # Create my_controller object
    my_controller = MyController()

    # Orient end link servos properly
    my_controller.send_command(6, [1, 0, 0, 0])
    my_controller.send_command(6, [2, 0, 0, 0])
    time.sleep(1)

    # Wind up right arm tendon using position command based on matlab Kinematics.m 
    # Finish with torque. Positive value: windup
    arms_node.get_logger().info("RIGHT ARM TIGHTEN CMD SENT")
    my_controller.send_command(5, [0, 2.9850, 0, 0])
    my_controller.send_command(5, [3, 1, 1, 2.2])
    time.sleep(3)

    # Wind up left arm tendon using position command, then torque command
    # Negative value: windup (-15.1398)
    arms_node.get_logger().info("LEFT ARM TIGHTEN CMD SENT")
    my_controller.send_command(4, [0, -15.1398, 0, 0])
    my_controller.send_command(4, [-5, 3, 1, 2.2])

    time.sleep(12)
    # print("here")
    my_controller.send_command(5, [5, 1, 1, 2.2])

    # Shutdown controller object
    my_controller.shutdown()

    # Shutdown ROS node
    arms_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()