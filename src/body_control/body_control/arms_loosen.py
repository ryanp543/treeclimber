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

    # Undo latch
    my_controller.send_command(7, [0, 0.9, 0, 0])
    time.sleep(40)

    # Loosen left arm
    arms_node.get_logger().info("LEFT ARM LOOSEN CMD SENT")
    my_controller.send_command(4, [0, 4.3, 0, 0])

    # Loosen right arm, temporarily tighten midway to unlatch
    arms_node.get_logger().info("RIGHT ARM LOOSEN CMD SENT")
    my_controller.send_command(5, [0, -2, 0, 0])
    my_controller.send_command(5, [0, 2, 0, 0])
    
    my_controller.send_command(5, [0, -3.7, 0, 0])

    # Shutdown controller object
    my_controller.shutdown()

    # Shutdown ROS node
    arms_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()