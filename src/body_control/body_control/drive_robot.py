import math
import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from body_controller import MyController
# from .body_controller import MyController

# Message IDs for different commands
# 0x1 = wheel position (value, P, I, D)
# 0x2 = wheel velocity
# 0x3 = turret motor position
# 0x4 = left tendon [torque setpoint, pos setpoint or time(s), kp, ki]
# 0x5 = right tendon
# 0x6 = (facing from back tail counterweight) left = 1, right = 2 servo roll
# 0x7 = clippers
# 0xa = enable motor (i.e. reset encoders)
# 0xb = get drive encoder data

def main(args=None):
    # Create ROS nodes
    rclpy.init(args=args)
    drive_node = rclpy.create_node('drive_publisher')
    drive_pub = drive_node.create_publisher(String, 'body_control_topic', 10)

    # Create my_controller object
    my_controller = MyController()

    # # Drive up
    # drive_node.get_logger().info("VELOCITY COMMAND SENT")
    # my_controller.send_command(2, [5, 10.0, 0, 0.1])
    # time.sleep(2)
    # my_controller.send_command(2, [0, 10.0, 0, 0.1])
    # time.sleep(1)
    
    # # Rotate turret
    # my_controller.send_command(3, [-math.pi/2, 25, 2, 5])
    # time.sleep(5)
    # my_controller.send_command(3, [(math.pi/2), 25, 2, 5])
    # # my_controller.send_command(3, [((math.pi/2) + 0.1), 25, 2, 5])
    # time.sleep(4)

    # Latch
    # my_controller.send_command(7, [0, 0.9, 0, 0])

    # servo
    my_controller.send_command(6, [1, 0, 0, 0])
    my_controller.send_command(6, [2, 0, 0, 0])

    time.sleep(2)

    my_controller.send_command(6, [1, 90, 0, 0])
    my_controller.send_command(6, [2, 90, 0, 0])

    time.sleep(2)

    my_controller.send_command(6, [1, -90, 0, 0])
    my_controller.send_command(6, [2, -90, 0, 0])
    
    time.sleep(2)
    
    my_controller.send_command(6, [1, 0, 0, 0])
    my_controller.send_command(6, [2, 0, 0, 0])

    # Shutdown controller object
    my_controller.shutdown()

    # Shutdown ROS node
    drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()