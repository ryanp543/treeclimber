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

    # Drive up and rotate servos up
    print("Drive up 1")
    my_controller.send_command(6, [1, 0, 0, 0])
    my_controller.send_command(6, [2, 0, 0, 0])
    time.sleep(2)
    my_controller.send_command(1, [8, 40.0, 1.0, 0.1])
    time.sleep(3)
    
    # Rotate turret and servos
    print("Rotate right 1")
    my_controller.send_command(3, [-math.pi/2, 25, 2, 5])
    my_controller.send_command(6, [1, -90, 0, 0])
    my_controller.send_command(6, [2, -90, 0, 0])
    time.sleep(5)

    # Drive around column
    my_controller.send_command(1, [5, 40.0, 1.0, 0.1])
    time.sleep(3)

    # Rotate back
    my_controller.send_command(3, [(math.pi/2), 25, 2, 5])
    my_controller.send_command(6, [1, 0, 0, 0])
    my_controller.send_command(6, [2, 0, 0, 0])
    time.sleep(5)

    # Drive up
    print("Drive up 2")
    my_controller.send_command(1, [10, 40.0, 1.0, 0.1])
    time.sleep(3)

    # Rotate turret and servos
    print("Rotate left 2")
    my_controller.send_command(3, [math.pi/2, 25, 2, 5])
    my_controller.send_command(6, [1, 90, 0, 0])
    my_controller.send_command(6, [2, 90, 0, 0])
    time.sleep(5)

    # Drive around column
    my_controller.send_command(1, [5, 40.0, 1.0, 0.1])
    time.sleep(3)

    # Rotate back
    my_controller.send_command(3, [-(math.pi/2), 25, 2, 5])
    my_controller.send_command(6, [1, 0, 0, 0])
    my_controller.send_command(6, [2, 0, 0, 0])
    time.sleep(5)

    # Drive down and rotate servo
    print("Drive down 3")
    my_controller.send_command(6, [1, -180, 0, 0])
    my_controller.send_command(6, [2, -180, 0, 0])
    time.sleep(1)
    my_controller.send_command(1, [-16, 40.0, 1.0, 0.1])
    time.sleep(3)

    # Shutdown controller object
    print("Done, shutting down now")
    my_controller.shutdown()

    # Shutdown ROS node
    drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()