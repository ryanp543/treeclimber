import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    msg = "hello brothers"
    print(msg)

if __name__ == '__main__':
    main()