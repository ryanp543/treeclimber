import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_dc import BrickDC

HOST = "localhost"
PORT = 4223
UID_dc = "6RPLkK"

if __name__ == '__main__':
    msg = "hello everyone"
    print(msg)

    ipcon = IPConnection()
    dc = BrickDC(UID_dc, ipcon)
    ipcon.connect(HOST, PORT)

    print(dc.get_current_consumption())

    ipcon.disconnect()