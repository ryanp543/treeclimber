import rclpy
import time
import math
import spidev
from rclpy.node import Node
from std_msgs.msg import String

SPI_BUS = 0
SPI_CS_ARMS = 0

MOTOR_RPM_MAX = 40 # RPM

def main(args=None):
    spi = spidev.SpiDev()

    spi.open(SPI_BUS, SPI_CS_ARMS)
    spi.max_speed_hz = 100000000

    # p = wheel positiion
    # v = wheel velocity
    # r = servo roll
    # l = left tendon (face from back)
    # r = right tendon (face from back)
    # c = clippers
    feature = "p"
    value = 65535 # 16 bit integer unsigned
    feature_bytes = feature.encode()
    value_bytes = value.to_bytes(2, 'big')
    print(feature_bytes)
    print(value_bytes)

    command = feature_bytes + value_bytes
    print(command)
    print(len(command))

    spi.writebytes(command)

    spi.close()
    print("hello")

if __name__ == '__main__':
    main()