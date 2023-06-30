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
    print("SPI port opened")
    spi.max_speed_hz = 10000000
    # spi.mode = 0b00

    # p = wheel positiion
    # v = wheel velocity
    # r = servo roll
    # l = left tendon (face from back)
    # r = right tendon (face from back)
    # c = clippers
    feature = "t" # 8 bit string character
    value = 65535 # 16 bit integer unsigned
    feature_bytes = feature.encode()
    value_bytes = value.to_bytes(2, 'big')

    # Command 3 bytes, 24 bits
    command = feature_bytes # + value_bytes
    print(command)
    print(len(command)) 

    # spi.writebytes([0xFF])
    # spi.writebytes([0xFF])

    try:
        spi.writebytes(command)
        # spi.xfer(command)
        print("Successfully written")
    except IOError as e:
        print("SPI communication error:", str(e))
    # finally:
    #     spi.close()
    print("hello")

if __name__ == '__main__':
    main()