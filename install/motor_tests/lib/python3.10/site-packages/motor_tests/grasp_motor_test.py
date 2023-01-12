import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import String

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.bricklet_io4_v2 import BrickletIO4V2
from tinkerforge.bricklet_industrial_counter import BrickletIndustrialCounter

HOST = "localhost"
PORT = 4223
UID_servo = "SNw"
UID_io2 = "QAuAz"
UID_master = "646H2t"
UID_counter = "GfZ"

FG_COUNTER_1 = 0
SIGNALS_PER_REV = 6
RADIANS_PER_SIGNAL = 2 * math.pi / SIGNALS_PER_REV
GEAR_REDUCTION = 296
BLDC_NOLOADRPM = 8000

# def cb_input_value(channel, changed, value):
#     global FG_COUNTER_1, RADIANS_PER_SIGNAL, GEAR_REDUCTION
#     if channel == 2 and value == False and changed == True:
#         FG_COUNTER_1 = FG_COUNTER_1 + 1
#         rad = FG_COUNTER_1 * RADIANS_PER_SIGNAL / GEAR_REDUCTION
#         print(rad)

    # print("Channel: " + str(channel))
    # print("Changed: " + str(changed))
    # print("Value: " + str(value))


def main(args=None):
    global BLDC_NOLOADRPM

    ipcon = IPConnection()
    io = BrickletIO4V2(UID_io2, ipcon)
    servo = BrickletServoV2(UID_servo, ipcon)
    counter = BrickletIndustrialCounter(UID_counter, ipcon)
    ipcon.connect(HOST, PORT)

    # Set settings of io bricklet
    io.set_configuration(0, "o", False)
    # io.register_callback(io.CALLBACK_INPUT_VALUE, cb_input_value)
    # io.set_input_value_callback_configuration(2, 4, True)
    # io.set_edge_count_configuration(2, 1, 1)

    # Set settings for counter bricklet
    counter.set_counter_configuration(0, 1, 0, 0, 0)
    counter.set_counter_active(0, True)
    counter.set_counter(0, 0)

    print(counter.get_all_counter_active())

    # Set settings of servo bricklet, speed is ratio of pulse width to period
    servo.set_period(0, 50)
    servo.set_degree(0, 0, BLDC_NOLOADRPM)
    servo.set_pulse_width(0, 50, 50)

    # Send PWM signal
    input("Press Enter to start")
    servo.set_position(0, BLDC_NOLOADRPM)
    servo.set_enable(0, True)
    print("Should have started")

    time.sleep(1)
    print(counter.get_counter(0))

    # stop the motion
    time.sleep(2.22222)
    servo.set_position(0, 0)
    servo.set_enable(0, False)
    print("Should have stopped")
    print(counter.get_counter(0))

    ipcon.disconnect()

if __name__ == '__main__':
    main()