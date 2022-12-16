import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.bricklet_io4_v2 import BrickletIO4V2

HOST = "localhost"
PORT = 4223
UID_servo = "SNw"
UID_io2 = "QAuAz"
UID_master = "646H2t"

def cb_input_value(channel, changed, value):
    print("Channel: " + str(channel))
    print("Changed: " + str(changed))
    print("Value: " + str(value))


def main(args=None):
    msg = "hello people"
    print(msg)

    ipcon = IPConnection()
    io = BrickletIO4V2(UID_io2, ipcon)
    servo = BrickletServoV2(UID_servo, ipcon)
    ipcon.connect(HOST, PORT)

    # Set settings of io bricklet
    io.set_configuration(0, "o", False)
    io.register_callback(io.CALLBACK_INPUT_VALUE, cb_input_value)
    io.set_input_value_callback_configuration(1, 0, True)

    # Set settings of servo bricklet 100, 50, 34, 
    servo.set_period(0, 50)
    servo.set_degree(0, 0, 6000)
    servo.set_pulse_width(0, 50, 50)
    print(servo.get_degree(0))
    print(servo.get_period(0))

    # Send PWM signal
    input("Press Enter to start")
    servo.set_position(0, 6000)
    servo.set_enable(0, True)
    print("Should have started")

    # stop the motion
    input("Press Enter to stop")
    servo.set_position(0, 0)
    servo.set_enable(0, False)
    print("Should have stopped")

    ipcon.disconnect()

if __name__ == '__main__':
    main()