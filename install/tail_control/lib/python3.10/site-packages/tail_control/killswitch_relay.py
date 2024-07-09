import RPi.GPIO as GPIO
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


# Bypasses killswitch
def activate(node, pin, led):
    node.get_logger().info("REMOTE KILLSWITCH BYPASSED")
    GPIO.output(pin, GPIO.HIGH)
    GPIO.output(led, GPIO.HIGH)


# Publishes every second to ROS topic (for debugging)
def timer_callback():
    msg = String()
    msg.data = 'Killswitch running' 
    ks_pub.publish(msg)


# When Ctrl-C in terminal, triggers killswitch
def deactivate(pin, led):
    GPIO.output(pin, GPIO.LOW)
    GPIO.output(led, GPIO.LOW)
    print("TRIGGERED")


# Main function
def main(args=None):
    # Set up GPIO pin, default it to LOW
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    mosfet_pin = 26
    led_pin = 19
    GPIO.setup(mosfet_pin, GPIO.OUT)
    GPIO.output(mosfet_pin, GPIO.LOW)
    GPIO.setup(led_pin, GPIO.OUT)
    GPIO.output(led_pin, GPIO.LOW)

    # Initialize rclpy and killswitch node
    global ks_node, ks_pub
    rclpy.init(args=args)
    ks_node = rclpy.create_node('killswitch_publisher')
    ks_pub = ks_node.create_publisher(String, 'killswitch_topic', 10)
    activate(ks_node, mosfet_pin, led_pin)
    
    # Create timer
    timer_period = 1
    ks_node.create_timer(timer_period, timer_callback)

    # Spin the ROS node until Ctrl-C
    try:
        rclpy.spin(ks_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            ks_node.destroy_node()
            rclpy.shutdown()

    # Once exited, turn everything off
    deactivate(mosfet_pin, led_pin)


if __name__ == "__main__":
    main()
