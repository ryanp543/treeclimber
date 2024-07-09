import body_controller
import math
import time

# Message IDs for different commands
# 0x1 = wheel position (value, P, I, D)
# 0x2 = wheel velocity
# 0x3 = turret motor position
# 0x4 = left tendon
# 0x5 = right tendon
# 0x6 = (facing from back tail counterweight) left = 2, right = 3 servo roll
# 0x7 = clippers
# 0xa = enable motor (i.e. reset encoders)
# 0xb = get drive encoder data

def main():
    my_controller = body_controller.MyController()
    my_controller.send_command(1, [2*math.pi, 40, 1, 0.1])
    time.sleep(1)
    my_controller.shutdown()

if __name__ == "__main__":
    main()