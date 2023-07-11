# pip install python-can
# if error RTNETLINK answers: Device or resource busy
# command in terminal: sudo /sbin/ip link set can0 down
import os
import can
import time

os.system('sudo /sbin/ip link set can0 down')
os.system('sudo ip link set can0 type can bitrate 1000000')
os.system('sudo ifconfig can0 up')

with can.interface.Bus(channel='can0', interface='socketcan', bitrate=1000000) as bus:
    print("yeah ya")
    msg = can.Message(arbitration_id=0x1, data=[1,3,5], is_extended_id=True)
    bus.send(msg)

print('hello world')


os.system('sudo ifconfig can0 down')