# sudo nano /boot/firmware/config.txt
# pip install python-can
# if error RTNETLINK answers: Device or resource busy
# command in terminal: sudo /sbin/ip link set can0 down
import os
import can
import time

os.system('sudo /sbin/ip link set can0 down')
os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')

with can.interface.Bus(channel='can0', interface='socketcan', bitrate=500000) as bus:
    msg = can.Message(arbitration_id=0x123, data=[1,3,5], is_extended_id=False)
    bus.send(msg)

print('hello world')


os.system('sudo ifconfig can0 down')