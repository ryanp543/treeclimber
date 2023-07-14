# sudo nano /boot/firmware/config.txt
# pip install python-can
# if error RTNETLINK answers: Device or resource busy
# command in terminal: sudo /sbin/ip link set can0 down
import os
import can
import time

def receive_status(msg):
    id = msg.arbitration_id
    data = msg.data

    values_received = []
    for k in range(0, len(data), 2):
        value_new = (data[k] << 8) + data[k+1]
        values_received.append(value_new)

    print(id)
    print(values_received)

def send_commands(bus):

    # Message IDs for different commands
    # 0x1 = wheel position (value, P, I, D)
    # 0x2 = wheel velocity
    # 0x3 = main servo roll
    # 0x4 = left servo (facing back from tail counterweight)
    # 0x5 = right servo (facing back from tail counterweight)
    # 0x6 = left tendon
    # 0x7 = right tendon
    # 0x8 = clippers
    msg_id = 0x1

    # How to encode convert strings to bytes
    feature = "p"
    feature_bytes = feature.encode()

    # Message packet is 8 bytes, typically four 16 bit unsigned integers
    values = [65535, 31231, 1511, 226] # 16 bit integers unsigned
    command = bytearray()
    for k in range (len(values)):
        command.extend(values[k].to_bytes(2, 'big')) 

    print(command)
    print(len(command))

    # Open can bus interface and send the command
    msg = can.Message(arbitration_id=msg_id, data=command, is_extended_id=False)
    bus.send(msg)

    print("Command sent!")

if __name__ == "__main__":
    # Run OS terminal commands to set up and open CANBus ports
    os.system('sudo /sbin/ip link set can0 down')
    os.system('sudo ip link set can0 type can bitrate 1000000')
    os.system('sudo ifconfig can0 up')

    # Set up CAN bus object
    bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=1000000)

    # Set up CAN bus listener
    notifier = can.Notifier(bus, [receive_status])

    # Run appropriate function to send commands
    send_commands(bus)

    # Shutdown the bus
    input("Press any key to shut down.")
    print("Shutting down, finishing script.")
    notifier.stop()
    bus.shutdown()
    os.system('sudo ifconfig can0 down')