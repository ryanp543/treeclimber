# sudo nano /boot/firmware/config.txt
# pip install python-can
# if error RTNETLINK answers: Device or resource busy
# command in terminal: sudo /sbin/ip link set can0 down
import os
import can
import time

POS_MAX = 31.0         # 10pi =~ 31 rad (around 5 whole rotations)
POS_MIN = -31.0
VEL_MAX = 4.0          # 41 rpm =~ 4 rad/s for 70 kg-cm torque
VEL_MIN = -4.0
SERVO_MAX = 270.0
SERVO_MIN = 0.0
KP_MAX = 100.0
KP_MIN = 0.0
KD_MAX = 10.0
KD_MIN = 0.0
KI_MAX = 30.0
KI_MIN = 0.0

# Function receive_status
# Callback function for incoming CANBus messages from the main body
def receive_status(msg):
    id = msg.arbitration_id
    data = msg.data

    values_received = []
    for k in range(0, len(data), 2):
        value_new = (data[k] << 8) + data[k+1]
        values_received.append(value_new)

    if id == 0:
        print(id)
        print(values_received)


# Function map_to_uint16
# Takes a value, a maximum, and a minimum and outputs a 16 bit integer that
# represents the value between the bounds. Note that any value above the max
# or below the min will be set to the max or the min, respectively. 
def map_to_uint16(value, max_value, min_value):
    value_range = max_value - min_value
    value = max(min_value, min(value, max_value))
    scaled_value = int(((value - min_value) / value_range) * 65535)

    return scaled_value & 0xFFFF


# Function create_command
# Takes the message id (which dictates the type of command) and the corresponding
# list of values and returns the 8 bytes of data that will be sent to the Teensy 
# on the main body. 
def create_command(id, value_list):
    # Depending on id, creates a list of the relevant maximums and minimums
    # See comments in send_command function
    match id:
        # ID 0x1: Position
        case 0x1:
            maximums = [POS_MAX, KP_MAX, KD_MAX, KI_MAX]
            minimums = [POS_MIN, KP_MIN, KD_MIN, KI_MIN]
        # ID 0x2: Velocity
        case 0x2:
            maximums = [VEL_MAX, KP_MAX, KD_MAX, KI_MAX]
            minimums = [VEL_MIN, KP_MIN, KD_MIN, KI_MIN]
        # ID 0x3: Main servo roll
        case 0x3:
            maximums = [SERVO_MAX, 1.0, 1.0, 1.0]
            minimums = [SERVO_MIN, 0.0, 0.0, 0.0]
        # ID 0x4: Left servo roll
        case 0x4:
            maximums = [SERVO_MAX, 1.0, 1.0, 1.0]
            minimums = [SERVO_MIN, 0.0, 0.0, 0.0]  
        case _:
            print("DESIRED COMMAND ID NOT WITHIN RANGE.")
    
    # Creates a scaled value from each value in the value list
    # Converts it to bytes and adds it to the command byte array
    scaled_value_list = []
    cmd = bytearray()
    for k in range(len(value_list)):
        scaled_value = map_to_uint16(value_list[k], maximums[k], minimums[k])
        scaled_value_list.append(scaled_value)
        cmd.extend(scaled_value.to_bytes(2, 'big')) 
    
    print(scaled_value_list)
    print(cmd)

    # Returns the 8 bytes of data to be sent representing the uint_16 values
    return cmd


# Function send_commands
# TODO: SEND DIFFERENT COMMANDS AT A RAPID RATE TO MAKE SURE NO CORRUPTION
# OF DATA ON THE RECEIVING END
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
    msg1_id = 0x3
    msg2_id = 0x4
    msg3_id = 0x3
    msg4_id = 0x4

    # Take command and maps to unsigned 16 bit integer values
    # Message packet is 8 bytes, typically four 16 bit unsigned integers
    # command = create_command(msg_id, [-24.6, 40.05, 7.2, 0.4])
    command1 = create_command(msg1_id, [270, 0.0, 0.0, 0.0])
    command2 = create_command(msg2_id, [180, 0.0, 0.0, 0.0])
    command3 = create_command(msg3_id, [0, 0.0, 0.0, 0.0])
    command4 = create_command(msg4_id, [45, 0.0, 0.0, 0.0])

    # Open can bus interface and send the command
    msg1 = can.Message(arbitration_id=msg1_id, data=command1, is_extended_id=False)
    msg2 = can.Message(arbitration_id=msg2_id, data=command2, is_extended_id=False)
    msg3 = can.Message(arbitration_id=msg3_id, data=command3, is_extended_id=False)
    msg4 = can.Message(arbitration_id=msg4_id, data=command4, is_extended_id=False)

    bus.send(msg1)
    time.sleep(0.5)
    bus.send(msg2)
    time.sleep(0.5)
    bus.send(msg3)
    time.sleep(0.1)
    bus.send(msg4)

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