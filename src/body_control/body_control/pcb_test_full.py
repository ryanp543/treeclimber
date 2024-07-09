# sudo nano /boot/firmware/config.txt
# pip install python-can
# if error RTNETLINK answers: Device or resource busy
# command in terminal: sudo /sbin/ip link set can0 down
import os
import can
import time
import math
import matplotlib.pyplot as plt

POS_MAX = 31.0         # 10pi =~ 31 rad (around 5 whole rotations)
POS_MIN = -31.0
VEL_MAX = 8.0          # 74 rpm =~ 8 rad/s for 70 kg-cm torque
VEL_MIN = -8.0
TURR_MAX = 12.5
TURR_MIN = -12.5
SERVO_MAX = 270.0
SERVO_MIN = 0.0
NUM_SERVOS = 3
TORQUE_MAX = 6.5        # [N*m], 70 kg stall torque = ~6.8 N*m
TORQUE_MIN = -6.5
KP_MAX = 100.0
KP_MIN = -1.0
KD_MAX = 5.0
KD_MIN = -1.0
KI_MAX = 30.0
KI_MIN = -1.0

position_data = []


# Function receive_status
# Callback function for incoming CANBus messages from the main body
def receive_status(msg):
    global position_data

    id = msg.arbitration_id
    data = msg.data

    values_received = []
    for k in range(0, len(data), 2):
        value_new = (data[k] << 8) + data[k+1]
        values_received.append(value_new)

    if id == 0:

        if values_received[0] == 1:
            # print(id)
            # print(values_received)
            position_data.append(values_received)
            # print(len(position_data))

            if values_received[-3:] == [0, 0, 0]:
                print("All data collected!")


# Function map_to_uint16
# Takes a value, a maximum, and a minimum and outputs a 16 bit integer that
# represents the value between the bounds. Note that any val    print(position_data[-1])ue above the max
# or below the min will be set to the max or the min, respectively. 
def map_to_uint16(value, max_value, min_value):
    value_range = max_value - min_value
    value = max(min_value, min(value, max_value))
    scaled_value = int(((value - min_value) / value_range) * 65535)

    return scaled_value & 0xFFFF


# 
def map_to_float(value, max_value, min_value):
    scaled_float = min_value + (value * (max_value-min_value) / (2**16-1))
    return scaled_float


# Function create_command
# Takes the message id (which dictates the type of command) and the corresponding
# list of values and returns the 8 bytes of data that will be sent to the Teensy 
# on the main body. 
def create_command(id, value_list):
    # Depending on id, creates a list of the relevant maximums and minimums
    # See comments in send_command function
    match id:
        # ID 1: Position
        case 1:
            maximums = [POS_MAX, KP_MAX, KD_MAX, KI_MAX]
            minimums = [POS_MIN, KP_MIN, KD_MIN, KI_MIN]
        # ID 2: Velocity
        case 2:
            maximums = [VEL_MAX, KP_MAX, KD_MAX, KI_MAX]
            minimums = [VEL_MIN, KP_MIN, KD_MIN, KI_MIN]
        # ID 3: Turret motor position
        case 3:
            maximums = [TURR_MAX, KP_MAX, KD_MAX, KI_MAX]
            minimums = [TURR_MIN, KP_MIN, KD_MIN, KI_MIN]
        # ID 4: Left tendon torque control
        case 4:
            maximums = [TORQUE_MAX, POS_MAX, KP_MAX, KI_MAX]
            minimums = [TORQUE_MIN, POS_MIN, KP_MIN, KI_MIN]
        # ID 5: Right tendon torque control
        case 5:
            maximums = [TORQUE_MAX, POS_MAX, KP_MAX, KI_MAX]
            minimums = [TORQUE_MIN, POS_MIN, KP_MIN, KI_MIN]
        # ID 6: Left and right servo rolls
        case 6: 
            maximums = [NUM_SERVOS, SERVO_MAX, 1.0, 1.0]
            minimums = [1.0, SERVO_MIN, 0.0, 0.0]
        case 8:
            maximums = [1.0, 1.0, 1.0, 1.0]
            minimums = [0.0, 0.0, 0.0, 0.0]  
        # ID 11: Get drive encoder data
        case 11:
            maximums = [1.0, 1.0, 1.0, 1.0]
            minimums = [0.0, 0.0, 0.0, 0.0]  
        case _:
            print("DESIRED COMMAND ID NOT WITHIN RANGE.")
    
    # Creates a scaled value from each value in the value list
    # Converts it to bytes and adds it to the command byte array
    scaled_value_list = []
    cmd = bytearray()
    for k in range(len(value_list)):
        scaled_value = map_to_uint16(value_list[k], maximums[k],
 minimums[k])
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
    # 0x3 = turret motor position
    # 0x4 = left tendon
    # 0x5 = right tendon
    # 0x6 = (facing from back tail counterweight) left = 2, right = 3 servo roll
    # 0x7 = clippers
    # 0xa = enable motor (i.e. reset encoders)
    # 0xb = get drive encoder data


    # Take command and maps to unsigned 16 bit integer values
    # Message packet is 8 bytes, typically four 16 bit unsigned integers

    # command1 = create_command(msg1_id, [1, 180, 0.0, 0.0]) # for servo control
    # command2 = create_command(msg2_id, [2, 180, 0, 0])

    # command1 = create_command(msg1_id, [1.5*math.pi, 10.0, 0, 0.1]) # for velocity control
    # command2 = create_command(msg2_id, [-1.5*math.pi, 10.0, 0, 0.1])
    # command3 = create_command(msg3_id, [0.0, 2.0, 2.0, 20])
    # command4 = create_command(msg4_id, [1, 0.0, 0.0, 0.0])

    # command1 = create_command(msg1_id, [-math.pi/2, 25, 2, 5]) # for turret motor control
    # command2 = create_command(msg1_id, [math.pi/2, 25, 2, 5]) # for turret motor control

    # for left tendon motor control
    msg1_id = 4
    command1 = create_command(msg1_id, [3, 5, 1, 2.2])
    msg1 = can.Message(arbitration_id=msg1_id, data=command1, is_extended_id=False)

    # for right tendon motor control
    msg2_id = 5
    command2 = create_command(msg2_id, [0, -4, 0, 0]) 
    msg2 = can.Message(arbitration_id=msg2_id, data=command2, is_extended_id=False)

    # Drive wheel position control
    msg3_id = 1
    command3 = create_command(msg3_id, [2*math.pi, 40, 1, 0.1]) 
    msg3 = can.Message(arbitration_id=msg3_id, data=command3, is_extended_id=False)

    # Rotate turret
    msg4_id = 3
    command4 = create_command(msg4_id, [-math.pi/2, 25, 2, 5])
    msg4 = can.Message(arbitration_id=msg4_id, data=command4, is_extended_id=False)
    
    # Drive wheel velocity
    msg5_id = 2
    msg6_id = 2
    msg7_id = 2
    command5 = create_command(msg5_id, [1.5*math.pi, 10.0, 0, 0.1]) 
    command6 = create_command(msg6_id, [-1.5*math.pi, 10.0, 0, 0.1])
    command7 = create_command(msg7_id, [0.0, 2.0, 2.0, 20])
    msg5 = can.Message(arbitration_id=msg5_id, data=command5, is_extended_id=False)
    msg6 = can.Message(arbitration_id=msg6_id, data=command6, is_extended_id=False)
    msg7 = can.Message(arbitration_id=msg7_id, data=command7, is_extended_id=False)

    # Drive servos
    msg8_id = 6
    msg9_id = 6
    msg10_id = 6
    msg11_id = 6
    command8 = create_command(msg8_id, [1, 180, 0.0, 0.0]) 
    command9 = create_command(msg9_id, [1, 0, 0.0, 0.0])
    command10 = create_command(msg10_id, [2, 180, 0.0, 0.0])
    command11 = create_command(msg11_id, [2, 0, 0.0, 0.0])
    msg8 = can.Message(arbitration_id=msg8_id, data=command8, is_extended_id=False)
    msg9 = can.Message(arbitration_id=msg9_id, data=command9, is_extended_id=False)
    msg10 = can.Message(arbitration_id=msg10_id, data=command10, is_extended_id=False)
    msg11 = can.Message(arbitration_id=msg11_id, data=command11, is_extended_id=False)


    # Open can bus interface and send the command
    bus.send(msg1)
    print("Sent first message")
    time.sleep(4)

    bus.send(msg2)
    print("Sent second message")
    time.sleep(4)

    bus.send(msg3)
    print("Sent third message")
    time.sleep(3)

    bus.send(msg4)
    print("Sent fourth message")
    time.sleep(3)

    bus.send(msg5)
    print("Sent 5th message")
    time.sleep(3)
    
    bus.send(msg6)
    print("Sent 6th message")
    time.sleep(3)

    bus.send(msg7)
    print("Sent 7th message")
    time.sleep(3)

    bus.send(msg8)
    print("Sent 8th message")
    time.sleep(1)

    bus.send(msg9)
    print("Sent 9th message")
    time.sleep(1)

    bus.send(msg10)
    print("Sent 10th message")
    time.sleep(1)

    bus.send(msg11)
    print("Sent 11th message")
    time.sleep(1)

    print("Command sent!")

# Function plot_data
# This function processes and plots the encoder data received over CANBus from Teensy
def plot_data():
    global position_data

    print(len(position_data))
    t = []
    t_vel = []
    pos = []
    vel = []

    # If data received is for position
    if position_data[0][0] == 1:
        t_start = (position_data[0][1] << 16) | position_data[0][2]

        for k in range(len(position_data)):
            if position_data[k] != [1, 0, 0, 0]: 
                t_at_k = (position_data[k][1] << 16) | position_data[k][2]
                pos_at_k = map_to_float(position_data[k][3], POS_MAX, POS_MIN)

                t.append((t_at_k - t_start)/1000)
                pos.append(pos_at_k)
            else:
                del t[-1]
                del pos[-1]
                break
    else:
        print("This data is for something else")

    # Create an array of velocity from position and time arrays pos and t
    inc = 5
    for k in range(inc+1, len(t), inc):
        new_vel = 0 # (pos[k] - pos[k-inc]) / (t[k] - t[k-inc])
        vel.append(new_vel)
        t_vel.append(t[k])
    
    print(pos[-1])
    print(vel[-1])

    # fig, axs = plt.subplots(2, 1)
    plt.plot(t, pos)
    plt.grid(True)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")

    # axs[1].plot(t_vel, vel)
    # axs[1].grid(True)
    # axs[1].set_xlabel("Time (s)")
    # axs[1].set_ylabel("Velocity (rad/s)")

    plt.show()

    position_data = []


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

    # Plot data
    


    