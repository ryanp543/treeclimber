import os
import can

POS_MAX = 31.0         # 10pi =~ 31 rad (around 5 whole rotations)
POS_MIN = -31.0
VEL_MAX = 8.0          # 74 rpm =~ 8 rad/s for 70 kg-cm torque
VEL_MIN = -8.0
TURR_MAX = 12.5
TURR_MIN = -12.5
SERVO_MAX = 105.0
SERVO_MIN = -195.0
NUM_SERVOS = 3
TORQUE_MAX = 6.5        # [N*m], 70 kg stall torque = ~6.8 N*m
TORQUE_MIN = -6.5
KP_MAX = 100.0
KP_MIN = -1.0
KD_MAX = 5.0
KD_MIN = -1.0
KI_MAX = 30.0
KI_MIN = -1.0

class MyController:
    # Initialization function
    def __init__(self):
        os.system('sudo /sbin/ip link set can0 down')
        os.system('sudo ip link set can0 type can bitrate 1000000')
        os.system('sudo ifconfig can0 up')
        self.bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=1000000)
        self.notifier = can.Notifier(self.bus, [self.receive_status])
        self.position_data = []

        print("CANbus module established")


    # Function receive_status
    # Callback function for incoming CANBus messages from the main body
    def receive_status(self, msg):
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
                self.position_data.append(values_received)
                # print(len(position_data))

                if values_received[-3:] == [0, 0, 0]:
                    print("All data collected!")
        

    # Function map_to_uint16
    # Takes a value, a maximum, and a minimum and outputs a 16 bit integer that
    # represents the value between the bounds. Note that value above the max
    # or below the min will be set to the max or the min, respectively. 
    def map_to_uint16(self, value, max_value, min_value):
        value_range = max_value - min_value
        value = max(min_value, min(value, max_value))
        scaled_value = int(((value - min_value) / value_range) * 65535)
        return scaled_value & 0xFFFF
    
    
    # Function create_command
    # Takes the message id (which dictates the type of command) and the corresponding
    # list of values and returns the 8 bytes of data that will be sent to the Teensy 
    # on the main body. 
    def create_command(self, id, value_list):
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
            scaled_value = self.map_to_uint16(value_list[k], maximums[k], minimums[k])
            scaled_value_list.append(scaled_value)
            cmd.extend(scaled_value.to_bytes(2, 'big')) 
        
        # Create string for printing
        value_string = ', '.join(map(str, value_list))
        scaled_string = ', '.join(map(str, scaled_value_list))
        cmd_string = repr(cmd) #', '.join(map(str, cmd))
        output_print_string = f"Cmd: [{value_string}], Bytes: {cmd_string}"
        print(output_print_string)

        # Returns the 8 bytes of data to be sent representing the uint_16 values
        return cmd
    
    
    # Function send_commands
    # TODO: SEND DIFFERENT COMMANDS AT A RAPID RATE TO MAKE SURE NO CORRUPTION
    # OF DATA ON THE RECEIVING END
    def send_command(self, id, command_values):
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

        command = self.create_command(id, command_values)
        msg = can.Message(arbitration_id=id, data=command, is_extended_id=False)

        # Open can bus interface and send the command
        self.bus.send(msg)
        # print("Sent first message")

    
    # Turns off receiver and can module 
    def shutdown(self):
        self.notifier.stop()
        self.bus.shutdown()
        os.system('sudo ifconfig can0 down')
