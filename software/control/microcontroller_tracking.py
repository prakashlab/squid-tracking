import platform
import serial
import serial.tools.list_ports
import time
import numpy as np
import warnings

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = ucontroller.CMD_LENGTH
        self.rx_buffer_length = ucontroller.MSG_LENGTH

        self.buffer_size_curr = 0
        self.buffer_size_prev = 0

        self.ReceivedData = {key:[] for key in REC_DATA}

        # AUTO-DETECT the Arduino! By Deepak
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino Due' == p.description]

        print(arduino_ports)

        if not arduino_ports:
            raise IOError("No Arduino found")
        if len(arduino_ports) > 1:
            warnings.warn('Multiple Arduinos found - using the first')
        else:
            print('Using Arduino found at : {}'.format(arduino_ports[0]))

        # establish serial communication
        self.serial = serial.Serial(arduino_ports[0],2000000)
        time.sleep(0.2)
        print('Serial Connection Open')

        # Simplifying the serial conn startup
        # self.hand_shaking_protocol()

    def hand_shaking_protocol(self):
        # Read string from Arduino
        print('try handshaking')
        initial_number = ord(self.serial.read())
        print(initial_number)
        print('first number received')
        if(initial_number == 1):
            print('\n ------------Communication established with the Arduino------------\n')
            cmd=bytearray(1)
            cmd[0]=2
            self.serial.write(cmd)

        second_number = ord(self.serial.read())
            
        if(second_number == 2):
            print('\n ------------Communication established both ways with the Arduino------------\n')
        print('handshaking finished')

    def close(self):
        self.serial.close()


    def send_motion_command_xytheta(self, microsteps_x, microsteps_y, microsteps_theta):
        # For gravity machine

        if(microsteps_x>=0):
            direction_x = 1;
        else:
            direction_x = 0;

        if(microsteps_y>=0):
            direction_y = 1;
        else:
            direction_y = 0;

        if(microsteps_theta>=0):
            direction_theta = 1;
        else:
            direction_theta = 0;

        microsteps_x = abs(microsteps_x)
        microsteps_y = abs(microsteps_y)
        microsteps_theta = abs(microsteps_theta)

        cmd = bytearray(self.tx_buffer_length)

        cmd[0] = ord('M')

        cmd[1] = direction_x
        cmd[2] = int(microsteps_x) >> 8
        cmd[3] = int(microsteps_x) & 0xff

        cmd[4] = direction_y
        cmd[5] = int(microsteps_y) >> 8
        cmd[6] = int(microsteps_y) & 0xff

        cmd[7] = direction_theta
        cmd[8] = int(microsteps_theta) >> 8
        cmd[9] = int(microsteps_theta) & 0xff

        self.serial.write(cmd)

    def send_motion_command_xy(self, microsteps_x, microsteps_y):
        # For squid

        if(microsteps_x>=0):
            direction_x = 1;
        else:
            direction_x = 0;

        if(microsteps_y>=0):
            direction_y = 1;
        else:
            direction_y = 0;

        microsteps_x = abs(microsteps_x)
        microsteps_y = abs(microsteps_y)

        cmd = bytearray(self.tx_buffer_length)

        cmd[0] = ord('M')

        cmd[1] = direction_x
        cmd[2] = int(microsteps_x) >> 8
        cmd[3] = int(microsteps_x) & 0xff

        cmd[4] = direction_y
        cmd[5] = int(microsteps_y) >> 8
        cmd[6] = int(microsteps_y) & 0xff

        self.serial.write(cmd)

    def send_tracking_command(self, tracking_flag):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('F')
        cmd[1] = ord('O')
        cmd[2] = tracking_flag

        self.serial.write(cmd)

    def send_focus_tracking_command(self, focus_tracking_flag):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('F')
        cmd[1] = ord('L')
        cmd[2] = focus_tracking_flag

        self.serial.write(cmd)

    def send_homing_command(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('F')
        cmd[1] = ord('H')

        self.serial.write(cmd)

    def send_stage_zero_command(self, stage):

        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('0')
        cmd[1] = ord(stage)

        self.serial.write(cmd)

    def send_liquid_lens_freq(self, freq):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('P')
        cmd[1] = ord('L')
        cmd[2] = ord('F')

        cmd[3], cmd[4] = split_int_2byte(round(freq*100)) 

        self.serial.write(cmd)

    def send_liquid_lens_amp(self, amp):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('P')
        cmd[1] = ord('L')
        cmd[2] = ord('A')

        cmd[3], cmd[4] = split_int_2byte(round(amp*100)) 

        self.serial.write(cmd)

    def send_liquid_lens_offset(self, offset):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('P')
        cmd[1] = ord('L')
        cmd[2] = ord('O')

        cmd[3], cmd[4] = split_int_2byte(round(offset*100)) 

        self.serial.write(cmd)



    # def send_command(self,command):
        
    #     # print('Sending data to uController')
    #     # print(command)

    #     cmd = bytearray(self.tx_buffer_length)
        
    #     cmd[0],cmd[1] = split_int_2byte(round(command[0]*100))                #liquid_lens_freq
    #     cmd[2] = int(command[1])                                                   # Focus-Tracking ON or OFF
    #     cmd[3] = int(command[2])                                                   #Homing
    #     cmd[4] = int(command[3])                                                   #tracking
    #     cmd[5],cmd[6] = split_signed_int_2byte(round(command[4]*100))         #Xerror
    #     cmd[7],cmd[8] = split_signed_int_2byte(round(command[5]*100))         #Yerror                           
    #     cmd[9],cmd[10] = split_signed_int_2byte(round(command[6]*100))        #Zerror
    #     cmd[11] = int(command[7])                                             # Stage-zero command    
        
    #     print('Zero stage : {}'.format(cmd[11]))

    #     self.serial.write(cmd)

    def read_received_packet_nowait(self):
        # wait to receive data
        if self.serial.in_waiting==0:
            return None
        if self.serial.in_waiting % self.rx_buffer_length != 0:
            return None
        
        # get rid of old data
        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))


        self.ReceivedData['FocusPhase']  = data2byte_to_int(data[0],data[1])*2*np.pi/65535.

        self.ReceivedData['X_stage']  = data[3]*2**24 + data[4]*2**16+data[5]*2**8 + data[6]
        if data[2]==1:
            self.ReceivedData['X_stage'] = -self.ReceivedData['X_stage']
        self.ReceivedData['Y_stage'] = data[8]*2**24 + data[9]*2**16+data[10]*2**8 + data[11]
        if data[7]==1:
            self.ReceivedData['Y_stage'] = -self.ReceivedData['Y_stage']
        self.ReceivedData['Theta_stage'] = data[13]*2**24 + data[14]*2**16+data[15]*2**8 + data[16]
        if data[12]==1:
            self.ReceivedData['Theta_stage'] = -self.ReceivedData['Theta_stage']
        
        self.ReceivedData['track_obj_stage'] = data[17]

        self.ReceivedData['track_obj_image_hrdware'] = bool(data[18])

        self.ReceivedData['homing_complete'] = bool(data[19])

        # print(self.ReceivedData)
        return self.ReceivedData

# Define a micro controller emulator

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        #REC_DATA = ['FocusPhase', 'X_stage', 'Y_stage', 'Theta_stage', 'track_obj_image', 'track_obj_stage']
        self.tx_buffer_length = 10
        self.FocusPhase = 0
        self.Xpos = 0
        self.Ypos = 0
        self.Thetapos = 0

        self.X_stage = 0
        self.Y_stage = 0
        self.Z_stage = 0
        
        self.track_obj_image = 1
        self.track_focus = 0

        self.track_obj_stage = 1
        self.track_obj_image_hrdware = 0

        self.liquid_lens_freq = 0
        self.liquid_lens_amp = 0
        self.liquid_lens_offset = 0

        self.homing_complete = 0

        self.RecData = {'FocusPhase':self.FocusPhase, 'X_stage': self.X_stage, 'Y_stage':self.Y_stage, 'Z_stage':self.Z_stage, 'track_obj_image':self.track_obj_image, 'track_obj_stage' : self.track_obj_stage, 'track_obj_image_hrdware':self.track_obj_image_hrdware, 'homing_complete':self.homing_complete}

        self.SendData = {key:[] for key in SEND_DATA}

        self.time_start = time.time()

    def close(self):
        pass

    def send_motion_command_xytheta(self, microsteps_x, microsteps_y, microsteps_theta):
        # For gravity machine

        if(microsteps_x>=0):
            direction_x = 1;
        else:
            direction_x = 0;

        if(microsteps_y>=0):
            direction_y = 1;
        else:
            direction_y = 0;

        if(microsteps_theta>=0):
            direction_theta = 1;
        else:
            direction_theta = 0;

        microsteps_x = abs(microsteps_x)
        microsteps_y = abs(microsteps_y)
        microsteps_theta = abs(microsteps_theta)

        cmd = bytearray(self.tx_buffer_length)

        cmd[0] = ord('M')

        cmd[1] = direction_x
        cmd[2] = int(microsteps_x) >> 8
        cmd[3] = int(microsteps_x) & 0xff

        cmd[4] = direction_y
        cmd[5] = int(microsteps_y) >> 8
        cmd[6] = int(microsteps_y) & 0xff

        cmd[7] = direction_theta
        cmd[8] = int(microsteps_theta) >> 8
        cmd[9] = int(microsteps_theta) & 0xff

        self.X_stage += microsteps_x
        self.Y_stage += microsteps_y
        self.Theta_stage += microsteps_theta

        # self.serial.write(cmd)

    def send_motion_command_xyz(self, microsteps_x, microsteps_y, microsteps_z):
        # For squid tracking

        if(microsteps_x>=0):
            direction_x = 1;
        else:
            direction_x = 0;

        if(microsteps_y>=0):
            direction_y = 1;
        else:
            direction_y = 0;

        if(microsteps_z>=0):
            direction_z = 1;
        else:
            direction_z = 0;

        microsteps_x = abs(microsteps_x)
        microsteps_y = abs(microsteps_y)
        microsteps_z = abs(microsteps_z)

        cmd = bytearray(self.tx_buffer_length)

        cmd[0] = ord('M')

        cmd[1] = direction_x
        cmd[2] = int(microsteps_x) >> 8
        cmd[3] = int(microsteps_x) & 0xff

        cmd[4] = direction_y
        cmd[5] = int(microsteps_y) >> 8
        cmd[6] = int(microsteps_y) & 0xff

        cmd[7] = direction_z
        cmd[8] = int(microsteps_z) >> 8
        cmd[9] = int(microsteps_z) & 0xff

        self.X_stage += microsteps_x
        self.Y_stage += microsteps_y
        self.Z_stage += microsteps_z

        # self.serial.write(cmd)

    def send_tracking_command(self, tracking_flag):

        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('F')
        cmd[1] = ord('O')
        cmd[2] = tracking_flag


        self.track_obj_image = tracking_flag

        print('Set tracking flag at uController: {}'.format(tracking_flag))


        # self.serial.write(cmd)

    def send_focus_tracking_command(self, focus_tracking_flag):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('F')
        cmd[1] = ord('L')
        cmd[2] = focus_tracking_flag

        self.track_focus = focus_tracking_flag

        print('Set focus-tracking flag at uController: {}'.format(focus_tracking_flag))


        # self.serial.write(cmd)

    def send_homing_command(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('F')
        cmd[1] = ord('H')

        print('Sent Homing command to uController')


        # self.serial.write(cmd)

    def send_stage_zero_command(self, stage):

        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('0')
        cmd[1] = ord(stage)

        print('Sent stage {} zero command to uController'.format(stage))


        # self.serial.write(cmd)

    def send_liquid_lens_freq(self, freq):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('P')
        cmd[1] = ord('L')
        cmd[2] = ord('F')

        cmd[3], cmd[4] = split_int_2byte(round(freq*100)) 

        self.liquid_lens_freq = freq

        print('Sent liquid lens freq: {} to uController'.format(freq))

        # self.serial.write(cmd)

    def send_liquid_lens_amp(self, amp):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('P')
        cmd[1] = ord('L')
        cmd[2] = ord('A')

        cmd[3], cmd[4] = split_int_2byte(round(amp*100)) 

        self.liquid_lens_amp = amp


        print('Sent liquid lens amp: {} to uController'.format(amp))


        # self.serial.write(cmd)

    def send_liquid_lens_offset(self, offset):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = ord('P')
        cmd[1] = ord('L')
        cmd[2] = ord('O')

        cmd[3], cmd[4] = split_int_2byte(round(offset*100)) 

        self.liquid_lens_offset = offset

        print('Sent liquid lens offset: {} to uController'.format(offset))


        # self.serial.write(cmd)

    def simulate_focus_phase(self):
        self.elapsed_time = time.time() - self.time_start
        self.FocusPhase = 2*np.pi*self.liquid_lens_freq*self.elapsed_time
        if(self.FocusPhase>2*np.pi):
            self.FocusPhase = 0

    def read_received_packet_nowait(self):

        self.simulate_focus_phase()

        self.RecData = {'FocusPhase':self.FocusPhase, 'X_stage': self.X_stage, 'Y_stage':self.Y_stage, 'Z_stage':self.Z_stage, 'track_obj_image':self.track_obj_image, 'track_obj_stage' : self.track_obj_stage, 'track_obj_image_hrdware':self.track_obj_image_hrdware, 'homing_complete':self.homing_complete}
        return self.RecData



# from Gravity machine
def split_int_2byte(number):
    return int(number)% 256,int(number) >> 8

def split_signed_int_2byte(number):
    if abs(number) > 32767:
        number = np.sign(number)*32767

    if number!=abs(number):
        number=65536+number
    return int(number)% 256,int(number) >> 8

def split_int_3byte(number):
    return int(number)%256, int(number) >> 8, int(number) >> 16

def data2byte_to_int(a,b):
    return a + 256*b

def data2byte_to_signed_int(a,b):
    nb= a+256*b
    if nb>32767:
        nb=nb-65536
    return nb

def data4byte_to_int(a,b,c,d):
    return a + (256)*b + (65536)*c + (16777216)*d
