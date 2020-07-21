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
        self.tx_buffer_length = 12
        self.rx_buffer_length = 20

        self.buffer_size_curr = 0
        self.buffer_size_prev = 0

        self.ReceivedData = {key:[] for key in REC_DATA}

        # AUTO-DETECT the Arduino! By Deepak
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino' in p.description]

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

        self.hand_shaking_protocol()

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


    def send_command(self,command):
        
        # print('Sending data to uController')
        # print(command)

        cmd = bytearray(self.tx_buffer_length)
        
        cmd[0],cmd[1] = split_int_2byte(round(command[0]*100))                #liquid_lens_freq
        cmd[2] = int(command[1])                                                   # Focus-Tracking ON or OFF
        cmd[3] = int(command[2])                                                   #Homing
        cmd[4] = int(command[3])                                                   #tracking
        cmd[5],cmd[6] = split_signed_int_2byte(round(command[4]*100))         #Xerror
        cmd[7],cmd[8] = split_signed_int_2byte(round(command[5]*100))         #Yerror                           
        cmd[9],cmd[10] = split_signed_int_2byte(round(command[6]*100))        #Zerror
        cmd[11] = int(command[7])                                             # Stage-zero command    
        
        print('Zero stage : {}'.format(cmd[11]))

        self.serial.write(cmd)

    def read_received_packet(self):

        # self.serial.reset_input_buffer()
        # wait to receive data


        

        while self.serial.in_waiting==0:
            print(self.serial.in_waiting)
            print('wait for data to arrive:1')
            pass

        while self.serial.in_waiting % self.rx_buffer_length != 0:
            print(self.serial.in_waiting)
            print('wait for data to arrive:2')
            pass
        

        num_bytes_in_rx_buffer = self.serial.in_waiting

        # print("number of bytes in the Rx buffer: " + str(num_bytes_in_rx_buffer))

        # get rid of old data
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            # print('getting rid of old data')
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

        # print('Focus phase: {}'.format(self.ReceivedData['FocusPhase']))
        # print('X-stage: {}'.format(self.ReceivedData['X_stage']))
        # print('Y-stage: {}'.format(self.ReceivedData['Y_stage']))
        # print('Theta-stage: {}'.format(self.ReceivedData['Theta_stage']))
        # print('Track stage: {}'.format(self.ReceivedData['track_obj_stage']))
        # print('Track image: {}'.format(self.ReceivedData['track_obj_image_hrdware']))



       

        return self.ReceivedData
        # return YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino, LED_measured, tracking_triggered,manualMode
        
        # return data

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

        return self.ReceivedData

# Define a micro controller emulator

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        #REC_DATA = ['FocusPhase', 'X_stage', 'Y_stage', 'Theta_stage', 'track_obj_image', 'track_obj_stage']
        self.FocusPhase = 0
        self.Xpos = 0
        self.Ypos = 0
        self.Thetapos = 0

        self.X_stage = 0
        self.Y_stage = 0
        self.Theta_stage = 0
        
        self.track_obj_image = 1
        self.track_focus = 0

        self.track_obj_stage = 1

        self.RecData = {'FocusPhase':self.FocusPhase, 'X_stage': self.X_stage, 'Y_stage':self.Y_stage, 'Theta_stage':self.Theta_stage, 'track_obj_image':self.track_obj_image, 'track_obj_stage' : self.track_obj_stage}

        self.SendData = {key:[] for key in SEND_DATA}

    def close(self):
        pass

    def toggle_LED(self,state):
        pass
    
    def toggle_laser(self,state):
        pass

    def move_x(self,delta):
        
        self.X_stage += delta

    def move_y(self,delta):
        
        self.Y_stage += delta


    def move_Theta(self,delta):
        
        self.Theta_stage += delta

    def move_z(self,delta):
        pass
    
    def send_command(self, command):
        print(command)
        #SEND_DATA = ['X_order', 'Y_order', 'Z_order', 'track_obj_image', 'track_focus', 'homing_state']
        X_order, Y_order, Theta_order = command[4], command[5], command[6]
        self.track_obj_image = command[3]
        self.track_focus = command[1]

        self.move_x(X_order)
        self.move_y(Y_order)
        self.move_Theta(Theta_order)


    def read_received_packet_nowait(self):
        self.RecData = {'FocusPhase':self.FocusPhase, 'X_stage': self.X_stage, 'Y_stage':self.Y_stage, 'Theta_stage':self.Theta_stage, 'track_obj_image':self.track_obj_image, 'track_obj_stage' : self.track_obj_stage}
        self.deltaX_stage = 0
        self.deltaY_stage = 0
        self.deltaTheta_stage = 0
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
