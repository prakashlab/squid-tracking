import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = MicrocontrollerDef.CMD_LENGTH
        self.rx_buffer_length = MicrocontrollerDef.MSG_LENGTH

        # AUTO-DETECT the Arduino! By Deepak
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino' in p.description]
        if not arduino_ports:
            raise IOError("No Arduino found")
        if len(arduino_ports) > 1:
            print('Multiple Arduinos found - using the first')
        else:
            print('Using Arduino found at : {}'.format(arduino_ports[0]))

        # establish serial communication
        self.serial = serial.Serial(arduino_ports[0],2000000)
        time.sleep(0.2)
        print('Serial Connection Open')

    def close(self):
        self.serial.close()

    def toggle_LED(self,state):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 3
        cmd[1] = state
        self.serial.write(cmd)
    
    def toggle_laser(self,state):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 4
        cmd[1] = state
        self.serial.write(cmd)

    def move_x(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.STEPS_PER_MM_X*Motion.MAX_MICROSTEPS)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 0
        cmd[1] = direction
        cmd[2] = round(n_microsteps) >> 8
        cmd[3] = round(n_microsteps) & 0xff
        self.serial.write(cmd)
        # time.sleep(WaitTime.BASE + WaitTime.X*abs(delta))
        print('Moving  x stage')
        print('X command sent to uController: {} {}'.format(np.sign(delta),n_microsteps))

    def move_y(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.STEPS_PER_MM_Y*Motion.MAX_MICROSTEPS)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 1
        cmd[1] = direction
        cmd[2] = round(n_microsteps) >> 8
        cmd[3] = round(n_microsteps) & 0xff
        self.serial.write(cmd)
        # time.sleep(WaitTime.BASE + WaitTime.Y*abs(delta))
        print('Moving  y stage')
        print('Y command sent to uController: {} {}'.format(np.sign(delta),n_microsteps))

    def move_z(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.STEPS_PER_MM_Z*Motion.MAX_MICROSTEPS)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 2
        cmd[1] = 1-direction
        cmd[2] = round(n_microsteps) >> 8
        cmd[3] = round(n_microsteps) & 0xff
        self.serial.write(cmd)
        # time.sleep(WaitTime.BASE + WaitTime.Z*abs(delta))
        print('Z command sent to uController: {} {}'.format(np.sign(delta),n_microsteps))

    def move_theta(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.STEPS_PER_REV_THETA_SHAFT*Motion.MAX_MICROSTEPS/(2*np.pi))
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 3
        cmd[1] = 1-direction
        cmd[2] = round(n_microsteps) >> 8
        cmd[3] = round(n_microsteps) & 0xff
        self.serial.write(cmd)
        # time.sleep(WaitTime.BASE + WaitTime.Z*abs(delta))
        print('Theta command sent to uController: {} {}'.format(np.sign(delta),n_microsteps))

    def move_x_nonblocking(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.MAX_MICROSTEPS)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 0
        cmd[1] = direction
        cmd[2] = int(n_microsteps) >> 8
        cmd[3] = int(n_microsteps) & 0xff
        self.serial.write(cmd)
        # print('x non-blocking command sent to uController: {}'.format(n_microsteps))

    def move_y_nonblocking(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.MAX_MICROSTEPS)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 1
        cmd[1] = direction
        cmd[2] = int(n_microsteps) >> 8
        cmd[3] = int(n_microsteps) & 0xff
        self.serial.write(cmd)
        # print('y non-blocking command sent to uController: {}'.format(n_microsteps))

    def move_z_nonblocking(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 2
        cmd[1] = 1-direction
        cmd[2] = int(n_microsteps) >> 8
        cmd[3] = int(n_microsteps) & 0xff
        self.serial.write(cmd)

    def move_theta_nonblocking(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.MAX_MICROSTEPS)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 3
        cmd[1] = 1-direction
        cmd[2] = int(n_microsteps) >> 8
        cmd[3] = int(n_microsteps) & 0xff
        self.serial.write(cmd)
        # print('Theta non-blocking command sent to uController: {}'.format(n_microsteps))


    # Convert below functions to be compatible with squid/octopi serial interface.
    def send_tracking_command(self, tracking_flag):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 4
        cmd[1] = tracking_flag

        self.serial.write(cmd)

    def send_homing_command(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 5
        cmd[1] = 1

        self.serial.write(cmd)

    def send_stage_zero_command(self, stage):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 6
        cmd[1] = stage

        self.serial.write(cmd)

    def send_focus_tracking_command(self, focus_tracking_flag):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 7
        cmd[1] = focus_tracking_flag

        self.serial.write(cmd)

    def send_liquid_lens_freq(self, freq):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 8
        cmd[1] = 0
        cmd[2], cmd[3] = split_int_2byte(round(freq*100)) 

        self.serial.write(cmd)

    def send_liquid_lens_amp(self, amp):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 8
        cmd[1] = 1
        cmd[2], cmd[3] = split_int_2byte(round(amp*100)) 

        self.serial.write(cmd)

    def send_liquid_lens_offset(self, offset):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 8
        cmd[1] = 2
        cmd[2], cmd[3] = split_int_2byte(round(offset*100)) 

        self.serial.write(cmd)

    def send_hardware_trigger_command(self, trigger_state):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 9
        cmd[1] = trigger_state

        self.serial.write(cmd)

    def set_number_of_planes_per_volume(self,value):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = SET_NUMBER_OF_PLANES_PER_VOLUME
        cmd[1] = value >> 8
        cmd[2] = value & 0xff
        self.serial.write(cmd)

    def set_number_of_requested_volumes(self,value):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = SET_NUMBER_OF_REQUESTED_VOLUMES
        cmd[1] = value >> 16
        cmd[2] = (value >> 8) & 0xff
        cmd[3] = value & 0xff
        self.serial.write(cmd)

    def set_frequency_Hz(self,value):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = SET_FREQUENCY_HZ
        cmd[1] = int(value*1000) >> 8
        cmd[2] = int(value*1000) & 0xff
        self.serial.write(cmd)

    def set_phase_delay(self,value):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = SET_PHASE_DELAY
        value = int((value/90)*65535.0)
        cmd[1] = int(value) >> 8
        cmd[2] = int(value) & 0xff
        self.serial.write(cmd)

    def start_trigger_generation(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = START_TRIGGER_GENERATION
        self.serial.write(cmd)

    def stop_trigger_generation(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = STOP_TRIGGER_GENERATION
        self.serial.write(cmd)

    def send_command(self,command):
        cmd = bytearray(self.tx_buffer_length)
        '''
        cmd[0],cmd[1] = self.split_int_2byte(round(command[0]*100))                #liquid_lens_freq
        # cmd[2],cmd[3]=self.split_int_2byte(round(command[1]*1000))               #liquid_lens_ampl
        # cmd[4],cmd[5]=self.split_int_2byte(round(command[2]*100))                #liquidLens_offset
        cmd[2] = int(command[1])                                                   # Focus-Tracking ON or OFF
        cmd[3] = int(command[2])                                                   #Homing
        cmd[4] = int(command[3])                                                   #tracking
        cmd[5],cmd[6] = self.split_signed_int_2byte(round(command[4]*100))         #Xerror
        cmd[7],cmd[8] = self.split_signed_int_2byte(round(command[5]*100))         #Yerror                           
        cmd[9],cmd[10] = self.split_signed_int_2byte(round(command[6]*100))        #Zerror
        cmd[11],cmd[12] = self.split_int_2byte(round(0))#command[9]*10))               #averageDt (millisecond with two digit after coma) BUG
        cmd[13] = int(command[8])                                               # LED intensity
        # Adding Trigger flag for other Video Streams (Boolean)
        # print('Trigger command sent {}'.format(command[9]))
        cmd[14] = int(command[9])
        # Adding Sampling Interval for other Video Streams
        # Minimum 10 ms (0.01 s) Maximum: 3600 s (1 hour)
        # Min value: 1 to 360000 
        # print('Interval command sent {}'.format(command[10]))
        cmd[15], cmd[16] = self.split_int_2byte(round(100*command[10]))
        '''
        self.serial.write(cmd)

    def read_received_packet(self):
        # wait to receive data
        while self.serial.in_waiting==0:
            pass
        while self.serial.in_waiting % self.rx_buffer_length != 0:
            pass

        num_bytes_in_rx_buffer = self.serial.in_waiting

        # get rid of old data
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            # print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))

        '''
        YfocusPhase = self.data2byte_to_int(data[0],data[1])*2*np.pi/65535.
        Xpos_arduino = data[3]*2**24 + data[4]*2**16+data[5]*2**8 + data[6]
        if data[2]==1:
            Xpos_arduino =-Xpos_arduino
        Ypos_arduino = data[8]*2**24 + data[9]*2**16+data[10]*2**8 + data[11]
        if data[7]==1:
            Ypos_arduino =-Ypos_arduino
        Zpos_arduino = data[13]*2**24 + data[14]*2**16+data[15]*2**8 + data[16]
        if data[12]==1:
            Zpos_arduino =-Zpos_arduino
        manualMode = data[17]
        LED_measured = self.data2byte_to_int(data[18], data[19])
        timeStamp = data[20]*2**24 + data[21]*2**16+data[22]*2**8 + data[23]
        tracking_triggered = bool(data[24])
        trigger_FL = bool(data[25])
        return [YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino, LED_measured, tracking_triggered],manualMode
        '''
        return data

    def read_received_packet_nowait(self):
        # wait to receive data
        if self.serial.in_waiting==0:
            return None
        if self.serial.in_waiting % self.rx_buffer_length != 0:
            return None
        
        # get rid of old data
        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            # print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))
        return data

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        pass

    def close(self):
        pass

    def toggle_LED(self,state):
        pass
    
    def toggle_laser(self,state):
        pass

    def move_x(self,delta):
        pass

    def move_y(self,delta):
        pass

    def move_x_nonblocking(self,delta):
        pass

    def move_y_nonblocking(self,delta):
        pass

    def move_z_nonblocking(self,delta):
        pass

    def move_theta_nonblocking(self,delta):
        pass
        
    def move_z(self,delta):
        pass

    def send_focus_tracking_command(self,focus_tracking_flag):
        pass

    def send_command(self,command):
        pass

    def read_received_packet(self):
        pass

    def read_received_packet_nowait(self):
        return None

    def set_number_of_planes_per_volume(self,value):
        pass

    def set_number_of_requested_volumes(self,value):
        pass

    def set_frequency_Hz(self,value):
        pass

    def set_phase_delay(self,value):
        pass

    def start_trigger_generation(self):
        pass

    def stop_trigger_generation(self):
        pass

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


