import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class TriggerController():
    def __init__(self,serial_number):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = MicrocontrollerDef.CMD_LENGTH
        self.rx_buffer_length = MicrocontrollerDef.MSG_LENGTH

        controller_ports = [ p.device for p in serial.tools.list_ports.comports() if serial_number == p.serial_number]
        if not controller_ports:
            raise IOError("No Controller Found")
        self.serial = serial.Serial(controller_ports[0],2000000)
        print('Teensy connected')

    def close(self):
        self.serial.close()

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
        print('start trigger generation')

    def stop_trigger_generation(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = STOP_TRIGGER_GENERATION
        self.serial.write(cmd)
        print('stop trigger generation')

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

class TriggerController_Simulation():
    def __init__(self,parent=None):
        pass

    def close(self):
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