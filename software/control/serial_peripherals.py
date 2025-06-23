import serial
from serial.tools import list_ports
import time
from typing import Tuple, Optional
import struct

class SerialDevice:
    """
    General wrapper for serial devices, with
    automating device finding based on VID/PID
    or serial number.
    """
    def __init__(self, port=None, VID=None,PID=None,SN=None, baudrate=9600, read_timeout=0.1, **kwargs):
        # Initialize the serial connection
        self.port = port
        self.VID = VID
        self.PID = PID
        self.SN = SN

        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.serial_kwargs = kwargs
        
        self.serial = None

        if VID is not None and PID is not None:
            for d in list_ports.comports():
                if d.vid == VID and d.pid == PID:
                    self.port = d.device
                    break
        if SN is not None:
            for d in list_ports.comports():
                if d.serial_number == SN:
                    self.port = d.device
                    break

        if self.port is not None:
            self.serial = serial.Serial(self.port, baudrate=baudrate, timeout=read_timeout, **kwargs)

    def open_ser(self, SN=None, VID=None, PID=None, baudrate =None, read_timeout=None,**kwargs):
        if self.serial is not None and not self.serial.is_open:
            self.serial.open()
        
        if SN is None:
            SN = self.SN

        if VID is None:
            VID = self.VID

        if PID is None:
            PID = self.PID

        if baudrate is None:
            baudrate = self.baudrate

        if read_timeout is None:
            read_timeout = self.read_timeout

        for k in self.serial_kwargs.keys():
            if k not in kwargs:
                kwargs[k] = self.serial_kwargs[k]

        if self.serial is None:
            if VID is not None and PID is not None:
                for d in list_ports.comports():
                    if d.vid == VID and d.pid == PID:
                        self.port = d.device
                        break
            if SN is not None:
                for d in list_ports.comports():
                    if d.serial_number == SN:
                        self.port = d.device
                        break
            if self.port is not None:
                self.serial = serial.Serial(self.port,**kwargs)

    def write_and_check(self, command, expected_response, read_delay=0.1, max_attempts=5, attempt_delay=1, check_prefix=True, print_response=False):
        # Write a command and check the response
        for attempt in range(max_attempts):
            self.serial.write(command.encode())
            time.sleep(read_delay)  # Wait for the command to be sent/executed

            response = self.serial.readline().decode().strip()
            if print_response:
                print(response)

            # flush the input buffer
            while self.serial.in_waiting:
                if print_response:
                    print(self.serial.readline().decode().strip())
                else:
                    self.serial.readline().decode().strip()

            # check response
            if response == expected_response:
                return response
            else:
                print(response)

            # check prefix if the full response does not match
            if check_prefix:
                if response.startswith(expected_response):
                    return response
            else:
                time.sleep(attempt_delay)  # Wait before retrying

        raise RuntimeError("Max attempts reached without receiving expected response.")

    def write_and_read(self, command, read_delay=0.1, max_attempts=3, attempt_delay=1):
        self.serial.write(command.encode())
        time.sleep(read_delay)  # Wait for the command to be sent
        response = self.serial.readline().decode().strip()
        return response

    def write(self, command):
        self.serial.write(command.encode())

    def close(self):
        # Close the serial connection
        self.serial.close()

class XLight_Simulation:
    def __init__(self):
        self.has_spinning_disk_motor = True
        self.has_spinning_disk_slider = True
        self.has_dichroic_filters_wheel = True
        self.has_emission_filters_wheel = True
        self.has_excitation_filters_wheel = True
        self.has_illumination_iris_diaphragm = True
        self.has_emission_iris_diaphragm = True
        self.has_dichroic_filter_slider = True
        self.has_ttl_control = True

        self.emission_wheel_pos = 1
        self.dichroic_wheel_pos = 1
        self.disk_motor_state = False
        self.spinning_disk_pos = 0

    def set_emission_filter(self,position, extraction=False, validate=False):
        self.emission_wheel_pos = position
        return position

    def get_emission_filter(self):
        return self.emission_wheel_pos

    def set_dichroic(self, position, extraction=False):
        self.dichroic_wheel_pos = position
        return position

    def get_dichroic(self):
        return self.dichroic_wheel_pos
    
    def set_disk_position(self, position):
        self.spinning_disk_pos = position
        return position

    def get_disk_position(self):
        return self.spinning_disk_pos

    def set_disk_motor_state(self, state):
        self.disk_motor_state = state
        return state

    def get_disk_motor_state(self):
        return self.disk_motor_state

    def set_illumination_iris(self,value):
        # value: 0 - 100
        self.illumination_iris = value
        return self.illumination_iris

    def set_emission_iris(self,value):
        # value: 0 - 100
        self.emission_iris = value
        return self.emission_iris

    def set_filter_slider(self,position):
        if str(position) not in ["0","1","2","3"]:
            raise ValueError("Invalid slider position!")
        self.slider_position = position
        return self.slider_position

# CrestOptics X-Light Port specs:
# 9600 baud
# 8 data bits
# 1 stop bit
# No parity
# no flow control

class XLight:
    """Wrapper for communicating with CrestOptics X-Light devices over serial"""
    def __init__(self, SN, sleep_time_for_wheel = 0.25, disable_emission_filter_wheel=True):
        """
        Provide serial number (default is that of the device
        cephla already has) for device-finding purposes. Otherwise, all
        XLight devices should use the same serial protocol
        """
        self.has_spinning_disk_motor = False
        self.has_spinning_disk_slider = False
        self.has_dichroic_filters_wheel = False
        self.has_emission_filters_wheel = False
        self.has_excitation_filters_wheel = False
        self.has_illumination_iris_diaphragm = False
        self.has_emission_iris_diaphragm = False
        self.has_dichroic_filter_slider = False
        self.has_ttl_control = False
        self.sleep_time_for_wheel = sleep_time_for_wheel

        self.disable_emission_filter_wheel = disable_emission_filter_wheel

        self.serial_connection = SerialDevice(SN=SN,baudrate=115200,
                bytesize=serial.EIGHTBITS,stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                xonxoff=False,rtscts=False,dsrdtr=False)
        self.serial_connection.open_ser()

        self.parse_idc_response(self.serial_connection.write_and_read("idc\r"))
        self.print_config()

    def parse_idc_response(self, response):
        # Convert hexadecimal response to integer
        config_value = int(response, 16)

        # Check each bit and set the corresponding variable
        self.has_spinning_disk_motor = bool(config_value & 0x00000001)
        self.has_spinning_disk_slider = bool(config_value & 0x00000002)
        self.has_dichroic_filters_wheel = bool(config_value & 0x00000004)
        self.has_emission_filters_wheel = bool(config_value & 0x00000008)
        self.has_excitation_filters_wheel = bool(config_value & 0x00000080)
        self.has_illumination_iris_diaphragm = bool(config_value & 0x00000200)
        self.has_emission_iris_diaphragm = bool(config_value & 0x00000400)
        self.has_dichroic_filter_slider = bool(config_value & 0x00000800)
        self.has_ttl_control = bool(config_value & 0x00001000)

    def print_config(self):
        print("Machine Configuration:")
        print(f"Spinning disk motor: {self.has_spinning_disk_motor}")
        print(f"Spinning disk slider: {self.has_spinning_disk_slider}")
        print(f"Dichroic filters wheel: {self.has_dichroic_filters_wheel}")
        print(f"Emission filters wheel: {self.has_emission_filters_wheel}")
        print(f"Excitation filters wheel: {self.has_excitation_filters_wheel}")
        print(f"Illumination Iris diaphragm: {self.has_illumination_iris_diaphragm}")
        print(f"Emission Iris diaphragm: {self.has_emission_iris_diaphragm}")
        print(f"Dichroic filter slider: {self.has_dichroic_filter_slider}")
        print(f"TTL control and combined commands subsystem: {self.has_ttl_control}")
    
    def set_emission_filter(self,position,extraction=False,validate=True):
        if self.disable_emission_filter_wheel:
            print('emission filter wheel disabled')
            return -1
        if str(position) not in ["1","2","3","4","5","6","7","8"]:
            raise ValueError("Invalid emission filter wheel position!")
        position_to_write = str(position)
        position_to_read = str(position)
        if extraction:
            position_to_write+="m"

        if validate:
            current_pos = self.serial_connection.write_and_check("B"+position_to_write+"\r","B"+position_to_read,read_delay=0.01)
            self.emission_wheel_pos = int(current_pos[1])
        else:
            self.serial_connection.write("B"+position_to_write+"\r")
            time.sleep(self.sleep_time_for_wheel)
            self.emission_wheel_pos = position

        return self.emission_wheel_pos

    def get_emission_filter(self):
        current_pos = self.serial_connection.write_and_check("rB\r","rB",read_delay=0.01)
        self.emission_wheel_pos = int(current_pos[2])
        return self.emission_wheel_pos

    def set_dichroic(self, position,extraction=False):
        if str(position) not in ["1","2","3","4","5"]:
            raise ValueError("Invalid dichroic wheel position!")
        position_to_write = str(position)
        position_to_read = str(position)
        if extraction:
            position_to_write+="m"

        current_pos = self.serial_connection.write_and_check("C"+position_to_write+"\r","C"+position_to_read,read_delay=0.01)
        self.dichroic_wheel_pos = int(current_pos[1])
        return self.dichroic_wheel_pos

    def get_dichroic(self):
        current_pos = self.serial_connection.write_and_check("rC\r","rC",read_delay=0.01)
        self.dichroic_wheel_pos = int(current_pos[2])
        return self.dichroic_wheel_pos

    def set_disk_position(self,position):
        if str(position) not in ["0","1","2","wide field","confocal"]:
            raise ValueError("Invalid disk position!")
        if position == "wide field":
            position = "0"

        if position == "confocal":
            position = "1'"

        position_to_write = str(position)
        position_to_read = str(position)

        current_pos = self.serial_connection.write_and_check("D"+position_to_write+"\r","D"+position_to_read,read_delay=5)
        self.spinning_disk_pos = int(current_pos[1])
        return self.spinning_disk_pos

    def set_illumination_iris(self,value):
        # value: 0 - 100
        self.illumination_iris = value
        value = str(int(10*value))
        self.serial_connection.write_and_check("J"+value+"\r","J"+value,read_delay=3)
        return self.illumination_iris

    def set_emission_iris(self,value):
        # value: 0 - 100
        self.emission_iris = value
        value = str(int(10*value))
        self.serial_connection.write_and_check("V"+value+"\r","V"+value,read_delay=3)
        return self.emission_iris

    def set_filter_slider(self,position):
        if str(position) not in ["0","1","2","3"]:
            raise ValueError("Invalid slider position!")
        self.slider_position = position
        position_to_write = str(position)
        position_to_read = str(position)
        self.serial_connection.write_and_check("P"+position_to_write+"\r","V"+position_to_read,read_delay=5)
        return self.slider_position

    def get_disk_position(self):
        current_pos = self.serial_connection.write_and_check("rD\r","rD",read_delay=0.01)
        self.spinning_disk_pos = int(current_pos[2])
        return self.spinning_disk_pos

    def set_disk_motor_state(self, state):
        """Set True for ON, False for OFF"""
        if state:
            state_to_write = "1"
        else:
            state_to_write = "0"

        current_pos = self.serial_connection.write_and_check("N"+state_to_write+"\r","N"+state_to_write,read_delay=2.5)

        self.disk_motor_state = bool(int(current_pos[1]))

    def get_disk_motor_state(self):
        """Return True for on, Off otherwise"""
        current_pos = self.serial_connection.write_and_check("rN\r","rN",read_delay=0.01)
        self.disk_motor_state = bool(int(current_pos[2]))
        return self.disk_motor_state

class LDI:
    """Wrapper for communicating with LDI over serial"""
    def __init__(self, SN="00000001"):
        """
        Provide serial number
        """
        self.serial_connection = SerialDevice(SN=SN,baudrate=9600,
                bytesize=serial.EIGHTBITS,stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE, 
                xonxoff=False,rtscts=False,dsrdtr=False)
        self.serial_connection.open_ser()
        self.intensity_mode = 'PC'
        self.shutter_mode = 'PC'
    
    def run(self):
        self.serial_connection.write_and_check("run!\r","ok")

    def set_shutter_mode(self,mode):
        if mode in ['EXT','PC']:
            self.serial_connection.write_and_check('SH_MODE='+mode+'\r',"ok")
            self.intensity_mode = mode

    def set_intensity_mode(self,mode):
        if mode in ['EXT','PC']:
            self.serial_connection.write_and_check('INT_MODE='+mode+'\r',"ok")
            self.intensity_mode = mode

    def set_intensity(self,channel,intensity):
        channel = str(channel)
        intensity = "{:.2f}".format(intensity)
        print('set:'+channel+'='+intensity+'\r')
        self.serial_connection.write_and_check('set:'+channel+'='+intensity+'\r',"ok")
        print('active channel: ' + str(self.active_channel))
    
    def set_shutter(self,channel,state):
        channel = str(channel)
        state = str(state)
        self.serial_connection.write_and_check('shutter:'+channel+'='+state+'\r',"ok")

    def get_shutter_state(self):
        self.serial_connection.write_and_check('shutter?\r','')

    def set_active_channel(self,channel):
        self.active_channel = channel
        print('[set active channel to ' + str(channel) + ']')

    def set_active_channel_shutter(self,state):
        channel = str(self.active_channel)
        state = str(state)
        print('shutter:'+channel+'='+state+'\r')
        self.serial_connection.write_and_check('shutter:'+channel+'='+state+'\r',"ok")

class LDI_Simulation:
    """Minimal simulation wrapper for LDI device functionality"""
    def __init__(self):
        self.intensity_mode = 'PC'
        self.shutter_mode = 'PC'
        self.active_channel = None
        self.channel_intensities = {i: 0.0 for i in range(1, 16)}
        self.channel_shutters = {i: 0 for i in range(1, 16)}
    
    def run(self):
        pass

    def set_shutter_mode(self, mode):
        if mode in ['EXT', 'PC']:
            self.shutter_mode = mode

    def set_intensity_mode(self, mode):
        if mode in ['EXT', 'PC']:
            self.intensity_mode = mode

    def set_intensity(self, channel, intensity):
        self.channel_intensities[int(channel)] = float(intensity)
    
    def set_shutter(self, channel, state):
        self.channel_shutters[int(channel)] = int(state)

    def get_shutter_state(self):
        return self.channel_shutters

    def set_active_channel(self, channel):
        self.active_channel = int(channel)

    def set_active_channel_shutter(self, state):
        if self.active_channel is not None:
            self.channel_shutters[self.active_channel] = int(state)