# set QT_API environment variable
import os, sys
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *
import control.tracking as tracking
import control.utils.image_processing as image_processing
import control.utils.pol2color as pol2color

from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
import cv2
import imutils
from datetime import datetime

from lxml import etree as ET
from pathlib import Path

import math
import json
import pandas as pd

class StreamHandler(QObject):
    ''' Signals 
    '''
    image_to_display = Signal(np.ndarray, str)
    thresh_image_to_display = Signal(np.ndarray)
    packet_image_to_write = Signal(np.ndarray, int, float)
    packet_image_for_tracking = Signal(np.ndarray, np.ndarray)
    signal_new_frame_received = Signal()
    signal_fps = Signal(int)
    signal_fps_display = Signal(float)
    signal_fps_save = Signal(str, float)
    signal_working_resolution = Signal(int)

    '''
    Signals
    image_to_display ->ImageDisplayer.enque
    packet_image_to_write ->ImageSaver
    packet_image_for_tracking -> Tracking_controller.on_new_frame
    signal_new_frame_received -> microcontroller_Receiver.get_Data

    Slots
    '''

    def __init__(self, camera = None , crop_width=2000,crop_height=2000, working_resolution_scaling = WORKING_RES_DEFAULT, imaging_channel = TRACKING, rotate_image_angle = 0, flip_image = None, is_polarization_camera = False ):
        QObject.__init__(self)
        self.fps_display = FPS['display']['default']
        self.fps_save = 1
        self.fps_track = 1
        self.timestamp_last_display = 0
        self.timestamp_last_save = 0
        self.timestamp_last_track = 0

        # @@@ This may cause issues if camera width/height is smaller than these values
        self.crop_width = crop_width
        self.crop_height = crop_height
        self.image_width = crop_width
        self.image_height = crop_height
        self.working_resolution_scaling = working_resolution_scaling

        self.rotate_image_angle = rotate_image_angle
        self.flip_image = flip_image
        self.camera = camera
        self.save_image_flag = False

        # If current image stream is used for tracking.
        self.imaging_channel = imaging_channel
        self.track_flag = False
        self.invert_image_flag = False
        self.handler_busy = False

        # for fps measurement
        self.timestamp_last = 0
        self.counter = 0
        self.fps_real = 0

        self.fps_display_real = 0
        self.counter_display = 0
        self.timestamp_last_display_real = 0

        self.fps_save_real = 0
        self.counter_save = 0

        self.is_polarization_camera = is_polarization_camera

        # Image thresholding parameters
        self.lower_HSV = np.array([0, 0, 100],dtype='uint8') 
        self.upper_HSV = np.array([255, 255, 255],dtype='uint8') 

    def start_recording(self):
        self.save_image_flag = True
        print('Starting Acquisition')

    def stop_recording(self):
        self.save_image_flag = False
        print('Stopping Acquisition')

    def start_tracking(self):
        self.track_flag = True

    def stop_tracking(self):
        self.track_flag = False

    def set_display_fps(self,fps):
        self.fps_display = fps
        #@@@Testing
        print(self.fps_display)

    def set_save_fps(self,fps):
        self.fps_save = fps
        print(self.fps_save)

    def set_crop(self,crop_width,crop_height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_working_resolution_scaling(self, working_resolution_scaling):
        self.working_resolution_scaling = working_resolution_scaling/100

    def set_image_thresholds(self, lower_HSV, upper_HSV):
        self.lower_HSV = lower_HSV
        self.upper_HSV = upper_HSV

        #@@@Testing
        # print('Updated color thresholds to {} and {}'.format(self.lower_HSV, self.upper_HSV))

    def update_invert_image_flag(self, flag):
        self.invert_image_flag = flag
        
    def threshold_image(self, image_resized, color):
        if(color):
            thresh_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2GRAY)
            image_resized = np.array(thresh_image, dtype='uint8')
            thresh_image = image_processing.threshold_image_gray(image_resized, self.lower_HSV[2], self.upper_HSV[2])
            # thresh_image = image_processing.threshold_image(image_resized,self.lower_HSV,self.upper_HSV)  #The threshold image as one channel
            if(self.invert_image_flag==True):
                thresh_image = 1 - thresh_image
        else:
            # print(self.lower_HSV[2])
            # print(self.upper_HSV[2])
            # img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            # print(max(image_resized))
            # print(min(image_resized))
            image_resized = np.array(image_resized, dtype='uint8')
            thresh_image = image_processing.threshold_image_gray(image_resized, self.lower_HSV[2], self.upper_HSV[2])
            if(self.invert_image_flag==True):
                thresh_image = 1 - thresh_image

        return thresh_image

    def get_real_stream_fps(self):
        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last:
            self.counter = self.counter+1
        else:
            self.timestamp_last = timestamp_now
            self.fps_real = self.counter
            self.counter = 0
            # print('real camera fps is ' + str(self.fps_real))
            self.signal_fps.emit(self.fps_real)

    def get_real_display_fps(self):
        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last_display_real:
            self.counter_display = self.counter_display+1
        else:
            self.timestamp_last_display_real = timestamp_now
            self.fps_display_real = self.counter_display
            self.counter_display = 0
            # print('real display fps is ' + str(self.fps_display_real))
            self.signal_fps_display.emit(self.fps_display_real)


    def on_new_frame(self, camera):

        # print('On new frame')
        camera.image_locked = True
        self.handler_busy = True
        self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()

        image = camera.current_frame

        # crop image
        image, self.image_width, self.image_height = image_processing.crop_image(image ,self.crop_width,self.crop_height)

        if(self.rotate_image_angle != 0):
            '''
                # ROTATE_90_CLOCKWISE
                # ROTATE_90_COUNTERCLOCKWISE
            '''
            if(self.rotate_image_angle == 90):
                image = cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
            elif(self.rotate_image_angle == -90):
                image = cv2.rotate(image,cv2.ROTATE_90_COUNTERCLOCKWISE)

        if(self.flip_image is not None):
            '''
                flipcode = 0: flip vertically
                flipcode > 0: flip horizontally
                flipcode < 0: flip vertically and horizontally
            '''
            if(self.flip_image == 'Vertical'):
                image = cv2.flip(image, 0)
            elif(self.flip_image == 'Horizontal'):
                image = cv2.flip(image, 1)
            elif(self.flip_image == 'Both'):
                image = cv2.flip(image, -1)
        
        self.get_real_stream_fps()

        # resize the image (convert to psuedocolor if the image is from a polarization camera)
        if self.is_polarization_camera:
            image_pol_pseudo_color = pol2color.pol2color(image)
            image_resized = imutils.resize(image_pol_pseudo_color, round(self.image_width*self.working_resolution_scaling))
        else:
            image_resized = imutils.resize(image, round(self.image_width*self.working_resolution_scaling))
        
        if(self.imaging_channel == TRACKING):
            # Threshold the image
            image_thresh = 255*np.array(self.threshold_image(image_resized, color = camera.is_color), dtype = 'uint8')
        
        # send image to track
        time_now = time.time() 
        if self.track_flag and self.imaging_channel == TRACKING:
            # track is a blocking operation - it needs to be
            self.packet_image_for_tracking.emit(image_resized, image_thresh)
            self.timestamp_last_track = time_now

        # send image to display
        time_now = time.time()
        if time_now - self.timestamp_last_display >= 1/self.fps_display:
            self.image_to_display.emit(image_resized, self.imaging_channel)
            if(self.imaging_channel == TRACKING):
                # Send thresholded image to display (only for tracking stream)
                self.thresh_image_to_display.emit(image_thresh)
                self.signal_working_resolution.emit(round(self.image_width*self.working_resolution_scaling))
            self.timestamp_last_display = time_now
            self.get_real_display_fps()
            
        # send image to write
        time_now = time.time()
        if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
            if camera.is_color:
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            self.packet_image_to_write.emit(image, camera.frame_ID, self.camera.timestamp)
            self.fps_save_real = round(1/(time_now - self.timestamp_last_save),1)
             # Send the real display FPS to the live Controller widget.
            self.signal_fps_save.emit(self.imaging_channel, self.fps_save_real)
            self.counter_save = 0
            self.timestamp_last_save = time_now
        else:
            self.counter_save += 1

        self.handler_busy = False
        camera.image_locked = False


class LiveController(QObject):

    def __init__(self,camera,microcontroller,control_illumination=False):
        QObject.__init__(self)
        self.camera = camera
        self.microcontroller = microcontroller
        self.microscope_mode = None
        self.trigger_mode = TriggerMode.SOFTWARE # @@@ change to None
        self.mode = None
        self.is_live = False
        self.was_live_before_autofocus = False
        self.was_live_before_multipoint = False
        self.control_illumination = control_illumination
        self.illumination_on = False

        self.fps_trigger = FPS['trigger_software']['default']
        self.timer_trigger_interval = (1/self.fps_trigger)*1000

        self.timer_trigger = QTimer()
        self.timer_trigger.setInterval(int(self.timer_trigger_interval))
        self.timer_trigger.timeout.connect(self.trigger_acquisition)

        self.trigger_ID = -1

        self.fps_real = 0
        self.counter = 0
        self.timestamp_last = 0

        self.exposure_time_bfdf_preset = None
        self.exposure_time_fl_preset = None
        self.exposure_time_fl_preview_preset = None
        self.analog_gain_bfdf_preset = None
        self.analog_gain_fl_preset = None
        self.analog_gain_fl_preview_preset = None

    # illumination control
    def turn_on_illumination(self):
        pass

    def turn_off_illumination(self):
        pass

    # illumination control
    def turn_on_illumination(self):
        self.microcontroller.turn_on_illumination()
        self.illumination_on = True

    def turn_off_illumination(self):
        self.microcontroller.turn_off_illumination()
        self.illumination_on = False

    def set_illumination(self,illumination_source,intensity):
        if illumination_source < 10: # LED matrix
            self.microcontroller.set_illumination_led_matrix(illumination_source,r=(intensity/100)*LED_MATRIX_R_FACTOR,g=(intensity/100)*LED_MATRIX_G_FACTOR,b=(intensity/100)*LED_MATRIX_B_FACTOR)
        else:
            self.microcontroller.set_illumination(illumination_source,intensity)

    def start_live(self):
        self.is_live = True
        self.camera.start_streaming()
        if self.trigger_mode == TriggerMode.SOFTWARE or self.trigger_mode == TriggerMode.HARDWARE:
            self._start_triggerred_acquisition()

    def stop_live(self):
        if self.is_live:
            self.is_live = False
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_triggerred_acquisition()
            if self.trigger_mode == TriggerMode.CONTINUOUS:
                self.camera.stop_streaming()
            if self.trigger_mode == TriggerMode.HARDWARE:
                self._stop_triggerred_acquisition()
            if self.control_illumination:
                self.turn_off_illumination()

    # software trigger related
    def trigger_acquisition(self):
        if self.trigger_mode == TriggerMode.SOFTWARE:
            if self.control_illumination and self.illumination_on == False:
                self.turn_on_illumination()
            self.trigger_ID = self.trigger_ID + 1
            self.camera.send_trigger()
            # measure real fps
            timestamp_now = round(time.time())
            if timestamp_now == self.timestamp_last:
                self.counter = self.counter+1
            else:
                self.timestamp_last = timestamp_now
                self.fps_real = self.counter
                self.counter = 0
                # print('real trigger fps is ' + str(self.fps_real))
        elif self.trigger_mode == TriggerMode.HARDWARE:
            self.trigger_ID = self.trigger_ID + 1
            self.microcontroller.send_hardware_trigger(control_illumination=True,illumination_on_time_us=self.camera.exposure_time*1000)

    def _start_triggerred_acquisition(self):
        self.timer_trigger.start()

    def _set_trigger_fps(self,fps_trigger):
        self.fps_trigger = fps_trigger
        self.timer_trigger_interval = (1/self.fps_trigger)*1000
        self.timer_trigger.setInterval(int(self.timer_trigger_interval))

    def _stop_triggerred_acquisition(self):
        self.timer_trigger.stop()

    # trigger mode and settings
    def set_trigger_mode(self, mode):
        if mode == TriggerMode.SOFTWARE:
            self.camera.set_software_triggered_acquisition()
            if self.is_live:
                self._start_triggerred_acquisition()
        if mode == TriggerMode.HARDWARE:
            print('Setting camera to hardware trigger')
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_triggerred_acquisition()
            # self.camera.reset_camera_acquisition_counter()
            self.camera.set_hardware_triggered_acquisition()
            self.microcontroller.set_strobe_delay_us(self.camera.strobe_delay_us)
        if mode == TriggerMode.CONTINUOUS: 
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_triggerred_acquisition()
            self.camera.set_continuous_acquisition()
        self.trigger_mode = mode

    def set_trigger_fps(self,fps):
        if self.trigger_mode == TriggerMode.SOFTWARE or self.trigger_mode == TriggerMode.HARDWARE:
            self._set_trigger_fps(fps)
    
    # set microscope mode
    # @@@ to do: change softwareTriggerGenerator to TriggerGeneratror
    def set_microscope_mode(self,mode):
        print("setting microscope mode to " + mode)
        
        # temporarily stop live while changing mode
        if self.is_live is True:
            self.timer_trigger.stop()
            self.turn_off_illumination()
        
        self.mode = mode
        if self.mode == MicroscopeMode.BFDF:
            self.camera.set_exposure_time(self.exposure_time_bfdf_preset)
            self.camera.set_analog_gain(self.analog_gain_bfdf_preset)
        elif self.mode == MicroscopeMode.FLUORESCENCE:
            self.camera.set_exposure_time(self.exposure_time_fl_preset)
            self.camera.set_analog_gain(self.analog_gain_fl_preset)
        elif self.mode == MicroscopeMode.FLUORESCENCE_PREVIEW:
            self.camera.set_exposure_time(self.exposure_time_fl_preview_preset)
            self.camera.set_analog_gain(self.analog_gain_fl_preview_preset)

        # restart live 
        if self.is_live is True:
            self.turn_on_illumination()
            self.timer_trigger.start()

    def get_trigger_mode(self):
        return self.trigger_mode

    def set_exposure_time_bfdf_preset(self,exposure_time):
        self.exposure_time_bfdf_preset = exposure_time
    def set_exposure_time_fl_preset(self,exposure_time):
        self.exposure_time_fl_preset = exposure_time
    def set_exposure_time_fl_preview_preset(self,exposure_time):
        self.exposure_time_fl_preview_preset = exposure_time
    def set_analog_gain_bfdf_preset(self,analog_gain):
        self.analog_gain_bfdf_preset = analog_gain
    def set_analog_gain_fl_preset(self,analog_gain):
        self.analog_gain_fl_preset = analog_gain
    def set_analog_gain_fl_preview_preset(self,analog_gain):
        self.analog_gain_fl_preview_preset = analog_gain

    # slot
    def on_new_frame(self):
        if self.fps_trigger <= 5:
            if self.control_illumination and self.illumination_on == True:
                self.turn_off_illumination()


class NavigationController(QObject):

    signal_x_mm = Signal(float)
    signal_y_mm = Signal(float)
    signal_z_mm = Signal(float)
    signal_theta_degree = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos_mm = 0
        self.y_pos_mm = 0
        self.z_pos_mm = 0
        self.theta_pos_rad = 0
        self.x_microstepping = MICROSTEPPING_DEFAULT_X
        self.y_microstepping = MICROSTEPPING_DEFAULT_Y
        self.z_microstepping = MICROSTEPPING_DEFAULT_Z
        self.theta_microstepping = MICROSTEPPING_DEFAULT_THETA
        self.enable_joystick_button_action = True

    def get_mm_per_ustep_X(self):
        return SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X)

    def get_mm_per_ustep_Y(self):
        return SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y)

    def get_mm_per_ustep_Z(self):
        return SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z)

    def move_x_usteps(self,usteps):
        self.microcontroller.move_x_usteps(int(usteps))

    def move_y_usteps(self,usteps):
        self.microcontroller.move_y_usteps(int(usteps))

    def move_z_usteps(self,usteps):
        self.microcontroller.move_z_usteps(int(usteps))

    def move_x(self,delta):
        self.microcontroller.move_x_usteps(int(delta/self.get_mm_per_ustep_X()))

    def move_y(self,delta):
        self.microcontroller.move_y_usteps(int(delta/self.get_mm_per_ustep_Y()))

    def move_z(self,delta):
        self.microcontroller.move_z_usteps(int(delta/self.get_mm_per_ustep_Z()))

    def move_x_to(self,delta):
        self.microcontroller.move_x_to_usteps(STAGE_MOVEMENT_SIGN_X*int(delta/self.get_mm_per_ustep_X()))

    def move_y_to(self,delta):
        self.microcontroller.move_y_to_usteps(STAGE_MOVEMENT_SIGN_Y*int(delta/self.get_mm_per_ustep_Y()))

    def move_z_to(self,delta):
        self.microcontroller.move_z_to_usteps(STAGE_MOVEMENT_SIGN_Z*int(delta/self.get_mm_per_ustep_Z()))

    def home_x(self):
        self.microcontroller.home_x()

    def home_y(self):
        self.microcontroller.home_y()

    def home_z(self):
        self.microcontroller.home_z()

    def home_theta(self):
        self.microcontroller.home_theta()

    def zero_x(self):
        self.microcontroller.zero_x()

    def zero_y(self):
        self.microcontroller.zero_y()

    def zero_z(self):
        self.microcontroller.zero_z()

    def zero_theta(self):
        self.microcontroller.zero_tehta()

    def home(self):
        pass

    def set_x_limit_pos_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_X > 0:
            self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,STAGE_MOVEMENT_SIGN_X*int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))

    def set_x_limit_neg_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_X > 0:
            self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,STAGE_MOVEMENT_SIGN_X*int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))

    def set_y_limit_pos_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Y > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,STAGE_MOVEMENT_SIGN_Y*int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))

    def set_y_limit_neg_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Y > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,STAGE_MOVEMENT_SIGN_Y*int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))

    def set_z_limit_pos_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Z > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,STAGE_MOVEMENT_SIGN_Z*int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))

    def set_z_limit_neg_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Z > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,STAGE_MOVEMENT_SIGN_Z*int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))
        

class SlidePositionControlWorker(QObject):
    
    finished = Signal()
    signal_stop_live = Signal()
    signal_resume_live = Signal()

    def __init__(self,slidePositionController,home_x_and_y_separately=False):
        QObject.__init__(self)
        self.slidePositionController = slidePositionController
        self.navigationController = slidePositionController.navigationController
        self.microcontroller = self.navigationController.microcontroller
        self.liveController = self.slidePositionController.liveController
        self.home_x_and_y_separately = home_x_and_y_separately

    def wait_till_operation_is_completed(self,timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S):
        while self.microcontroller.is_busy():
            time.sleep(SLEEP_TIME_S)
            if time.time() - timestamp_start > SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S:
                print('Error - slide position switching timeout, the program will exit')
                self.navigationController.move_x(0)
                self.navigationController.move_y(0)
                exit()

    def move_to_slide_loading_position(self):
        was_live = self.liveController.is_live
        if was_live:
            self.signal_stop_live.emit()
        if self.home_x_and_y_separately:
            timestamp_start = time.time()
            self.navigationController.home_x()
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.zero_x()
            self.navigationController.move_x(SLIDE_POSITION.LOADING_X_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.home_y()
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.zero_y()
            self.navigationController.move_y(SLIDE_POSITION.LOADING_Y_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
        else:
            timestamp_start = time.time()
            self.navigationController.home_xy()
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.zero_x()
            self.navigationController.zero_y()
            self.navigationController.move_x(SLIDE_POSITION.LOADING_X_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.move_y(SLIDE_POSITION.LOADING_Y_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
        if was_live:
            self.signal_resume_live.emit()
        self.slidePositionController.slide_loading_position_reached = True
        self.finished.emit()

    def move_to_slide_scanning_position(self):
        was_live = self.liveController.is_live
        if was_live:
            self.signal_stop_live.emit()
        if self.home_x_and_y_separately:
            timestamp_start = time.time()
            self.navigationController.home_y()
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.zero_y()
            self.navigationController.move_y(SLIDE_POSITION.SCANNING_Y_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.home_x()
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.zero_x()
            self.navigationController.move_x(SLIDE_POSITION.SCANNING_X_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
        else:
            timestamp_start = time.time()
            self.navigationController.home_xy()
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.zero_x()
            self.navigationController.zero_y()
            self.navigationController.move_y(SLIDE_POSITION.SCANNING_Y_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)            
            self.navigationController.move_x(SLIDE_POSITION.SCANNING_X_MM)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
        if was_live:
            self.signal_resume_live.emit()
        self.slidePositionController.slide_scanning_position_reached = True
        self.finished.emit()

class SlidePositionController(QObject):

    signal_slide_loading_position_reached = Signal()
    signal_slide_scanning_position_reached = Signal()
    signal_clear_slide = Signal()

    def __init__(self,navigationController,liveController):
        QObject.__init__(self)
        self.navigationController = navigationController
        self.liveController = liveController
        self.slide_loading_position_reached = False
        self.slide_scanning_position_reached = False

    def move_to_slide_loading_position(self):
        # create a QThread object
        self.thread = QThread()
        # create a worker object
        self.slidePositionControlWorker = SlidePositionControlWorker(self)
        # move the worker to the thread
        self.slidePositionControlWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.slidePositionControlWorker.move_to_slide_loading_position)
        self.slidePositionControlWorker.signal_stop_live.connect(self.slot_stop_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.signal_resume_live.connect(self.slot_resume_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.finished.connect(self.signal_slide_loading_position_reached.emit)
        self.slidePositionControlWorker.finished.connect(self.slidePositionControlWorker.deleteLater)
        self.slidePositionControlWorker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()

    def move_to_slide_scanning_position(self):
        # create a QThread object
        self.thread = QThread()
        # create a worker object
        self.slidePositionControlWorker = SlidePositionControlWorker(self)
        # move the worker to the thread
        self.slidePositionControlWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.slidePositionControlWorker.move_to_slide_scanning_position)
        self.slidePositionControlWorker.signal_stop_live.connect(self.slot_stop_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.signal_resume_live.connect(self.slot_resume_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.finished.connect(self.signal_slide_scanning_position_reached.emit)
        self.slidePositionControlWorker.finished.connect(self.slidePositionControlWorker.deleteLater)
        self.slidePositionControlWorker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()
        self.signal_clear_slide.emit()

    def slot_stop_live(self):
        self.liveController.stop_live()

    def slot_resume_live(self):
        self.liveController.start_live()



class ImageDisplay(QObject):

    image_to_display = Signal(np.ndarray, str)

    def __init__(self):
        QObject.__init__(self)
        self.queue = Queue(10) # max 10 items in the queue
        self.image_lock = Lock()
        self.stop_signal_received = False
        self.thread = Thread(target=self.process_queue)
        self.thread.start()        
        
    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image, frame_ID, timestamp, imaging_channel] = self.queue.get(timeout=0.1)
                self.image_lock.acquire(True)
                # Send image and imaging_channel
                self.image_to_display.emit(image, imaging_channel)
                self.image_lock.release()
                self.queue.task_done()
            except:
                # print("Exception:", sys.exc_info()[0])
                # print('Not sending image to display window')
                pass

    # def enqueue(self,image,frame_ID,timestamp):
    def enqueue(self,image, trackingStream = False):
        try:
            # print('In image display queue')
            self.queue.put_nowait([image, None, None, imaging_channel])
            # when using self.queue.put(str_) instead of try + nowait, program can be slowed down despite multithreading because of the block and the GIL
        except:
            pass
            # print('imageDisplay queue is full, image discarded')

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()

    # def __del__(self):
        # self.wait()

# from gravity machine
class ImageDisplayWindow(QMainWindow):

    roi_bbox = Signal(np.ndarray)

    def __init__(self, window_title='', DrawCrossHairs = False,  show_LUT=False, autoLevels=False):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()
        self.show_LUT = show_LUT
        self.autoLevels = autoLevels

        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.setMinimumSize(800, 600)
        self.graphics_widget.view = self.graphics_widget.addViewBox()
        
        ## lock the aspect ratio so pixels are always square
        self.graphics_widget.view.setAspectLocked(True)
        self.graphics_widget.view.invertY()
        
        ## Create image item
        if self.show_LUT:
            self.graphics_widget.view = pg.ImageView()
            self.graphics_widget.img = self.graphics_widget.view.getImageItem()
            self.graphics_widget.img.setBorder("w")
            self.graphics_widget.view.ui.roiBtn.hide()
            self.graphics_widget.view.ui.menuBtn.hide()
            self.LUTWidget = self.graphics_widget.view.getHistogramWidget()
        else:
            self.graphics_widget.img = pg.ImageItem(border="w")
            self.graphics_widget.view.addItem(self.graphics_widget.img)

        self.image_width = None
        self.image_height = None

        ## Create ROI
        self.roi_pos = (0.5,0.5)
        self.roi_size = (500,500)
        self.ROI = pg.ROI(self.roi_pos, self.roi_size, scaleSnap=True, translateSnap=True)
        self.ROI.setZValue(10)
        self.ROI.addScaleHandle((0,0), (1,1))
        self.ROI.addScaleHandle((1,1), (0,0))
        self.graphics_widget.view.addItem(self.ROI)
        self.ROI.hide()
        self.ROI.sigRegionChanged.connect(self.updateROI)
        self.roi_pos = self.ROI.pos()
        self.roi_size = self.ROI.size()

        ## Variables for annotating images
        self.DrawRect = False
        self.ptRect1 = None
        self.ptRect2 = None

        self.DrawCirc = False
        self.centroid = None

        self.DrawCrossHairs = DrawCrossHairs
        self.image_offset = np.array([0, 0])

        image_layout = QVBoxLayout()
        if self.show_LUT:
            image_layout.addWidget(self.graphics_widget.view)
            self.btn_toggle_auto_levels = QPushButton('Autolevel')
            self.btn_toggle_auto_levels.setCheckable(True)
            self.btn_toggle_auto_levels.setChecked(self.autoLevels)
            image_layout.addWidget(self.btn_toggle_auto_levels)
            self.btn_toggle_auto_levels.clicked.connect(self.toggle_auto_levels)
        else:
            image_layout.addWidget(self.graphics_widget)
        self.widget.setLayout(image_layout)
        self.setCentralWidget(self.widget)
        self.widget.setMinimumSize(800, 600)

    def display_image(self,image, imaging_channel = TRACKING):    
        image = np.copy(image) # Avoid overwriting the source image
        if(imaging_channel == TRACKING):
            self.image_height, self.image_width = image_processing.get_image_height_width(image)
            if(self.DrawRect):
                cv2.rectangle(image, self.ptRect1, self.ptRect2,(255,255,255) , 4) #cv2.rectangle(img, (20,20), (300,300),(0,0,255) , 2)#
                self.DrawRect=False

            if(self.DrawCirc):
                cv2.circle(image,(self.centroid[0],self.centroid[1]), 20, (255,0,0), 2)
                self.DrawCirc=False

            if(self.DrawCrossHairs):
                # Only need to do this if the image size changes
                self.update_image_center_width(image)
                self.draw_crosshairs()
                cv2.line(image, self.horLine_pt1, self.horLine_pt2, (255,255,255), thickness=3, lineType=8, shift=0) 
                cv2.line(image, self.verLine_pt1, self.verLine_pt2, (255,255,255), thickness=3, lineType=8, shift=0) 

        self.graphics_widget.img.setImage(image, autoLevels=self.autoLevels)
        # print('In ImageDisplayWindow display image')
    
    def toggle_auto_levels(self):
        self.autoLevels = self.btn_toggle_auto_levels.isChecked()

    def draw_rectangle(self, pts):
        # Connected to Signal from Tracking object
        self.DrawRect=True
        self.ptRect1=(pts[0][0],pts[0][1])
        self.ptRect2=(pts[1][0],pts[1][1])

    def draw_circle(self, centroid):
        # Connected to Signal from Tracking object
        self.DrawCirc=True
        self.centroid=(centroid[0],centroid[1])
        
    def draw_crosshairs(self):
        # Connected to Signal from Tracking object
        cross_length = round(self.image_width/20)

        self.horLine_pt1 = (int(self.tracking_center[0] - cross_length/2), int(self.tracking_center[1]))
        self.horLine_pt2 = (int(self.tracking_center[0] + cross_length/2), int(self.tracking_center[1]))

        self.verLine_pt1 = (int(self.tracking_center[0]), int(self.tracking_center[1] - cross_length/2))
        self.verLine_pt2 = (int(self.tracking_center[0]), int(self.tracking_center[1] + cross_length/2))

    def update_image_center_width(self,image):
        self.image_center, self.image_width = image_processing.get_image_center_width(image)
        self.tracking_center = self.image_center + self.image_offset

    def update_image_offset(self, new_image_offset):
        self.image_offset = new_image_offset
        print('ROI pos: {}'.format(self.roi_pos))
        print('ROI size: {}'.format(self.roi_size))

    def updateROI(self):
        self.roi_pos = self.ROI.pos()
        self.roi_size = self.ROI.size()

    def toggle_ROI_selector(self, flag):
        if(flag == True):
            self.ROI.show()
        else:
            self.ROI.hide()

    def send_bbox(self):
        self.updateROI()
        width = self.roi_size[0]
        height = self.roi_size[1]
        xmin = max(0, self.roi_pos[0])
        ymin = max(0, self.roi_pos[1])
        # print('Bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))

        self.roi_bbox.emit(np.array([int(xmin), int(ymin), int(width), int(height)]))
        # print('Sent bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))
