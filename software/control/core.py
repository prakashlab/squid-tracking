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
import control.utils_config as utils_config

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

    def set_crop(self,crop_width,height):
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

        self.fps_software_trigger = FPS['trigger_software']['default']
        self.timer_software_trigger_interval = (1/self.fps_software_trigger)*1000

        self.timer_software_trigger = QTimer()
        self.timer_software_trigger.setInterval(self.timer_software_trigger_interval)
        self.timer_software_trigger.timeout.connect(self.trigger_acquisition_software)

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
        if self.trigger_mode == TriggerMode.SOFTWARE:
            self._start_software_triggerred_acquisition()

    def stop_live(self):
        if self.is_live:
            self.is_live = False
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            self.camera.stop_streaming()
            self.turn_off_illumination()

    # software trigger related
    def trigger_acquisition_software(self):
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

    def _start_software_triggerred_acquisition(self):
        self.timer_software_trigger.start()

    def _set_software_trigger_fps(self,fps_software_trigger):
        self.fps_software_trigger = fps_software_trigger
        self.timer_software_trigger_interval = (1/self.fps_software_trigger)*1000
        self.timer_software_trigger.setInterval(self.timer_software_trigger_interval)

    def _stop_software_triggerred_acquisition(self):
        self.timer_software_trigger.stop()

    # trigger mode and settings
    def set_trigger_mode(self, mode):
        if mode == TriggerMode.SOFTWARE:
            self.camera.set_software_triggered_acquisition()
            if self.is_live:
                self._start_software_triggerred_acquisition()
        if mode == TriggerMode.HARDWARE:
            print('Setting camera to hardware trigger')
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            self.camera.set_hardware_triggered_acquisition()
        if mode == TriggerMode.CONTINUOUS: 
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            self.camera.set_continuous_acquisition()
        self.trigger_mode = mode

    def set_trigger_fps(self,fps):
        if self.trigger_mode == TriggerMode.SOFTWARE:
            self._set_software_trigger_fps(fps)
    
    # set microscope mode
    # @@@ to do: change softwareTriggerGenerator to TriggerGeneratror
    def set_microscope_mode(self,mode):
        print("setting microscope mode to " + mode)
        
        # temporarily stop live while changing mode
        if self.is_live is True:
            self.timer_software_trigger.stop()
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
            self.timer_software_trigger.start()

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
        if self.fps_software_trigger <= 5:
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

    def move_x_usteps(self,usteps):
        self.microcontroller.move_x_usteps(int(usteps))

    def move_y_usteps(self,usteps):
        self.microcontroller.move_y_usteps(int(usteps))

    def move_z_usteps(self,usteps):
        self.microcontroller.move_z_usteps(int(usteps))

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

class AutofocusWorker(QObject):

    finished = Signal()
    image_to_display = Signal(np.ndarray)
    # signal_current_configuration = Signal(Configuration)

    def __init__(self,autofocusController):
        QObject.__init__(self)
        self.autofocusController = autofocusController

        self.camera = self.autofocusController.camera
        self.microcontroller = self.autofocusController.navigationController.microcontroller
        self.navigationController = self.autofocusController.navigationController
        self.liveController = self.autofocusController.liveController

        self.N = self.autofocusController.N
        self.deltaZ = self.autofocusController.deltaZ
        self.deltaZ_usteps = self.autofocusController.deltaZ_usteps
        
        self.crop_width = self.autofocusController.crop_width
        self.crop_height = self.autofocusController.crop_height

    def run(self):
        self.run_autofocus()
        self.finished.emit()

    def wait_till_operation_is_completed(self):
        while self.microcontroller.is_busy():
            time.sleep(SLEEP_TIME_S)

    def run_autofocus(self):
        # @@@ to add: increase gain, decrease exposure time
        # @@@ can move the execution into a thread - done 08/21/2021
        focus_measure_vs_z = [0]*self.N
        focus_measure_max = 0

        z_af_offset_usteps = self.deltaZ_usteps*round(self.N/2)
        self.navigationController.move_z_usteps(-z_af_offset_usteps)
        self.wait_till_operation_is_completed()

        # maneuver for achiving uniform step size and repeatability when using open-loop control
        # can be moved to the firmware
        self.navigationController.move_z_usteps(-160)
        self.wait_till_operation_is_completed()
        self.navigationController.move_z_usteps(160)
        self.wait_till_operation_is_completed()

        steps_moved = 0
        for i in range(self.N):
            self.navigationController.move_z_usteps(self.deltaZ_usteps)
            self.wait_till_operation_is_completed()
            steps_moved = steps_moved + 1
            self.liveController.turn_on_illumination()
            self.wait_till_operation_is_completed()
            self.camera.send_trigger()
            image = self.camera.read_frame()
            self.liveController.turn_off_illumination()
            image = utils.crop_image(image,self.crop_width,self.crop_height)
            self.image_to_display.emit(image)
            QApplication.processEvents()
            timestamp_0 = time.time()
            focus_measure = utils.calculate_focus_measure(image)
            timestamp_1 = time.time()
            print('             calculating focus measure took ' + str(timestamp_1-timestamp_0) + ' second')
            focus_measure_vs_z[i] = focus_measure
            print(i,focus_measure)
            focus_measure_max = max(focus_measure, focus_measure_max)
            if focus_measure < focus_measure_max*AF.STOP_THRESHOLD:
                break

        # move to the starting location
        self.navigationController.move_z_usteps(-steps_moved*self.deltaZ_usteps)
        self.wait_till_operation_is_completed()

        # maneuver for achiving uniform step size and repeatability when using open-loop control
        self.navigationController.move_z_usteps(-160)
        self.wait_till_operation_is_completed()
        self.navigationController.move_z_usteps(160)
        self.wait_till_operation_is_completed()

        # determine the in-focus position
        idx_in_focus = focus_measure_vs_z.index(max(focus_measure_vs_z))

        # move to the calculated in-focus position
        self.navigationController.move_z_usteps(idx_in_focus*self.deltaZ_usteps)
        self.wait_till_operation_is_completed()
        if idx_in_focus == 0:
            print('moved to the bottom end of the AF range')
        if idx_in_focus == self.N-1:
            print('moved to the top end of the AF range')

class AutoFocusController(QObject):

    z_pos = Signal(float)
    autofocusFinished = Signal()
    image_to_display = Signal(np.ndarray)

    def __init__(self,camera,navigationController,liveController):
        QObject.__init__(self)
        self.camera = camera
        self.navigationController = navigationController
        self.liveController = liveController
        self.N = None
        self.deltaZ = None
        self.deltaZ_usteps = None
        self.crop_width = AF.CROP_WIDTH
        self.crop_height = AF.CROP_HEIGHT
        self.autofocus_in_progress = False

    def set_N(self,N):
        self.N = N

    def set_deltaZ(self,deltaZ_um):
        mm_per_ustep_Z = SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z)
        self.deltaZ = deltaZ_um/1000
        self.deltaZ_usteps = round((deltaZ_um/1000)/mm_per_ustep_Z)

    def set_crop(self,crop_width,height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def autofocus(self):
        # stop live
        if self.liveController.is_live:
            self.was_live_before_autofocus = True
            self.liveController.stop_live()
        else:
            self.was_live_before_autofocus = False

        # temporarily disable call back -> image does not go through streamHandler
        if self.camera.callback_is_enabled:
            self.callback_was_enabled_before_autofocus = True
            self.camera.stop_streaming()
            self.camera.disable_callback()
            self.camera.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        else:
            self.callback_was_enabled_before_autofocus = False

        self.autofocus_in_progress = True

        # create a QThread object
        try:
            if self.thread.isRunning():
                print('*** autofocus thread is still running ***')
                self.thread.terminate()
                self.thread.wait()
                print('*** autofocus threaded manually stopped ***')
        except:
            pass
        self.thread = QThread()
        # create a worker object
        self.autofocusWorker = AutofocusWorker(self)
        # move the worker to the thread
        self.autofocusWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.autofocusWorker.run)
        self.autofocusWorker.finished.connect(self._on_autofocus_completed)
        self.autofocusWorker.finished.connect(self.autofocusWorker.deleteLater)
        self.autofocusWorker.finished.connect(self.thread.quit)
        self.autofocusWorker.image_to_display.connect(self.slot_image_to_display)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()
        
    def _on_autofocus_completed(self):
        # re-enable callback
        if self.callback_was_enabled_before_autofocus:
            self.camera.stop_streaming()
            self.camera.enable_callback()
            self.camera.start_streaming()
        
        # re-enable live if it's previously on
        if self.was_live_before_autofocus:
            self.liveController.start_live()

        # emit the autofocus finished signal to enable the UI
        self.autofocusFinished.emit()
        QApplication.processEvents()
        print('autofocus finished')

        # update the state
        self.autofocus_in_progress = False

    def slot_image_to_display(self,image):
        #self.image_to_display.emit(image)
        pass

    def wait_till_autofocus_has_completed(self):
        while self.autofocus_in_progress == True:
            time.sleep(0.005)
        print('autofocus wait has completed, exit wait')

class MultiPointWorker(QObject):

    finished = Signal()
    image_to_display = Signal(np.ndarray)
    image_to_display_multi = Signal(np.ndarray,int)
    signal_current_configuration = Signal(Configuration)
    signal_register_current_fov = Signal(float,float)

    def __init__(self,multiPointController):
        QObject.__init__(self)
        self.multiPointController = multiPointController

        self.camera = self.multiPointController.camera
        self.microcontroller = self.multiPointController.microcontroller
        self.navigationController = self.multiPointController.navigationController
        self.liveController = self.multiPointController.liveController
        self.autofocusController = self.multiPointController.autofocusController
        self.configurationManager = self.multiPointController.configurationManager
        self.NX = self.multiPointController.NX
        self.NY = self.multiPointController.NY
        self.NZ = self.multiPointController.NZ
        self.Nt = self.multiPointController.Nt
        self.deltaX = self.multiPointController.deltaX
        self.deltaX_usteps = self.multiPointController.deltaX_usteps
        self.deltaY = self.multiPointController.deltaY
        self.deltaY_usteps = self.multiPointController.deltaY_usteps
        self.deltaZ = self.multiPointController.deltaZ
        self.deltaZ_usteps = self.multiPointController.deltaZ_usteps
        self.dt = self.multiPointController.deltat
        self.do_autofocus = self.multiPointController.do_autofocus
        self.crop_width = self.multiPointController.crop_width
        self.crop_height = self.multiPointController.crop_height
        self.display_resolution_scaling = self.multiPointController.display_resolution_scaling
        self.counter = self.multiPointController.counter
        self.experiment_ID = self.multiPointController.experiment_ID
        self.base_path = self.multiPointController.base_path
        self.selected_configurations = self.multiPointController.selected_configurations

        self.timestamp_acquisition_started = self.multiPointController.timestamp_acquisition_started
        self.time_point = 0

    def run(self):
        while self.time_point < self.Nt:
            # continous acquisition
            if self.dt == 0:
                self.run_single_time_point()
                if self.multiPointController.abort_acqusition_requested:
                    break
                self.time_point = self.time_point + 1
            # timed acquisition
            else:
                self.run_single_time_point()
                if self.multiPointController.abort_acqusition_requested:
                    break
                self.time_point = self.time_point + 1
                # check if the aquisition has taken longer than dt or integer multiples of dt, if so skip the next time point(s)
                while time.time() > self.timestamp_acquisition_started + self.time_point*self.dt:
                    print('skip time point ' + str(self.time_point+1))
                    self.time_point = self.time_point+1
                if self.time_point == self.Nt:
                    break # no waiting after taking the last time point
                # wait until it's time to do the next acquisition
                while time.time() < self.timestamp_acquisition_started + self.time_point*self.dt:
                    time.sleep(0.05)
        self.finished.emit()

    def wait_till_operation_is_completed(self):
        while self.microcontroller.is_busy():
            time.sleep(SLEEP_TIME_S)

    def run_single_time_point(self):

        # disable joystick button action
        self.navigationController.enable_joystick_button_action = False

        self.FOV_counter = 0
        print('multipoint acquisition - time point ' + str(self.time_point+1))
        
        # for each time point, create a new folder
        current_path = os.path.join(self.base_path,self.experiment_ID,str(self.time_point))
        os.mkdir(current_path)

        # create a dataframe to save coordinates
        coordinates_pd = pd.DataFrame(columns = ['i', 'j', 'k', 'x (mm)', 'y (mm)', 'z (um)'])

        x_scan_direction = 1
        dx_usteps = 0
        dy_usteps = 0
        dz_usteps = 0

        # along y
        for i in range(self.NY):

            self.FOV_counter = 0 # so that AF at the beginning of each new row

            # along x
            for j in range(self.NX):

                if (self.NZ > 1):
                    # maneuver for achiving uniform step size and repeatability when using open-loop control
                    self.navigationController.move_z_usteps(-160)
                    self.wait_till_operation_is_completed()
                    self.navigationController.move_z_usteps(160)
                    self.wait_till_operation_is_completed()
                    time.sleep(SCAN_STABILIZATION_TIME_MS_Z/1000)

                # z-stack
                for k in range(self.NZ):

                    # perform AF only if when not taking z stack
                    if (self.NZ == 1) and (self.do_autofocus) and (self.FOV_counter%Acquisition.NUMBER_OF_FOVS_PER_AF==0):
                    # temporary: replace the above line with the line below to AF every FOV
                    # if (self.NZ == 1) and (self.do_autofocus):
                        configuration_name_AF = MULTIPOINT_AUTOFOCUS_CHANNEL
                        config_AF = next((config for config in self.configurationManager.configurations if config.name == configuration_name_AF))
                        self.signal_current_configuration.emit(config_AF)
                        self.autofocusController.autofocus()
                        self.autofocusController.wait_till_autofocus_has_completed()

                    '''
                    # moved to before each z-stack on 12/29/2021
                    if (self.NZ > 1):
                        # maneuver for achiving uniform step size and repeatability when using open-loop control
                        self.navigationController.move_z_usteps(80)
                        self.wait_till_operation_is_completed()
                        self.navigationController.move_z_usteps(-80)
                        self.wait_till_operation_is_completed()
                        time.sleep(SCAN_STABILIZATION_TIME_MS_Z/1000)
                    '''
                    
                    file_ID = str(i) + '_' + str(j if x_scan_direction==1 else self.NX-1-j) + '_' + str(k)
                    # metadata = dict(x = self.navigationController.x_pos_mm, y = self.navigationController.y_pos_mm, z = self.navigationController.z_pos_mm)
                    # metadata = json.dumps(metadata)

                    # iterate through selected modes
                    for config in self.selected_configurations:
                        self.signal_current_configuration.emit(config)
                        self.wait_till_operation_is_completed()
                        self.liveController.turn_on_illumination()
                        self.wait_till_operation_is_completed()
                        self.camera.send_trigger() 
                        image = self.camera.read_frame()
                        self.liveController.turn_off_illumination()
                        image = utils.crop_image(image,self.crop_width,self.crop_height)
                        # self.image_to_display.emit(cv2.resize(image,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
                        image_to_display = utils.crop_image(image,round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling))
                        self.image_to_display.emit(image_to_display)
                        self.image_to_display_multi.emit(image_to_display,config.illumination_source)
                        saving_path = os.path.join(current_path, file_ID + '_' + str(config.name).replace(' ','_') + '.' + Acquisition.IMAGE_FORMAT)
                        if self.camera.is_color:
                            if 'BF LED matrix' in config.name:
                                if MULTIPOINT_BF_SAVING_OPTION == 'Raw':
                                    image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                                elif MULTIPOINT_BF_SAVING_OPTION == 'RGB2GRAY':
                                    image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
                                elif MULTIPOINT_BF_SAVING_OPTION == 'Green Channel Only':
                                    image = image[:,:,1]
                            else:
                                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                        cv2.imwrite(saving_path,image)
                        # tifffile.imsave(saving_path, image, description=metadata)
                        QApplication.processEvents()

                    # add the coordinate of the current location
                    coordinates_pd = coordinates_pd.append({'i':i,'j':j,'k':k,
                                                            'x (mm)':self.navigationController.x_pos_mm,
                                                            'y (mm)':self.navigationController.y_pos_mm,
                                                            'z (um)':self.navigationController.z_pos_mm*1000},
                                                            ignore_index = True)

                    # register the current fov in the navigationViewer 
                    self.signal_register_current_fov.emit(self.navigationController.x_pos_mm,self.navigationController.y_pos_mm)

                    # check if the acquisition should be aborted
                    if self.multiPointController.abort_acqusition_requested:
                        self.liveController.turn_off_illumination()
                        self.navigationController.move_x_usteps(-dx_usteps)
                        self.wait_till_operation_is_completed()
                        self.navigationController.move_y_usteps(-dy_usteps)
                        self.wait_till_operation_is_completed()
                        self.navigationController.move_z_usteps(-dz_usteps)
                        self.wait_till_operation_is_completed()
                        coordinates_pd.to_csv(os.path.join(current_path,'coordinates.csv'),index=False,header=True)
                        self.navigationController.enable_joystick_button_action = True
                        return

                    if self.NZ > 1:
                        # move z
                        if k < self.NZ - 1:
                            self.navigationController.move_z_usteps(self.deltaZ_usteps)
                            self.wait_till_operation_is_completed()
                            time.sleep(SCAN_STABILIZATION_TIME_MS_Z/1000)
                            dz_usteps = dz_usteps + self.deltaZ_usteps
                
                if self.NZ > 1:
                    # move z back
                    self.navigationController.move_z_usteps(-self.deltaZ_usteps*(self.NZ-1))
                    self.wait_till_operation_is_completed()
                    dz_usteps = dz_usteps - self.deltaZ_usteps*(self.NZ-1)

                # update FOV counter
                self.FOV_counter = self.FOV_counter + 1

                if self.NX > 1:
                    # move x
                    if j < self.NX - 1:
                        self.navigationController.move_x_usteps(x_scan_direction*self.deltaX_usteps)
                        self.wait_till_operation_is_completed()
                        time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)
                        dx_usteps = dx_usteps + x_scan_direction*self.deltaX_usteps

            '''
            # instead of move back, reverse scan direction (12/29/2021)
            if self.NX > 1:
                # move x back
                self.navigationController.move_x_usteps(-self.deltaX_usteps*(self.NX-1))
                self.wait_till_operation_is_completed()
                time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)
            '''
            x_scan_direction = -x_scan_direction

            if self.NY > 1:
                # move y
                if i < self.NY - 1:
                    self.navigationController.move_y_usteps(self.deltaY_usteps)
                    self.wait_till_operation_is_completed()
                    time.sleep(SCAN_STABILIZATION_TIME_MS_Y/1000)
                    dy_usteps = dy_usteps + self.deltaY_usteps

        if self.NY > 1:
            # move y back
            self.navigationController.move_y_usteps(-self.deltaY_usteps*(self.NY-1))
            self.wait_till_operation_is_completed()
            time.sleep(SCAN_STABILIZATION_TIME_MS_Y/1000)
            dy_usteps = dy_usteps - self.deltaY_usteps*(self.NY-1)

        # move x back at the end of the scan
        if x_scan_direction == -1:
            self.navigationController.move_x_usteps(-self.deltaX_usteps*(self.NX-1))
            self.wait_till_operation_is_completed()
            time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)

        coordinates_pd.to_csv(os.path.join(current_path,'coordinates.csv'),index=False,header=True)
        self.navigationController.enable_joystick_button_action = True

class MultiPointController(QObject):

    acquisitionFinished = Signal()
    image_to_display = Signal(np.ndarray)
    image_to_display_multi = Signal(np.ndarray,int)
    signal_current_configuration = Signal(Configuration)
    signal_register_current_fov = Signal(float,float)

    def __init__(self,camera,navigationController,liveController,autofocusController,configurationManager):
        QObject.__init__(self)

        self.camera = camera
        self.microcontroller = navigationController.microcontroller # to move to gui for transparency
        self.navigationController = navigationController
        self.liveController = liveController
        self.autofocusController = autofocusController
        self.configurationManager = configurationManager
        self.NX = 1
        self.NY = 1
        self.NZ = 1
        self.Nt = 1
        mm_per_ustep_X = SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X)
        mm_per_ustep_Y = SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y)
        mm_per_ustep_Z = SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z)
        self.deltaX = Acquisition.DX
        self.deltaX_usteps = round(self.deltaX/mm_per_ustep_X)
        self.deltaY = Acquisition.DY
        self.deltaY_usteps = round(self.deltaY/mm_per_ustep_Y)
        self.deltaZ = Acquisition.DZ/1000
        self.deltaZ_usteps = round(self.deltaZ/mm_per_ustep_Z)
        self.deltat = 0
        self.do_autofocus = False
        self.crop_width = Acquisition.CROP_WIDTH
        self.crop_height = Acquisition.CROP_HEIGHT
        self.display_resolution_scaling = Acquisition.IMAGE_DISPLAY_SCALING_FACTOR
        self.counter = 0
        self.experiment_ID = None
        self.base_path = None
        self.selected_configurations = []

    def set_NX(self,N):
        self.NX = N
    def set_NY(self,N):
        self.NY = N
    def set_NZ(self,N):
        self.NZ = N
    def set_Nt(self,N):
        self.Nt = N
    def set_deltaX(self,delta):
        mm_per_ustep_X = SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X)
        self.deltaX = delta
        self.deltaX_usteps = round(delta/mm_per_ustep_X)
    def set_deltaY(self,delta):
        mm_per_ustep_Y = SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y)
        self.deltaY = delta
        self.deltaY_usteps = round(delta/mm_per_ustep_Y)
    def set_deltaZ(self,delta_um):
        mm_per_ustep_Z = SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z)
        self.deltaZ = delta_um/1000
        self.deltaZ_usteps = round((delta_um/1000)/mm_per_ustep_Z)
    def set_deltat(self,delta):
        self.deltat = delta
    def set_af_flag(self,flag):
        self.do_autofocus = flag

    def set_crop(self,crop_width,height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_base_path(self,path):
        self.base_path = path

    def start_new_experiment(self,experiment_ID): # @@@ to do: change name to prepare_folder_for_new_experiment
        # generate unique experiment ID
        self.experiment_ID = experiment_ID.replace(' ','_') + '_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%-S.%f')
        self.recording_start_time = time.time()
        # create a new folder
        os.mkdir(os.path.join(self.base_path,self.experiment_ID))
        self.configurationManager.write_configuration(os.path.join(self.base_path,self.experiment_ID)+"/configurations.xml") # save the configuration for the experiment
        acquisition_parameters = {'dx(mm)':self.deltaX, 'Nx':self.NX, 'dy(mm)':self.deltaY, 'Ny':self.NY, 'dz(um)':self.deltaZ*1000,'Nz':self.NZ,'dt(s)':self.deltat,'Nt':self.Nt,'with AF':self.do_autofocus}
        f = open(os.path.join(self.base_path,self.experiment_ID)+"/acquisition parameters.json","w")
        f.write(json.dumps(acquisition_parameters))
        f.close()


    def set_selected_configurations(self, selected_configurations_name):
        self.selected_configurations = []
        for configuration_name in selected_configurations_name:
            self.selected_configurations.append(next((config for config in self.configurationManager.configurations if config.name == configuration_name)))
        
    def run_acquisition(self): # @@@ to do: change name to run_experiment
        print('start multipoint')
        print(str(self.Nt) + '_' + str(self.NX) + '_' + str(self.NY) + '_' + str(self.NZ))

        self.abort_acqusition_requested = False

        self.configuration_before_running_multipoint = self.liveController.currentConfiguration
        # stop live
        if self.liveController.is_live:
            self.liveController.was_live_before_multipoint = True
            self.liveController.stop_live() # @@@ to do: also uncheck the live button
        else:
            self.liveController.was_live_before_multipoint = False

        # disable callback
        if self.camera.callback_is_enabled:
            self.camera.callback_was_enabled_before_multipoint = True
            self.camera.stop_streaming()
            self.camera.disable_callback()
            self.camera.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        else:
            self.camera.callback_was_enabled_before_multipoint = False

        # run the acquisition
        self.timestamp_acquisition_started = time.time()
        # create a QThread object
        self.thread = QThread()
        # create a worker object
        self.multiPointWorker = MultiPointWorker(self)
        # move the worker to the thread
        self.multiPointWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.multiPointWorker.run)
        self.multiPointWorker.finished.connect(self._on_acquisition_completed)
        self.multiPointWorker.finished.connect(self.multiPointWorker.deleteLater)
        self.multiPointWorker.finished.connect(self.thread.quit)
        self.multiPointWorker.image_to_display.connect(self.slot_image_to_display)
        self.multiPointWorker.image_to_display_multi.connect(self.slot_image_to_display_multi)
        self.multiPointWorker.signal_current_configuration.connect(self.slot_current_configuration,type=Qt.BlockingQueuedConnection)
        self.multiPointWorker.signal_register_current_fov.connect(self.slot_register_current_fov)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()

    def _on_acquisition_completed(self):
        # restore the previous selected mode
        self.signal_current_configuration.emit(self.configuration_before_running_multipoint)

        # re-enable callback
        if self.camera.callback_was_enabled_before_multipoint:
            self.camera.stop_streaming()
            self.camera.enable_callback()
            self.camera.start_streaming()
            self.camera.callback_was_enabled_before_multipoint = False
        
        # re-enable live if it's previously on
        if self.liveController.was_live_before_multipoint:
            self.liveController.start_live()
        
        # emit the acquisition finished signal to enable the UI
        self.acquisitionFinished.emit()
        QApplication.processEvents()

    def request_abort_aquisition(self):
        self.abort_acqusition_requested = True

    def slot_image_to_display(self,image):
        self.image_to_display.emit(image)

    def slot_image_to_display_multi(self,image,illumination_source):
        self.image_to_display_multi.emit(image,illumination_source)

    def slot_current_configuration(self,configuration):
        self.signal_current_configuration.emit(configuration)

    def slot_register_current_fov(self,x_mm,y_mm):
        self.signal_register_current_fov.emit(x_mm,y_mm)

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

    def __init__(self, window_title='', DrawCrossHairs = False):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()

        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.view = self.graphics_widget.addViewBox()
        
        ## lock the aspect ratio so pixels are always square
        self.graphics_widget.view.setAspectLocked(True)
        
        ## Create image item
        self.graphics_widget.img = pg.ImageItem(border='w', axisOrder='row-major')
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

        layout = QGridLayout()
        layout.addWidget(self.graphics_widget, 0, 0) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

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

        self.graphics_widget.img.setImage(image, autoLevels=False)
        # print('In ImageDisplayWindow display image')
    
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

        self.roi_bbox.emit(np.array([xmin, ymin, width, height]))
        # print('Sent bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))
