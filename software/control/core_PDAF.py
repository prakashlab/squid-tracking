# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import control.utils as utils
from control._def import *
from control.core import *
import control.tracking as tracking

from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
import cv2
from datetime import datetime

import skimage # pip3 install -U scikit-image
import skimage.registration

class PDAFController(QObject):

    # input: stream from camera 1, stream from camera 2
    # input: from internal_states shared variables
    # output: amount of defocus, which may be read by or emitted to focusTrackingController (that manages focus tracking on/off, PID coefficients)

    signal_defocus_pixel_shift = Signal(float)
    signal_defocus_um_display = Signal(float)
    signal_defocus_um_tracking = Signal(float)
    signal_error = Signal(float)
    signal_image1 = Signal(np.ndarray)
    signal_image2 = Signal(np.ndarray)

    def __init__(self,tracking_controller_in_plane):
        QObject.__init__(self)
        self.registration_upsample_factor = 5
        self.image1_received = False
        self.image2_received = False
        self.locked = False
        self.tracking_controller_in_plane = tracking_controller_in_plane
        self.internal_state = self.tracking_controller_in_plane.internal_state # to do: add this directly to the input
        self.microcontroller = self.tracking_controller_in_plane.microcontroller # to do: add this directly to the input
        
        self.offset_x = PDAF.x_offset_default
        self.offset_y = PDAF.y_offset_default
        self.ROI_ratio_width = PDAF.ROI_ratio_width_default
        self.ROI_ratio_height = PDAF.ROI_ratio_height_default
        self.shift_to_distance_um = PDAF.shift_to_distance_um_default
        self.PDAF_calculation_enable = False
        self.PDAF_tracking_enable = False

        self.scale_factor = 1.0 # for keeping the image size for PDAF calculation below 512 x 512 (for now)

        self.defocus_um_for_enable_tracking_min = -10000
        self.defocus_um_for_enable_tracking_max = 10000

    def register_image_from_camera_1(self,image):
        if(self.locked==True):
            return
        self.image1 = np.copy(image)
        self.image1_received = True
        if(self.image2_received):
            self.calculate_defocus()

    def register_image_from_camera_2(self,image):
        if(self.locked==True):
            return
        self.image2 = np.copy(image)
        if PDAF_FLIPUD: 
            self.image2 = np.flipud(self.image2)
        else:
            self.image2 = np.fliplr(self.image2)
        self.image2_received = True
        if(self.image1_received):
            self.calculate_defocus()

    def calculate_defocus(self):
        if self.tracking_controller_in_plane.centroid is not None and self.tracking_controller_in_plane.objectFound is True: 
            self.locked = True
            # cropping parameters
            self.x = self.tracking_controller_in_plane.centroid[0]
            self.y = self.tracking_controller_in_plane.centroid[1]
            self.w = int(abs(self.tracking_controller_in_plane.rect_pts[0][0]-self.tracking_controller_in_plane.rect_pts[1][0])*self.ROI_ratio_width)
            self.h = int(abs(self.tracking_controller_in_plane.rect_pts[0][1]-self.tracking_controller_in_plane.rect_pts[1][1])*self.ROI_ratio_height)
            # crop
            self.image1 = self.image1[max((self.y-int(self.h/2)),0):min(self.image1.shape[0],(self.y+int(self.h/2))),max(0,(self.x-int(self.w/2))):min(self.image1.shape[1],(self.x+int(self.w/2)))]
            self.image2 = self.image2[max((self.y+self.offset_y-int(self.h/2)),0):min(self.image2.shape[0],(self.y+self.offset_y+int(self.h/2))),max(0,(self.x+self.offset_x-int(self.w/2))):min(self.image2.shape[1],(self.x+self.offset_x+int(self.w/2)))] 
            try:
                # resize
                if max(self.h,self.w) > 512:
                    self.scale_factor = 512.0/max(self.h,self.w)
                    # print('resize image ... ' + str(self.scale_factor))
                    self.image1 = cv2.resize(self.image1,(int(self.image1.shape[1]*self.scale_factor),int(self.image1.shape[0]*self.scale_factor)))
                    self.image2 = cv2.resize(self.image2,(int(self.image2.shape[1]*self.scale_factor),int(self.image2.shape[0]*self.scale_factor)))
                    # self.image1 = cv2.resize(self.image1,None,self.scale_factor,self.scale_factor)
                    # self.image2 = cv2.resize(self.image2,None,self.scale_factor,self.scale_factor)
                else:
                    self.scale_factor = 1.0
                # send cropped images to display
                self.signal_image1.emit(self.image1)
                self.signal_image2.emit(self.image2)
                # print(self.image1.shape)
                # print((self.h,self.w))
                # print('---')
                if self.image1.shape[0] > 0.9*self.h*self.scale_factor and self.image1.shape[1] > 0.9*self.w*self.scale_factor and self.image1.shape == self.image2.shape and self.PDAF_calculation_enable:
                    # calculate shift
                    shift, error = self._compute_shift_from_image_pair()
                    shift = shift/self.scale_factor
                    # save result
                    self.internal_state.data['PDAF_shift'] = shift
                    self.internal_state.data['PDAF_error'] = error 
                    # only output the defocus when calculation is reliable
                    if error < 0.5:
                        # self.signal_defocus_pixel_shift.emit(shift)
                        self.defocus_um = shift*self.shift_to_distance_um
                        self.signal_defocus_um_display.emit(self.defocus_um)
                        self.signal_error.emit(error)
                        # emit defocus for tracking
                        if self.PDAF_tracking_enable and ( self.defocus_um >= self.defocus_um_for_enable_tracking_min ) and ( self.defocus_um <= self.defocus_um_for_enable_tracking_max):
                            # self.signal_defocus_um_tracking.emit(self.defocus_um)
                            self.tracking_controller_in_plane.track_focus = True
                            self.tracking_controller_in_plane.focus_error = self.defocus_um/1000.0
                        else:
                            self.tracking_controller_in_plane.track_focus = False
                            self.tracking_controller_in_plane.focus_error = 0
            except:
                pass
            # get ready for the next calculation
            self.image1_received = False
            self.image2_received = False
            self.locked = False
        else:
            pass

    def _compute_shift_from_image_pair(self):
        # method 1: calculate 2D cross correlation -> find peak or centroid
        '''
        I1 = np.array(self.image1,dtype=np.int)
        I2 = np.array(self.image2,dtype=np.int)
        I1 = I1 - np.mean(I1)
        I2 = I2 - np.mean(I2)
        xcorr = cv2.filter2D(I1,cv2.CV_32F,I2)
        cv2.imshow('xcorr',np.array(255*xcorr/np.max(xcorr),dtype=np.uint8))
        cv2.waitKey(15)  
        '''
        # method 2: use skimage.registration.phase_cross_correlation
        shifts,error,phasediff = skimage.registration.phase_cross_correlation(self.image1,self.image2,upsample_factor=self.registration_upsample_factor,space='real')
        print('shift: ' + str(shifts) + ', error: ' + "{:.2f}".format(error) ) # for debugging
        if PDAF_FLIPUD:
            return shifts[1], error # shift[0] vs shift[1] depends on camera orientation
        else:
            return shifts[0], error # shift[0] vs shift[1] depends on camera orientation

    def set_x_offset(self,value):
        self.offset_x = value

    def set_y_offset(self,value):
        self.offset_y = value

    def set_ROI_ratio_width(self,value):
        self.ROI_ratio_width = value

    def set_ROI_ratio_height(self,value):
        self.ROI_ratio_height = value

    def set_shift_to_distance_um(self,value):
        self.shift_to_distance_um = value

    def enable_caculation(self,enabled):
        self.PDAF_calculation_enable = enabled
        print('PDAF calculation: ' + str(enabled))

    def enable_tracking(self,enabled):
        self.PDAF_tracking_enable = enabled
        print('PDAF tracking: ' + str(enabled))
        if enabled:
            self.internal_state.data['track_focus'] = True
            self.internal_state.data['track_focus_PDAF'] = True
            self.microcontroller.send_focus_tracking_command(True)
            # note that self.tracking_controller_in_plane.track_focus is set to True in the calculate_defocus section - only when error is smaller than a set threshold
        else:
            self.internal_state.data['track_focus'] = False
            self.internal_state.data['track_focus_PDAF'] = False
            self.microcontroller.send_focus_tracking_command(False)
            self.tracking_controller_in_plane.track_focus = False

    def set_defocus_um_for_enable_tracking_min(self,value):
        self.defocus_um_for_enable_tracking_min = value
        
    def set_defocus_um_for_enable_tracking_max(self,value):
        self.defocus_um_for_enable_tracking_max = value

    def close(self):
        pass

class TwoCamerasPDAFCalibrationController(QObject):

    acquisitionFinished = Signal()
    image_to_display_camera1 = Signal(np.ndarray)
    image_to_display_camera2 = Signal(np.ndarray)
    # signal_current_configuration = Signal(Configuration)

    z_pos = Signal(float)

    def __init__(self,camera1,camera2,navigationController,liveController1,liveController2,configurationManager=None):
        QObject.__init__(self)

        self.camera1 = camera1
        self.camera2 = camera2
        self.navigationController = navigationController
        self.liveController1 = liveController1
        self.liveController2 = liveController2
        self.configurationManager = configurationManager
        self.NZ = 1
        self.Nt = 1
        self.deltaZ = Acquisition.DZ/1000
        self.deltaZ_usteps = round((Acquisition.DZ/1000)*Motion.STEPS_PER_MM_Z)
        self.crop_width = Acquisition.CROP_WIDTH
        self.crop_height = Acquisition.CROP_HEIGHT
        self.display_resolution_scaling = Acquisition.IMAGE_DISPLAY_SCALING_FACTOR
        self.counter = 0
        self.experiment_ID = None
        self.base_path = None

    def set_NX(self,N):
        self.NX = N
    def set_NY(self,N):
        self.NY = N
    def set_NZ(self,N):
        self.NZ = N
    def set_Nt(self,N):
        self.Nt = N
    def set_deltaX(self,delta):
        self.deltaX = delta
        self.deltaX_usteps = round(delta*Motion.STEPS_PER_MM_XY)
    def set_deltaY(self,delta):
        self.deltaY = delta
        self.deltaY_usteps = round(delta*Motion.STEPS_PER_MM_XY)
    def set_deltaZ(self,delta_um):
        self.deltaZ = delta_um/1000
        self.deltaZ_usteps = round((delta_um/1000)*Motion.STEPS_PER_MM_Z)
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
        self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f')
        self.recording_start_time = time.time()
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path,self.experiment_ID))
            if self.configurationManager:
                self.configurationManager.write_configuration(os.path.join(self.base_path,self.experiment_ID)+"/configurations.xml") # save the configuration for the experiment
        except:
            pass

    def set_selected_configurations(self, selected_configurations_name):
        self.selected_configurations = []
        for configuration_name in selected_configurations_name:
            self.selected_configurations.append(next((config for config in self.configurationManager.configurations if config.name == configuration_name)))
        
    def run_acquisition(self): # @@@ to do: change name to run_experiment
        print('start multipoint')
        
        # stop live
        if self.liveController1.is_live:
            self.liveController1.was_live_before_multipoint = True
            self.liveController1.stop_live() # @@@ to do: also uncheck the live button
        else:
            self.liveController1.was_live_before_multipoint = False
        # stop live
        if self.liveController2.is_live:
            self.liveController2.was_live_before_multipoint = True
            self.liveController2.stop_live() # @@@ to do: also uncheck the live button
        else:
            self.liveController2.was_live_before_multipoint = False

        # disable callback
        if self.camera1.callback_is_enabled:
            self.camera1.callback_was_enabled_before_multipoint = True
            self.camera1.stop_streaming()
            self.camera1.disable_callback()
            self.camera1.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        else:
            self.camera1.callback_was_enabled_before_multipoint = False
        # disable callback
        if self.camera2.callback_is_enabled:
            self.camera2.callback_was_enabled_before_multipoint = True
            self.camera2.stop_streaming()
            self.camera2.disable_callback()
            self.camera2.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        else:
            self.camera2.callback_was_enabled_before_multipoint = False

        for self.time_point in range(self.Nt):
            self._run_multipoint_single()

        # re-enable callback
        if self.camera1.callback_was_enabled_before_multipoint:
            self.camera1.stop_streaming()
            self.camera1.enable_callback()
            self.camera1.start_streaming()
            self.camera1.callback_was_enabled_before_multipoint = False
        # re-enable callback
        if self.camera2.callback_was_enabled_before_multipoint:
            self.camera2.stop_streaming()
            self.camera2.enable_callback()
            self.camera2.start_streaming()
            self.camera2.callback_was_enabled_before_multipoint = False

        if self.liveController1.was_live_before_multipoint:
            self.liveController1.start_live()
        if self.liveController2.was_live_before_multipoint:
            self.liveController2.start_live()

        # emit acquisitionFinished signal
        self.acquisitionFinished.emit()
        QApplication.processEvents()

    def _run_multipoint_single(self):
        # for each time point, create a new folder
        current_path = os.path.join(self.base_path,self.experiment_ID,str(self.time_point))
        os.mkdir(current_path)
        
        # z-stack
        for k in range(self.NZ):
            file_ID = str(k)
            if self.configurationManager is not None:
                # iterate through selected modes
                for config in self.selected_configurations:
                    self.signal_current_configuration.emit(config)
                    self.camera1.send_trigger() 
                    image = self.camera1.read_frame()
                    image = utils.crop_image(image,self.crop_width,self.crop_height)
                    saving_path = os.path.join(current_path, 'camera1_' + file_ID + str(config.name) + '.' + Acquisition.IMAGE_FORMAT)
                    image_to_display = utils.crop_image(image,round(self.crop_width*self.liveController1.display_resolution_scaling), round(self.crop_height*self.liveController1.display_resolution_scaling))
                    self.image_to_display_camera1.emit(image_to_display)
                    if self.camera1.is_color:
                        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                    cv2.imwrite(saving_path,image)

                    self.camera2.send_trigger() 
                    image = self.camera2.read_frame()
                    image = utils.crop_image(image,self.crop_width,self.crop_height)
                    saving_path = os.path.join(current_path, 'camera2_' + file_ID + str(config.name) + '.' + Acquisition.IMAGE_FORMAT)
                    image_to_display = utils.crop_image(image,round(self.crop_width*self.liveController2.display_resolution_scaling), round(self.crop_height*self.liveController2.display_resolution_scaling))
                    self.image_to_display_camera2.emit(image_to_display)
                    if self.camera2.is_color:
                        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                    cv2.imwrite(saving_path,image)
                    QApplication.processEvents()
            else:
                self.camera1.send_trigger() 
                image = self.camera1.read_frame()
                image = utils.crop_image(image,self.crop_width,self.crop_height)
                saving_path = os.path.join(current_path, 'camera1_' + file_ID + '.' + Acquisition.IMAGE_FORMAT)
                image_to_display = utils.crop_image(image,round(self.crop_width*self.liveController1.display_resolution_scaling), round(self.crop_height*self.liveController1.display_resolution_scaling))
                self.image_to_display_camera1.emit(image_to_display)
                if self.camera1.is_color:
                    image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                cv2.imwrite(saving_path,image)

                self.camera2.send_trigger() 
                image = self.camera2.read_frame()
                image = utils.crop_image(image,self.crop_width,self.crop_height)
                saving_path = os.path.join(current_path, 'camera2_' + file_ID + '.' + Acquisition.IMAGE_FORMAT)
                image_to_display = utils.crop_image(image,round(self.crop_width*self.liveController2.display_resolution_scaling), round(self.crop_height*self.liveController2.display_resolution_scaling))
                self.image_to_display_camera2.emit(image_to_display)
                if self.camera2.is_color:
                    image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                cv2.imwrite(saving_path,image)
                QApplication.processEvents()
            # move z
            if k < self.NZ - 1:
                self.navigationController.move_z_usteps(self.deltaZ_usteps)
        
        # move z back
        self.navigationController.move_z_usteps(-self.deltaZ_usteps*(self.NZ-1))
