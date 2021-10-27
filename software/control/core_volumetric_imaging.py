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

from lxml import etree as ET
from pathlib import Path

import tifffile as tif

# case 1: no scanning, high speed only - set current_min and current_max to be the same
# case 2: with liquid lens scanning

class VolumetricImagingController(QObject):

    signal_volumetric_imaging_stopped = Signal()
    signal_trigger_mode = Signal(str)

    def __init__(self,camera,trigger_controller,liquid_lens,volumetricImagingStreamHandler,volumetricImagingImageSaver,internal_state):
        QObject.__init__(self)
        self.camera = camera
        self.volumetricImagingStreamHandler = volumetricImagingStreamHandler
        self.volumetricImagingImageSaver = volumetricImagingImageSaver
        self.internal_state = internal_state
        self.trigger_controller = trigger_controller
        self.liquid_lens = liquid_lens
        
        self.current_mA_min = 0
        self.current_mA_max = 0
        self.frequency_Hz = 1 # 0.1,0.2,0.5,1,2,5,10,20,25
        self.phase_delay = 0 # 0-90 degree
        self.number_of_planes_per_volume = VOLUMETRIC_IMAGING_NUMBER_OF_PLANES_PER_VOLUME_DEFAULT
        self.number_of_requested_volumes = 0 # 0 means infinite
        self.number_of_requested_frames = 0 # 0 means infinite

        self.current_mA_static = 0
        
        self.flag_record_acquisition = False
        self.flag_volumetric_imaging_started = True # including zero amplitude

        # make connections
        self.volumetricImagingStreamHandler.signal_volumetric_imaging_completed.connect(self.slot_volumetric_imaging_completed)
        
    def start_volumetric_imaging(self):
        # set camera to hardware trigger if liquid lens scanning frequency is non-zero
        if self.frequency_Hz > 0:
            # self.camera.set_hardware_triggered_acquisition()
            # to do: emit a signal to update the widget
            self.signal_trigger_mode.emit(TriggerMode.HARDWARE)
        else:
            # self.camera.set_continuous_acquisition()
            # to do: emit a signal to update the widget
            self.signal_trigger_mode.emit(TriggerMode.CONTINUOUS)
        
        # calculate number of requested frames
        self.number_of_requested_frames = self.number_of_planes_per_volume * self.number_of_requested_volumes

        # update the StreamHandler
        self.volumetricImagingStreamHandler.flag_volumetric_imaging_started = True # used for detecting the first frame after hardware trigger
        self.volumetricImagingStreamHandler.number_of_requested_frames = self.number_of_requested_frames
        if self.flag_record_acquisition:
            self.volumetricImagingImageSaver.start_a_new_folder()
            self.volumetricImagingStreamHandler.start_recording()

        # start liquid lens
        if self.frequency_Hz > 0:
            self.liquid_lens.start_scanning(self.current_mA_min,self.current_mA_max,self.frequency_Hz)

        # set up the trigger controller
        self.trigger_controller.set_number_of_planes_per_volume(self.number_of_planes_per_volume)
        self.trigger_controller.set_number_of_requested_volumes(self.number_of_requested_volumes)
        self.trigger_controller.set_frequency_Hz(self.frequency_Hz)
        self.trigger_controller.set_phase_delay(self.phase_delay)
        self.trigger_controller.start_trigger_generation()

        # update the flag
        self.flag_volumetric_imaging_started = True

    def stop_volumetric_imaging(self):
        self.volumetricImagingStreamHandler.stop_recording()
        self.volumetricImagingStreamHandler.reset()
        self.trigger_controller.stop_trigger_generation()
        # self.camera.set_continuous_acquisition()
        self.signal_trigger_mode.emit(TriggerMode.CONTINUOUS)
        self.liquid_lens.set_current_mA(self.current_mA_static)
        self.flag_volumetric_imaging_started= False

    def slot_volumetric_imaging_completed(self):
        self.flag_volumetric_imaging_started = False
        # self.camera.set_continuous_acquisition()
        self.signal_trigger_mode.emit(TriggerMode.CONTINUOUS)
        self.liquid_lens.set_current_mA(self.current_mA_static)
        self.signal_volumetric_imaging_stopped.emit()

    def set_liquid_lens_scanning_current_min(self,value):
        print('set current min to ' + str(value))
        self.current_mA_min = value

    def set_liquid_lens_scanning_current_max(self,value):
        print('set current max to ' + str(value))
        self.current_mA_max = value

    def set_liquid_lens_scanning_frequency(self,value):
        print('set frequency_Hz to ' + str(value))
        self.frequency_Hz = value

    def set_number_of_planes_per_volume(self,value):
        print('set number of planes per volume to ' + str(value))
        self.number_of_planes_per_volume = value
        self.volumetricImagingStreamHandler.number_of_planes_per_volume = value

    def set_number_of_requested_volumes(self,value):
        print('set number of volumes to ' + str(value))
        self.number_of_requested_volumes = value

    def set_liquid_lens_current(self,value=None):
        if self.flag_volumetric_imaging_started:
            print('change to current mode - stop sweep')
            self.stop_volumetric_imaging()
            self.signal_volumetric_imaging_stopped.emit()
        if value is not None:
            self.current_mA_static = value
        self.liquid_lens.set_current_mA(self.current_mA_static)

    def set_flag_record_acquisition(self,value):
        self.flag_record_acquisition = value

    def start_recording(self):
        self.flag_record_acquisition = True
        self.volumetricImagingImageSaver.start_a_new_folder()
        self.volumetricImagingStreamHandler.start_recording()

    def stop_recording(self):
        self.flag_record_acquisition = False
        self.volumetricImagingStreamHandler.stop_recording()

    def set_phase_delay(self,phase_delay):
        print('update phase delay to ' + str(phase_delay) + ' degree')
        self.phase_delay = phase_delay
        self.trigger_controller.set_phase_delay(phase_delay)

    def enable_focus_measure_calculation(self,enabled):
        self.volumetricImagingStreamHandler.flag_calculate_focus_measure = enabled

    def enable_focus_tracking(self,enabled):
        self.volumetricImagingStreamHandler.flag_focus_tracking = enabled

    def close(self):
        pass

class VolumetricImagingStreamHandler(QObject):

    image_to_display = Signal(np.ndarray)
    packet_image_for_array_display = Signal(np.ndarray, int)
    # packet_image_to_write = Signal(np.ndarray, int, int, float)
    # packet_image_stack_to_write = Signal(np.ndarray, int)
    packet_image_to_write = Signal(np.ndarray, int)
    packet_image_stack_to_display = Signal(np.ndarray)
    signal_new_frame_received = Signal()
    signal_volumetric_imaging_completed = Signal() 
    signal_focus_measure_plot = Signal(np.ndarray,np.ndarray)
    signal_defocus = Signal(float)

    # not used - for compatability with standard stream handler
    signal_fps = Signal(int)
    signal_fps_display = Signal(float)
    signal_fps_save = Signal(str, float)

    def __init__(self, tracking_controller, crop_width=Acquisition.CROP_WIDTH, crop_height=Acquisition.CROP_HEIGHT, display_resolution_scaling=1, imaging_channel = 'volumetric imaging', rotate_image_angle = 0, flip_image = None):
        QObject.__init__(self)

        self.rotate_image_angle = rotate_image_angle
        self.flip_image = flip_image
        self.imaging_channel = imaging_channel

        self.flag_volumetric_imaging_started = False
        self.flag_save_images = False
        self.flag_first_image = True
        
        self.frame_ID_offset = None
        self.frame_ID = None
        self.plane_ID = None
        
        self.number_of_planes_per_volume = VOLUMETRIC_IMAGING_NUMBER_OF_PLANES_PER_VOLUME_DEFAULT
        self.number_of_requested_frames = None

        self.flag_calculate_focus_measure = False
        self.flag_focus_tracking = False

        self.fps_display = 30
        self.timestamp_last_display = 0
        self.timestamp_last_array_display = 0

        self.crop_width = crop_width
        self.crop_height = crop_height
        self.display_resolution_scaling = display_resolution_scaling
        
        self.handler_busy = False

        # for fps measurement
        self.timestamp_last = 0
        self.counter = 0
        self.fps_real = 0

        # for focus tracking
        self.tracking_controller = tracking_controller

    def set_display_fps(self,fps):
        self.fps_display = fps

    def set_crop(self,crop_width,height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_display_resolution_scaling(self, display_resolution_scaling):
        self.display_resolution_scaling = display_resolution_scaling/100
        print(self.display_resolution_scaling)

    def reset(self):
        self.flag_first_image = True
        self.frame_ID_offset = None
        self.flag_volumetric_imaging_started = False

    def on_new_frame(self, camera):

        camera.image_locked = True
        self.handler_busy = True
        self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()

        # crop image
        image_cropped,image_width,image_height = utils.image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
        image_cropped = np.squeeze(image_cropped)

        # flip image
        if(self.rotate_image_angle != 0):
            if(self.rotate_image_angle == 90):
                image_cropped = cv2.rotate(image_cropped,cv2.ROTATE_90_CLOCKWISE)
            elif(self.rotate_image_angle == -90):
                image_cropped = cv2.rotate(image_cropped,cv2.ROTATE_90_COUNTERCLOCKWISE)
        if(self.flip_image is not None):
            if(self.flip_image == 'Vertical'):
                image_cropped = cv2.flip(image_cropped, 0)
            elif(self.flip_image == 'Horizontal'):
                image_cropped = cv2.flip(image_cropped, 1)
            elif(self.flip_image == 'Both'):
                image_cropped = cv2.flip(image_cropped, -1)

        # set flag and frame number offset for volumetric imaging
        if self.flag_volumetric_imaging_started and self.flag_first_image:
            self.flag_first_image = False
            self.frame_ID_offset = camera.frame_ID
            self.focus_measure_num_points = min(int(np.ceil(self.number_of_planes_per_volume/2))+1,self.number_of_planes_per_volume)
            self.focus_measure_index = np.linspace(-1,1,self.focus_measure_num_points)
            self.focus_measure = np.zeros(self.focus_measure_num_points)
            self.image_stack = np.empty([self.number_of_planes_per_volume,image_height,image_width]).astype(np.uint8)

        # set frame ID - when self.frame_ID_offset is None, it means volumetric imaging has stopped
        if self.frame_ID_offset is not None:
            self.frame_ID = camera.frame_ID - self.frame_ID_offset
            self.plane_ID = self.frame_ID%self.number_of_planes_per_volume
            self.image_stack[self.plane_ID,:,:] = image_cropped
        else:
            self.frame_ID = camera.frame_ID

        # print(self.frame_ID)

        # calculate focus measure when volumetric imaging is enabled
        if self.flag_calculate_focus_measure and self.frame_ID_offset is not None:
            if self.plane_ID < self.focus_measure_num_points: 
                # calculate focus measure
                self.focus_measure[self.plane_ID] = image_processing.calculate_focus_measure(image_cropped)
            if self.plane_ID == self.focus_measure_num_points-1:
                self.signal_focus_measure_plot.emit(self.focus_measure_index,self.focus_measure)
                # calculate defocus
                self.defocus = np.sum(np.multiply(self.focus_measure_index,self.focus_measure))/np.sum(self.focus_measure)
                self.signal_defocus.emit(self.defocus)
                print('defocus: ' + str(self.defocus) + ' sum of focus measure: ' + str(np.sum(self.focus_measure)) )
                # focus tracking
                if self.flag_focus_tracking:
                    self.tracking_controller.track_focus = True
                    self.tracking_controller.focus_error = self.defocus*(-1)
                    # to do: add an adjustable scaling factor - this scaling factor depends on the liquid lens scan amplitude and objective being used
                else:
                    self.tracking_controller.track_focus = False
                    self.tracking_controller.focus_error = 0

        # check if camera has registered requested number of frames
        if self.flag_volumetric_imaging_started and self.number_of_requested_frames!=0 and self.frame_ID>=(self.number_of_requested_frames-1):
            self.reset()
            self.signal_volumetric_imaging_completed.emit()

        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last:
            self.counter = self.counter+1
        else:
            self.timestamp_last = timestamp_now
            self.fps_real = self.counter
            self.counter = 0
            self.signal_fps.emit(self.fps_real)

        # send image to display
        time_now = time.time()
        if time_now-self.timestamp_last_display >= 1/self.fps_display:
            # self.image_to_display.emit(cv2.resize(image_cropped,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
            self.image_to_display.emit(image_cropped)
            self.timestamp_last_display = time_now

        # send image to array display
        # time_now = time.time()
        # if time_now-self.timestamp_last_array_display >= 1/5:
            # self.packet_image_for_array_display.emit(image_cropped,self.plane_ID)
            # self.timestamp_last_array_display = time_now
        self.packet_image_for_array_display.emit(image_cropped,self.plane_ID)

        '''
        # send image to write
        if self.flag_save_images:
            if camera.is_color:
                image_cropped = cv2.cvtColor(image_cropped,cv2.COLOR_RGB2BGR)
            self.packet_image_to_write.emit(image_cropped,camera.frame_ID,self.plane_ID,camera.timestamp)
        '''

        # send the image stack to display/save
        if self.flag_volumetric_imaging_started and self.plane_ID == self.number_of_planes_per_volume-1:
            # save z-stack
            if self.flag_save_images:
                self.packet_image_to_write.emit(self.image_stack,self.frame_ID)
            self.packet_image_stack_to_display.emit(self.image_stack)

        self.handler_busy = False
        camera.image_locked = False

    def start_recording(self):
        self.flag_save_images = True

    def stop_recording(self):
        self.flag_save_images = False

    def set_save_fps(self,fps):
        pass

class VolumetricImagingImageSaver(QObject):

    stop_recording = Signal()
    imageName = Signal(str, str)

    def __init__(self,internal_state,image_format='tif'):
        QObject.__init__(self)
        self.internal_state = internal_state
        self.image_format = image_format
        self.base_path = './'
        self.experiment_ID = ''
        self.max_num_image_per_folder = 1000
        self.queue = Queue(200) # max 200 items in the queue
        self.image_lock = Lock()
        self.stop_signal_received = False
        self.thread = Thread(target=self.process_queue)
        self.thread.start()
        self.counter = 0
        self.liquid_lens_settings = ''

    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            if self.queue.empty() == False:
                try:
                    [image,frame_ID] = self.queue.get(timeout=0.1)
                    self.image_lock.acquire(True)
                    folder_ID = int(self.counter/self.max_num_image_per_folder)
                    file_ID = int(self.counter%self.max_num_image_per_folder)
                    # create a new folder
                    if file_ID == 0:
                        os.mkdir(os.path.join(self.base_path,self.experiment_ID,str(folder_ID)))
                    saving_path = os.path.join(self.base_path,self.experiment_ID,str(folder_ID),str(file_ID) + '_' + str(frame_ID) + '.tif')
                    tif.imwrite(saving_path, image, imagej=True)
                    # cv2.imwrite(saving_path,image)
                    self.counter = self.counter + 1
                    self.queue.task_done()
                    self.image_lock.release()
                except:
                    print('error occurred during processing image saving')
                    pass
            else:
                time.sleep(0.001)
                            
    def enqueue(self,image,frame_ID):
        try:
            self.queue.put_nowait([image,frame_ID])
            # when using self.queue.put(str_), program can be slowed down despite multithreading because of the block and the GIL
        except:
            print('imageSaver queue is full, image discarded')

    def set_base_path(self,path):
        self.base_path = path

    def start_a_new_folder(self,experiment_ID=''):
        # generate unique experiment ID
        if 'experiment_ID' in self.internal_state.data.keys():
            self.experiment_ID = self.internal_state.data['experiment_ID'] + '_3D[' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ']' 
        else:
            self.internal_state.data['experiment_ID'] = self.experiment_ID
            self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + '_3D'
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path,self.experiment_ID))
        except:
            print('making a new folder failed')
            pass
        # reset the counter
        self.counter = 0

    # not used - for compatability with standard stream handler
    def set_recording_time_limit(self,time_limit):
        self.recording_time_limit = time_limit

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()


class ImageArrayDisplayWindow(QMainWindow):

    def __init__(self, window_title=''):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()

        # interpret image data as row-major instead of col-major
        pg.setConfigOptions(imageAxisOrder='row-major')

        self.sub_windows = []
        for i in range(9):
            self.sub_windows.append(pg.GraphicsLayoutWidget())
            self.sub_windows[i].view = self.sub_windows[i].addViewBox(enableMouse=True)
            self.sub_windows[i].img = pg.ImageItem(border='w')
            self.sub_windows[i].view.setAspectLocked(True)
            self.sub_windows[i].view.addItem(self.sub_windows[i].img)

        ## Layout
        layout = QGridLayout()
        layout.addWidget(self.sub_windows[0], 0, 0)
        layout.addWidget(self.sub_windows[1], 0, 1)
        layout.addWidget(self.sub_windows[2], 0, 2)
        layout.addWidget(self.sub_windows[3], 1, 0) 
        layout.addWidget(self.sub_windows[4], 1, 1) 
        layout.addWidget(self.sub_windows[5], 1, 2) 
        layout.addWidget(self.sub_windows[6], 2, 0) 
        layout.addWidget(self.sub_windows[7], 2, 1) 
        layout.addWidget(self.sub_windows[8], 2, 2) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        # set window size
        desktopWidget = QDesktopWidget();
        width = min(desktopWidget.height()*0.9,1000) #@@@TO MOVE@@@#
        height = width
        self.setFixedSize(width,height)

    def display_image(self,image,i):
        if i < 9:
            self.sub_windows[i].img.setImage(image,autoLevels=False)
            self.sub_windows[i].view.autoRange(padding=0)
