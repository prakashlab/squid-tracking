'''
core objects that run the back-end of the GUI.
Key objects:

    1. StreamHandler

'''

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


from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
import cv2
import imutils
from datetime import datetime

class StreamHandler(QObject):
    ''' Signals 
    '''
    image_to_display = Signal(np.ndarray, bool)
    packet_image_to_write = Signal(np.ndarray, int, float)
    packet_image_for_tracking = Signal(np.ndarray, np.ndarray)
    signal_new_frame_received = Signal()

    '''
    Signals
    image_to_display ->ImageDisplayer.enque
    packet_image_to_write ->ImageSaver
    packet_image_for_tracking -> Tracking_controller.on_new_frame
    signal_new_frame_received -> microcontroller_Receiver.get_Data

    Slots


    '''

    def __init__(self, camera = None , crop_width=3000,crop_height=3000,working_resolution_scaling = 0.5, trackingStream = True):
        QObject.__init__(self)
        self.fps_display = 1
        self.fps_save = 1
        self.fps_track = 1
        self.timestamp_last_display = 0
        self.timestamp_last_save = 0
        self.timestamp_last_track = 0

        self.crop_width = crop_width
        self.crop_height = crop_height
        self.working_resolution_scaling = working_resolution_scaling

        self.camera = camera

        # Raw image width
        if(camera is not None):
            self.image_width = self.camera.width
        else:
            self.image_width = 720


        self.save_image_flag = False

        # If current image stream is used for tracking.
        self.trackingStream = trackingStream

        self.track_flag = True

        self.handler_busy = False

        # for fps measurement
        self.timestamp_last = 0
        self.counter = 0
        self.fps_real = 0

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
        self.tracking_flag = True

    def stop_tracking(self):
        self.tracking_flag = False

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
        print(self.working_resolution_scaling)

    def set_image_thresholds(self, lower_HSV, upper_HSV):
        self.lower_HSV = lower_HSV
        self.upper_HSV = upper_HSV

        #@@@Testing
        print('Updated color thresholds to {} and {}'.format(self.lower_HSV, self.upper_HSV))

    def threshold_image(self, image_resized, color):
        if(color):
            thresh_image = image_processing.threshold_image(image_resized,self.lower_HSV,self.upper_HSV)  #The threshold image as one channel

        else:
            # print(self.lower_HSV[2])
            # print(self.upper_HSV[2])
            # img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            thresh_image = image_processing.threshold_image_gray(image_resized, self.lower_HSV[2], self.upper_HSV[2])

        return thresh_image

    def on_new_frame(self, camera):

        camera.image_locked = True
        self.handler_busy = True
        self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()
        # This also triggers the microcontroller_Receiever

        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last:
            self.counter = self.counter+1
        else:
            self.timestamp_last = timestamp_now
            self.fps_real = self.counter
            self.counter = 0
            print('real camera fps is ' + str(self.fps_real))

        # crop image
        # image = image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
        image = camera.current_frame

        # save a copy of full-res image for saving (make sure to do a deep copy)
        # @@@@@@@@@

        # image_resized = cv2.resize(image,(round(self.crop_width*self.working_resolution_scaling), round(self.crop_height*self.working_resolution_scaling)),cv2.INTER_LINEAR)
        image_resized = imutils.resize(image, round(self.image_width*self.working_resolution_scaling))

        
        
        # Deepak: For now tracking with every image from camera
        time_now = time.time() 
        if self.track_flag and self.trackingStream:
            # track is a blocking operation - it needs to be
            # @@@ will cropping before emitting the signal lead to speedup?
            image_thresh = 255*np.array(self.threshold_image(image_resized, color = camera.is_color), dtype = 'uint8')

            self.packet_image_for_tracking.emit(image_resized, image_thresh)
            self.timestamp_last_track = time_now

        # send image to display
        time_now = time.time()
        if time_now - self.timestamp_last_display >= 1/self.fps_display:
            if camera.is_color:
                image_resized = cv2.cvtColor(image_resized,cv2.COLOR_RGB2BGR)

            self.image_to_display.emit(image_resized, self.trackingStream)
            
            self.timestamp_last_display = time_now

        # send image to write
        time_now = time.time()
        if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
            if camera.is_color:
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            self.packet_image_to_write.emit(image, camera.frame_ID, self.camera.timestamp)
            self.timestamp_last_save = time_now


        

        self.handler_busy = False
        camera.image_locked = False


    def on_new_frame_from_simulation(self,image,frame_ID = None,timestamp = None):
        # check whether image is a local copy or pointer, if a pointer, needs to prevent the image being modified while this function is being executed
        color = False

        self.handler_busy = True
        self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()
        # This also triggers the microcontroller_Receiever

        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last:
            self.counter = self.counter+1
        else:
            self.timestamp_last = timestamp_now
            self.fps_real = self.counter
            self.counter = 0
            print('real camera fps is ' + str(self.fps_real))

        # crop image
        # image = image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
        image = np.array(np.copy(image), dtype = 'uint8')
        # save a copy of full-res image for saving (make sure to do a deep copy)
        # @@@@@@@@@

        # image_resized = cv2.resize(image,(round(self.crop_width*self.working_resolution_scaling), round(self.crop_height*self.working_resolution_scaling)),cv2.INTER_LINEAR)
        image_resized = imutils.resize(image, round(self.image_width*self.working_resolution_scaling))

        
        
        # Deepak: For now tracking with every image from camera 
        if self.track_flag and self.trackingStream:
            # track is a blocking operation - it needs to be
            # @@@ will cropping before emitting the signal lead to speedup?
            # print('Sending image to tracking controller...')

            image_thresh = 255*np.array(self.threshold_image(image_resized, color = False), dtype='uint8')

            cv2.imshow('Thresh image',image_thresh)
            cv2.waitKey(1)

            self.packet_image_for_tracking.emit(image_resized, image_thresh)
            self.timestamp_last_track = timestamp_now

        # send image to display
        time_now = time.time()
        if time_now - self.timestamp_last_display >= 1/self.fps_display:
            # print('Sending image to display...')
            if color:
                image_resized = cv2.cvtColor(image_resized,cv2.COLOR_RGB2BGR)

            self.image_to_display.emit(image_resized, self.trackingStream)
            
            self.timestamp_last_display = time_now

        # send image to write
        if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
            # print('Saving image...')
            if color:
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            self.packet_image_to_write.emit(image,camera.frame_ID,camera.timestamp)
            self.timestamp_last_save = time_now

        self.handler_busy = False

    def stop(self):
        pass
        # self.camera.stop()

class ImageSaver(QObject):

    stop_recording = Signal()

    def __init__(self,image_format='bmp'):
        QObject.__init__(self)
        self.base_path = './'
        self.experiment_ID = ''
        self.image_format = image_format
        self.max_num_image_per_folder = 1000
        self.queue = Queue(10) # max 10 items in the queue
        self.image_lock = Lock()
        self.stop_signal_received = False
        self.thread = Thread(target=self.process_queue)
        self.thread.start()
        self.counter = 0
        self.recording_start_time = 0
        self.recording_time_limit = -1

    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image,frame_ID,timestamp] = self.queue.get(timeout=0.1)
                self.image_lock.acquire(True)
                folder_ID = int(self.counter/self.max_num_image_per_folder)
                file_ID = int(self.counter%self.max_num_image_per_folder)
                # create a new folder
                if file_ID == 0:
                    os.mkdir(os.path.join(self.base_path,self.experiment_ID,str(folder_ID)))
                saving_path = os.path.join(self.base_path,self.experiment_ID,str(folder_ID),str(file_ID) + '.' + self.image_format)
                
                cv2.imwrite(saving_path,image)
                self.counter = self.counter + 1
                self.queue.task_done()
                self.image_lock.release()
            except:
                pass
                            
    def enqueue(self,image,frame_ID,timestamp):
        try:
            self.queue.put_nowait([image,frame_ID,timestamp])
            if ( self.recording_time_limit>0 ) and ( time.time()-self.recording_start_time >= self.recording_time_limit ):
                self.stop_recording.emit()
            # when using self.queue.put(str_), program can be slowed down despite multithreading because of the block and the GIL
        except:
            print('imageSaver queue is full, image discarded')

    def set_base_path(self,path):
        self.base_path = path

    def set_recording_time_limit(self,time_limit):
        self.recording_time_limit = time_limit

    def start_new_experiment(self,experiment_ID):
        # generate unique experiment ID
        self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f')
        self.recording_start_time = time.time()
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path,self.experiment_ID))
        except:
            pass
        # reset the counter
        self.counter = 0

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()

class LiveController(QObject):

    def __init__(self,camera,microcontroller):
        QObject.__init__(self)
        self.camera = camera
        self.microcontroller = microcontroller
        self.microscope_mode = None
        self.trigger_mode = TriggerMode.SOFTWARE # @@@ change to None
        self.is_live = False
        self.was_live_before_autofocus = False
        self.was_live_before_multipoint = False

        self.fps_software_trigger = 10;
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
        # if self.mode == MicroscopeMode.BFDF:
        #     self.microcontroller.toggle_LED(1)
        # else:
        #     self.microcontroller.toggle_laser(1)

    def turn_off_illumination(self):
        pass
        # if self.mode == MicroscopeMode.BFDF:
        #     self.microcontroller.toggle_LED(0)
        # else:
        #     self.microcontroller.toggle_laser(0)

    def start_live(self):
        self.is_live = True
        if self.trigger_mode == TriggerMode.SOFTWARE:
            self._start_software_triggerred_acquisition()

    def stop_live(self):
    	if self.is_live:
            self.is_live = False
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            self.turn_off_illumination()

    # software trigger related
    def trigger_acquisition_software(self):
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
            print('real trigger fps is ' + str(self.fps_real))

    def _start_software_triggerred_acquisition(self):
        self.timer_software_trigger.start()

    def _set_software_trigger_fps(self,fps_software_trigger):
        self.fps_software_trigger = fps_software_trigger
        self.timer_software_trigger_interval = (1/self.fps_software_trigger)*1000
        self.timer_software_trigger.setInterval(self.timer_software_trigger_interval)

    def _stop_software_triggerred_acquisition(self):
        self.timer_software_trigger.stop()

    # trigger mode and settings
    def set_trigger_mode(self,mode):
        pass # @@@

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
            self.turn_off_illumination()

class NavigationController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)
    zPos = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0

    def move_x(self,delta):
        self.microcontroller.move_x(delta)
        self.x_pos = self.x_pos + delta
        self.xPos.emit(self.x_pos)

    def move_y(self,delta):
        self.microcontroller.move_y(delta)
        self.y_pos = self.y_pos + delta
        self.yPos.emit(self.y_pos)

    def move_z(self,delta):
        self.microcontroller.move_z(delta)
        self.z_pos = self.z_pos + delta
        self.zPos.emit(self.z_pos*1000)

class ImageDisplay(QObject):

    image_to_display = Signal(np.ndarray, bool)

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
                [image, frame_ID, timestamp, trackingStream] = self.queue.get(timeout=0.1)
                print('Got image from queue')
                # self.image_lock.acquire(True)
                # Send image and trackingStream
                self.image_to_display.emit(image, trackingStream)
                print('Sent image to display window')
                # self.image_lock.release()
                # self.queue.task_done()
            except:
                print("Exception:", sys.exc_info()[0])
                print('Not sending image to display window')

    # def enqueue(self,image,frame_ID,timestamp):
    def enqueue(self,image, trackingStream = False):
        try:
            print('In image display queue')
            self.queue.put_nowait([image, None, None, trackingStream])
            # when using self.queue.put(str_) instead of try + nowait, program can be slowed down despite multithreading because of the block and the GIL
            pass
        except:
            print('imageDisplay queue is full, image discarded')

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()

    def __del__(self):
        self.wait()

# from gravity machine
class ImageDisplayWindow(QMainWindow):

    def __init__(self, window_title=''):
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
        self.graphics_widget.img = pg.ImageItem(border='w')
        self.graphics_widget.view.addItem(self.graphics_widget.img)

        ## Variables for annotating images
        self.DrawRect = False
        self.ptRect1 = None
        self.ptRect2 = None

        self.DrawCirc = False
        self.centroid = None

        self.DrawCrossHairs = False


        layout = QGridLayout()
        layout.addWidget(self.graphics_widget, 0, 0) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

    def display_image(self,image, trackingStream):
        
        if(trackingStream):

            if(self.DrawRect):
                cv2.rectangle(image, self.ptRect1, self.ptRect2,(0,0,0) , 2) #cv2.rectangle(img, (20,20), (300,300),(0,0,255) , 2)#
                self.DrawRect=False

            if(self.DrawCirc):
                cv2.circle(image,(self.centroid[0],self.centroid[1]), 20, (255,0,0), 2)
                self.DrawCirc=False

            if(self.DrawCrossHairs):
                cv2.Line(image, horLine_pt1, horLine_pt2, (255,255,255), thickness=1, lineType=8, shift=0) 
                cv2.Line(image, verLine_pt1, verLine_pt2, (255,255,255), thickness=1, lineType=8, shift=0) 
                self.DrawCrossHairs=False

        self.graphics_widget.img.setImage(image,autoLevels=False)
        print('In ImageDisplayWindow display image')
    
    
    def draw_rectangle(self, RectPts):
        # Connected to Signal from Tracking object
        self.DrawRect=True
        self.ptRect1=(pts[0][0],pts[0][1])
        self.ptRect2=(pts[1][0],pts[1][1])

    def draw_circle(self, centroid):
        # Connected to Signal from Tracking object
        self.DrawCirc=True
        self.centroid=(centroid[0],centroid[1])
        
    def draw_crosshairs(self, image_center, image_width):
        # Connected to Signal from Tracking object
        cross_length = round(image_width/20)

        horLine_pt1 = (round(image_center[0] - cross_length/2), image_center[1])
        horLine_pt2 = (round(image_center[0] + cross_length/2), image_center[1])

        verLine_pt1 = (round(image_center[0]), round(image_center[1] - cross_length/2))
        verLine_pt2 = (round(image_center[0]), round(image_center[1] + cross_length/2))

        self.DrawCrossHairs = True












        