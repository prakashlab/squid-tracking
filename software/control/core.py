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

    def __init__(self, camera = None , crop_width=2000,crop_height=2000, working_resolution_scaling = WORKING_RES_DEFAULT, imaging_channel = TRACKING):
        QObject.__init__(self)
        self.fps_display = FPS['display']['default']
        self.fps_save = 1
        self.fps_track = 1
        self.timestamp_last_display = 0
        self.timestamp_last_save = 0
        self.timestamp_last_track = 0

        self.crop_width = crop_width
        self.crop_height = crop_height
        self.working_resolution_scaling = working_resolution_scaling

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
        # print(self.working_resolution_scaling)

    def set_image_thresholds(self, lower_HSV, upper_HSV):
        self.lower_HSV = lower_HSV
        self.upper_HSV = upper_HSV

        #@@@Testing
        print('Updated color thresholds to {} and {}'.format(self.lower_HSV, self.upper_HSV))

    def update_invert_image_flag(self, flag):
        self.invert_image_flag = flag
        
    def threshold_image(self, image_resized, color):
        if(color):
            thresh_image = image_processing.threshold_image(image_resized,self.lower_HSV,self.upper_HSV)  #The threshold image as one channel

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
        # This also triggers the microcontroller_Receiever

        
        self.get_real_stream_fps()

        # crop image
        image = image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
        # image = camera.current_frame

        # save a copy of full-res image for saving (make sure to do a deep copy)
        # @@@@@@@@@

        
        if(self.imaging_channel == TRACKING):
            image_resized = cv2.resize(image,(round(self.crop_width*self.working_resolution_scaling), round(self.crop_height*self.working_resolution_scaling)),cv2.INTER_LINEAR)
            # image_resized = imutils.resize(image, self.working_image_width)

            # Threshold the image based on the color-thresholds
            image_thresh = 255*np.array(self.threshold_image(image_resized, color = camera.is_color), dtype = 'uint8')

        else:

            image_resized = np.copy(image)
        
        # Deepak: For now tracking with every image from camera
        time_now = time.time() 
        if self.track_flag and self.imaging_channel == TRACKING:
            # track is a blocking operation - it needs to be
            # @@@ will cropping before emitting the signal lead to speedup?

            self.packet_image_for_tracking.emit(image_resized, image_thresh)
            self.timestamp_last_track = time_now

        # send image to display
        time_now = time.time()
        if time_now - self.timestamp_last_display >= 1/self.fps_display:

            if camera.is_color:
                image_resized = cv2.cvtColor(image_resized,cv2.COLOR_RGB2BGR)

            # print('Displaying image for {} channel'.format(self.imaging_channel))
            
            self.image_to_display.emit(image_resized, self.imaging_channel)
            
            if(self.imaging_channel == TRACKING):
                # Send thresholded image to display (only for tracking stream)
                self.thresh_image_to_display.emit(image_thresh)
                self.signal_working_resolution.emit(round(self.crop_width*self.working_resolution_scaling))
            
            self.timestamp_last_display = time_now

            self.get_real_display_fps()

            
        # send image to write
        time_now = time.time()
        if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
            if camera.is_color:
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            
            print('Sending image for save')
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


    # def on_new_frame_from_simulation(self,image,frame_ID = None,timestamp = None):
    #     # check whether image is a local copy or pointer, if a pointer, needs to prevent the image being modified while this function is being executed
    #     color = False

    #     self.handler_busy = True
    #     self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()
    #     # This also triggers the microcontroller_Receiever

    #     # measure real fps
    #     timestamp_now = round(time.time())
    #     if timestamp_now == self.timestamp_last:
    #         self.counter = self.counter+1
    #     else:
    #         self.timestamp_last = timestamp_now
    #         self.fps_real = self.counter
    #         self.counter = 0
    #         print('real camera fps is ' + str(self.fps_real))

    #     # crop image
    #     # image = image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
    #     image = np.array(np.copy(image), dtype = 'uint8')
    #     # save a copy of full-res image for saving (make sure to do a deep copy)
    #     # @@@@@@@@@

    #     # image_resized = cv2.resize(image,(round(self.crop_width*self.working_resolution_scaling), round(self.crop_height*self.working_resolution_scaling)),cv2.INTER_LINEAR)
    #     image_resized = imutils.resize(image, round(self.image_width*self.working_resolution_scaling))

        
        
    #     # Deepak: For now tracking with every image from camera 
    #     if self.track_flag and self.trackingStream:
    #         # track is a blocking operation - it needs to be
    #         # @@@ will cropping before emitting the signal lead to speedup?
    #         # print('Sending image to tracking controller...')

    #         image_thresh = 255*np.array(self.threshold_image(image_resized, color = False), dtype='uint8')

    #         cv2.imshow('Thresh image',image_thresh)
    #         cv2.waitKey(1)

    #         self.packet_image_for_tracking.emit(image_resized, image_thresh)
    #         self.timestamp_last_track = timestamp_now

    #     # send image to display
    #     time_now = time.time()
    #     if time_now - self.timestamp_last_display >= 1/self.fps_display:
    #         # print('Sending image to display...')
    #         if color:
    #             image_resized = cv2.cvtColor(image_resized,cv2.COLOR_RGB2BGR)

    #         self.image_to_display.emit(image_resized, self.trackingStream)
            
    #         self.timestamp_last_display = time_now

    #     # send image to write
    #     if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
    #         # print('Saving image...')
    #         if color:
    #             image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
    #         self.packet_image_to_write.emit(image,camera.frame_ID,camera.timestamp)
    #         self.timestamp_last_save = time_now

    #     self.handler_busy = False

    # def stop(self):
    #     pass
    #     # self.camera.stop()


class LiveController(QObject):

    def __init__(self,camera,microcontroller):
        QObject.__init__(self)
        self.camera = camera
        self.microcontroller = microcontroller
        self.microscope_mode = None
        self.trigger_mode = TriggerMode.SOFTWARE # @@@ change to None
        self.mode = None
        self.is_live = False
        self.was_live_before_autofocus = False
        self.was_live_before_multipoint = False

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
        if self.mode == MicroscopeMode.BFDF:
            self.microcontroller.toggle_LED(1)
        else:
            self.microcontroller.toggle_laser(1)

    def turn_off_illumination(self):
        if self.mode == MicroscopeMode.BFDF:
            self.microcontroller.toggle_LED(0)
        else:
            self.microcontroller.toggle_laser(0)

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

            # Send command to uController to stop triggering camera
            self.microcontroller.send_hardware_trigger_command(0)

            if self.is_live:
                self._start_software_triggerred_acquisition()
        if mode == TriggerMode.HARDWARE:
            print('Setting camera to hardware trigger')
            self.camera.set_hardware_triggered_acquisition()

            # Send command to uController to start triggering camera
            self.microcontroller.send_hardware_trigger_command(1)

        if mode == TriggerMode.CONTINUOUS: 
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            if mode == TriggerMode.HARDWARE:
                # Stop hardware triggered aquisition
                self.microcontroller.send_hardware_trigger_command(0)
                
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
            self.turn_off_illumination()

class NavigationController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)
    zPos = Signal(float)
    thetaPos = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos = 0 # in mm
        self.y_pos = 0 # in mm
        self.theta_pos = 0 # in radians
        self.z_pos = 0 # in mm

    def move_x(self,delta):
        self.microcontroller.move_x(delta)
        self.x_pos = self.x_pos + delta

    def move_y(self,delta):
        self.microcontroller.move_y(delta)
        self.y_pos = self.y_pos + delta

    def move_z(self,delta):
        self.microcontroller.move_z(delta)
        self.z_pos = self.z_pos + delta

    def move_theta(self,delta):
        self.microcontroller.move_theta(delta)
        self.theta_pos = self.theta_pos + delta

    # For closed loop operation get stage positions from uController.
    def update_stage_positions(self, x_pos, y_pos, theta_pos):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.theta_pos = theta_pos

    # @@@ Implement later. Make the NavigationController the central node for stage positions.
    def emit_stage_positions(self):
        pass



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

    def __init__(self, window_title='', DrawCrossHairs = False, rotate_image_angle = 0, flip_image = None):
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

        # Image rotations and flipping
        self.rotate_image_angle = rotate_image_angle
        self.flip_image = flip_image

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


     


        self.graphics_widget.img.setImage(image,autoLevels=False)
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
        ymin = max(0, self.image_height - height - self.roi_pos[1])
        # print('Bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))

        self.roi_bbox.emit(np.array([xmin, ymin, width, height]))
        # print('Sent bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))