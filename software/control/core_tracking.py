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
import control.tracking as tracking
import utils.image_processing as image_processing


import utils.PID as PID
import utils.units_converter as units_converter

from queue import Queue
from collections import deque
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
import cv2

import time
from datetime import datetime

class TrackingController(QObject):

    # Signals
    centroid_image = Signal(np.ndarray)
    Rect_pt1_pt2 = Signal(np.ndarray)
    plot_data = Signal(np.ndarray)
    set_trackBusy = Signal(int)
    clear_trackBusy = Signal(int)

    motion_command_signal = Signal(np.ndarray)

    def __init__(self,microcontroller, image_axis = ['X', 'Z'], focus_axis = ['Y'], focus_tracker = 'liq-lens'):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        
        self.image_axis = image_axis
        self.focus_axis = focus_axis

        # Define list of trackers being used(maybe do this as a definition?)
        # OpenCV tracking suite
        OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
        }
        # Neural Net based trackers
        NEURALNETTRACKERS = {"daSiamRPN":[]}

        # Image Tracker type
        self.tracker_type = "csrt"

        self.create_tracker()

        # Focus Tracker type
        self.focus_tracker = focus_tracker

       

        start_flag = True

        self.objectFound = False

        self.tracking_triggered_prev = False

        self.centroid = None
        self.rect_pts = None

        

        self.setPoint_image = None

        self.image_center = None

        self.image_width = None

        self.posError_image = np.array([0,0])

        self.image_offset = np.array([0,0])

        # Create a tracking object that does the image-based tracking
        # Pass it the list of trackers
        self.tracker_xz = tracking.Tracker_Image(OPENCV_OBJECT_TRACKERS, NEURALNETTRACKERS)
        # Create a tracking object that does the focus-based tracking
        self.tracker_y = tracking.Tracker_Focus()

        # PID controller for each axis

        self.pid_controller_x = PID.PID()
        self.pid_controller_y = PID.PID()
        self.pid_controller_z = PID.PID()

        self.resetPID = True

        self.manualMode = False
        self.manualMode_prev = False

        self.tracking_frame_counter = 0

        # Deque data length
        self.dequeLen = 20

        #Time
        self.begining_Time = 0           #Time begin the first time we click on the start_tracking button
        self.Time = deque(maxlen=self.dequeLen)

        # Stage position
        self.X_stage = deque(maxlen=self.dequeLen)
        self.Y_stage = deque(maxlen=self.dequeLen)
        self.Theta_stage = deque(maxlen=self.dequeLen)

        self.X_image = deque(maxlen=self.dequeLen)
        self.Z_image = deque(maxlen=self.dequeLen)

        # Object position relative to the stage. 
        # This is what is recorded as the measured object position
        self.X_objStage = deque(maxlen=self.dequeLen)
        self.Y_objStage = deque(maxlen=self.dequeLen)
        self.Z_objStage = deque(maxlen=self.dequeLen)





    def on_new_frame(self,image, thresh_image = None, setPoint_image = None, frame_ID = None,timestamp = None ):
        
        # read current location from the microcontroller
        microcontroller_data, manualMode = self.microcontroller.read_received_packet()

        # Parse microcontroller data
        [YfocusPhase,Xpos_microController,Ypos_microController,Thetapos_microController, 
        tracking_triggered] = arduino_data

        if tracking_triggered and tracking_triggered != self.tracking_triggered_prev:
            ''' @@@@@ Then emit the start_tracking signal to change the track button 
            state of the Tracking Widget.
             EMIT (tracking_triggered)
            '''
            pass
        self.tracking_triggered_prev = tracking_triggered

        # Update geometric parameters of image
        self.update_image_center(image)

        self.update_image_offset()

        self.update_tracking_setpoint()

        # initialize the tracker when a new track is started
        if self.tracking_frame_counter == 0 or self.objectFound == False:
            ''' 
            First frame
            Get centroid using thresholding and initialize tracker based on this object.
            initialize the tracker
            '''
            start_flag = True

            # initialize the PID controller
            self.resetPID = True
        
        else:

            start_flag = False
            self.resetPID = False

            
        self.objectFound, self.centroid, self.rect_pts 
                = self.tracker_xz.track(image, thresh_image, self.tracker, self.tracker_type, 
                start_flag = start_flag)
        

        # Deepak: Good to avoid core image processing here. 
        # This belongs in streamHandler, or a separate object.
        # crop the image, resize the image 
        # [to fill]

    

        # Things to do if an object is detected.
        if(self.objectFound):

            if ((self.manualMode_prev == 1 and self.manualMode == 0)):
                # If we switched from manual to auto stage control.
                self.resetPID = True

            self.manualMode_prev = self.manualMode

            # Find the object's position relative to the tracking set point on the image
            self.posError_image = self.centroid - self.setPoint_image

            # Get the error and convert it to mm
            x_error, z_error = units_converter.px_to_mm(self.posError_image[0], self.image_width), 
                                units_converter.px_to_mm(self.posError_image[1], self.image_width), 


            # get the object location along the optical axis. 
            # Is the object position necessary for this? Alternatively we can pass the centroid
            # and handle this downstream
            y_error = self.tracker_y.track(image, self.focus_tracker)

            

            # Emit the detected centroid position so other widgets can access it.
            self.centroid_image.emit(self.centroid)
            self.rect_pts.emit(self.rect_pts)

            self.update_stage_position(Xpos_microController, Ypos_microController, 
                                        Thetapos_microController)

            self.update_image_position()

            self.update_obj_position()  

            # get motion commands


            dx = self.pid_controller_x.get_actuation(x)
            dy = self.pid_controller_y.get_actuation(y)
            dz = self.pid_controller_z.get_actuation(z)

            X_order, Y_order, Z_order = self.generate_motion_commands(x_error,y_error,z_error)

            
            # Send the motion commands so they are available for the DataSaver
            motion_command_signal.emit(np.array([X_order, Y_order, Z_order]))


            # save the coordinate information (possibly enqueue image for saving here to if a separate ImageSaver object is being used) before the next movement
            # [to fill]


            # send order to the microcontroller (Emit a signal that triggers this action 
            # in the microcontroller write queue object)
            


    def start_a_new_track(self):
        self.tracking_frame_counter = 0

    def create_tracker(self):
        if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
            self.tracker = self.OPENCV_OBJECT_TRACKERS[self.tracker_type]()

        elif(self.tracker_type in self.NeuralNetTrackers.keys()):
            
            
            print('Using {} tracker'.format(self.tracker_type))

    def update_tracker(self, tracker_type):
        self.tracker_type = tracker_type

        # Update the actual tracker
        self.create_tracker()


    def update_thresh_image(self, thresh_image_data):
        # Take thresh_image from the thresh_image stream and set it as current image.
        self.thresh_image = thresh_image_data

    def update_image_center(self, image):
        self.image_center, self.image_width = image_processing.get_image_center_width(image)

    def update_tracking_setpoint(self):

        self.setPoint_image = self.image_center + self.image_offset

    def update_image_offset(self, image_offset):
        self.image_offset = image_offset

    def update_elapsed_time(self):

        self.Time.append(time.time() - self.begining_Time)

    def update_stage_position(self, X,Y,Theta):

        self.X_stage.append(X)
        self.Y_stage.append(Y)
        self.Theta_stage.append(Theta)

    def update_image_position(self):
        # Object position relative to image center
        self.X_image.append(self.centroid[0] - self.image_center[0])
        self.Z_image.append(self.centroid[1] - self.image_center[1])

    def update_obj_position(self):

        self.X_objStage.append(self.X_stage[-1] + self.X_image)

        self.Y_objStage.append(self.Y_stage[-1])

        if(len(self.Time)>1):
            self.Z_objStage.append(self.ZobjStage[-1]+(self.Z_image[-1]-self.Z_image[-2])- units_converter.rad_to_mm(self.Theta_stage[-1]-self.Theta_stage[-2],self.X_objStage[-1]))
        else:
            self.Z_objStage.append(0)

    # def get_Yerror(self,cropped_image):
    #     self.track_focalAxis=0
    #     self.Yerror=0
    #     if self.start_Y_tracking:
    #         focusMeasure = image_processing.YTracking_Objective_Function(cropped_image, self.color)
    #         self.Yerror,self.track_focalAxis = self.ytracker.get_error(focusMeasure)
    #         # Disabling Y tracking for 3D PIV
    #         # self.isYorder = 0

    def generate_motion_commands(self, x_error, y_error, z_error):
        # Take an error signal and pass it through a PID algorithm
        x_error_steps = units_converter.X_mm_to_step(x_error)
        y_error_steps = units_converter.Y_mm_to_step(y_error)
        z_error_steps = units_converter.Z_mm_to_step(z_error)

        if self.resetPID:
            self.pid_controller_x.initiate(x_error_steps,self.Time[-1]) #reset the PID
            self.pid_controller_y.initiate(y_error_steps,self.Time[-1]) #reset the PID
            self.pid_controller_z.initiate(z_error_steps,self.Time[-1]) #reset the PID
            Z_order = 0
            X_order = 0
            Y_order = 0
        else:
            X_order = self.pid_controller_x.update(x_error_steps,self.Time[-1])
            X_order = round(X_order,2)

            Y_order = self.pid_controller_y.update(y_error_steps,self.Time[-1])
            Y_order = y_error_steps #@@@ NonPID focus tracking; may need to reverse the sign
            Y_order = round(Y_order,2)

            Z_order = self.pid_controller_z.update(z_error_steps,self.Time[-1])
            Z_order = round(Z_order,2)


        return X_order, Y_order, Z_order

    def get_nonMotion_commands(self, )



class trackingDataSaver(QObject):

    def __init__(self):
        QObject.__init__(self)

        self.queueLen = 10

        self.queue = Queue(self.queueLen) # max 10 items in the queue
        
        self.thread = Thread(target=self.process_queue)
        self.thread.start()

    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image,frame_ID,timestamp] = self.queue.get(timeout=0.1)
                
                self.counter = self.counter + 1
                self.queue.task_done()
                self.image_lock.release()
            except:
                pass


    def start_DataSaver(self):
        pass



    def stop_DataSaver(self):

        self.stop_signal_received = True
        self.thread.join()

    

    # Define slots that set the current value of variables  based on signals from other objects/widgets
    def setLiquidLensFreq(self, freq):
        self.LiquidLensFreq_curr = freq

    def set_StartFocusTracking(self, start_focus_tracking):
        self.start_focus_tracking_curr = start_focus_tracking

    def setImageName(self, imageName):
        self.imageName_curr = imageName


class ImageSaver(QObject):

    stop_recording = Signal()

    imageName = Signal(str)

    def __init__(self,image_format='.tiff'):
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
                # The file names should be unique for gravity machine
                file_ID = self.counter
                # create a new folder
                if file_ID == 0:
                    os.mkdir(os.path.join(self.base_path,self.experiment_ID,str(folder_ID)))
                saving_path = os.path.join(self.base_path,self.experiment_ID,str(folder_ID),str(file_ID) + '.' + self.image_format)
                
                # Emit the image name so DataSaver can save it along with the stage positions
                imageName.emit(str(file_ID) + '.' + self.image_format)
                # Save the image
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


            

           



