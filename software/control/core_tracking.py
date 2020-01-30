# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *
import control.tracking as tracking
import control.FocusTracking_LiquidLens as tracking_focus
import control.utils.image_processing as image_processing


import control.utils.PID as PID
import control.utils.units_converter as units_converter

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

    start_tracking_signal = Signal()

    save_command_signal = Signal(np.ndarray)

    multiplex_send_signal = Signal(int, int, int)

    ''' 
    Connection map

    centroid_image -> ImageDisplayer.draw_object
    Rect_pt1_pt2 -> ImageDisplayer.draw_bbox
    plot_data -> PlotWidget
    save_command_signal ->  DataSaver
    multiplex_send_signal -> multiplex_Send

    '''


    def __init__(self,microcontroller, image_axis = ['X', 'Z'], focus_axis = ['Y'], focus_tracker = 'liq-lens'):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        
        self.image_axis = image_axis
        self.focus_axis = focus_axis

        # Define list of trackers being used(maybe do this as a definition?)
        # OpenCV tracking suite
        self.OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
        }
        # Neural Net based trackers
        self.NEURALNETTRACKERS = {"daSiamRPN":[]}

        # Image Tracker type
        self.tracker_type = "csrt"

        self.create_tracker()

        # Focus Tracker type
        self.focus_tracker = focus_tracker

       

        start_flag = True

        self.objectFound = False

        self.tracking_triggered_prev = False

        # Image type of the tracking image stream
        self.color = False

        self.centroid = None
        self.rect_pts = None

        

        self.setPoint_image = None

        self.image_center = None

        self.image_width = None

        self.posError_image = np.array([0,0])

        self.image_offset = np.array([0,0])

        # Create a tracking object that does the image-based tracking
        # Pass it the list of trackers
        self.tracker_xz = tracking.Tracker_Image(self.OPENCV_OBJECT_TRACKERS, self.NEURALNETTRACKERS)
        # Create a tracking object that does the focus-based tracking
        self.tracker_y = tracking_focus.Tracker_Focus()

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

        self.initialise_track()

        




    # Triggered by signal from StreamHandler
    def on_new_frame(self,image, thresh_image = None, setPoint_image = None, frame_ID = None,timestamp = None ):
        
        # read current location from the microcontroller
        microcontroller_data, manualMode = self.microcontroller.read_received_packet()

        # Parse microcontroller data
        [YfocusPhase, Xpos_microController, Ypos_microController,Thetapos_microController, 
        tracking_triggered] = arduino_data

        if tracking_triggered and tracking_triggered != self.tracking_triggered_prev:
            ''' @@@@@ Then emit the start_tracking signal to change the track button 
            state of the Tracking Widget.
             EMIT (tracking_triggered)
            '''
            # This is Toggles the state of the Tracking Controller Widget
            start_tracking_signal.emit()
            
        self.tracking_triggered_prev = tracking_triggered

        if self.track_obj_image == True:
            # Update geometric parameters of image
            self.update_image_center_width(image)

            self.update_image_offset()

            self.update_tracking_setpoint()

            # Set the search area for nearest-nbr search
            self.set_searchArea()

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

                # Get initial parameters of the tracking image stream that are immutable
                self.set_image_props(image)
            
            else:

                start_flag = False
                self.resetPID = False

                
            self.objectFound, self.centroid, self.rect_pts = self.tracker_xz.track(image, thresh_image, self.tracker, self.tracker_type, start_flag = start_flag)
            

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
                x_error, z_error = units_converter.px_to_mm(self.posError_image[0], self.image_width), units_converter.px_to_mm(self.posError_image[1], self.image_width), 


                # get the object location along the optical axis. 
                # Is the object position necessary for this? Alternatively we can pass the centroid
                # and handle this downstream

                # Update the focus phase
                self.tracke_y.update_data(YfocusPhase)

                if self.focus_tracking:
                    y_error = self.tracker_y.get_error(image, self.focus_tracker)

                

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

                X_order, Y_order, Z_order = self.get_motion_commands(x_error,y_error,z_error)

            else:
                X_order, Y_order, Z_order = 0,0,0            
                # X_order, Y_order, Z_order is in stepper motor steps
                
            # We want to send to the microcontroller at a constant rate, even if an object is not found

            # Send the motion commands and instruct the multiplex send object to send data 
            # to the microcontroller
            multiplex_send_signal.emit(X_order, Y_order, Z_order)


            # Send the motion commands so they are available for the DataSaver
            save_command_signal.emit(np.array([self.Time[-1], self.X_objStage[-1], 
                    self.Y_objStage[-1], self.Z_objStage[-1], self.Theta_stage[-1], self.X_image[-1], self.Z_image[-1]]))



        # save the coordinate information (possibly enqueue image for saving here to if a separate ImageSaver object is being used) before the next movement
        # [to fill]

    # Triggered when you hit track_obj_image
    def initialise_track(self):

        self.tracking_frame_counter = 0

        #Time
        self.begining_Time = time.time()           #Time begin the first time we click on the start_tracking button
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

    def create_tracker(self):
        if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
            self.tracker = self.OPENCV_OBJECT_TRACKERS[self.tracker_type]()

        elif(self.tracker_type in self.NEURALNETTRACKERS.keys()):
            
            print('Using {} tracker'.format(self.tracker_type))

    def update_tracker(self, tracker_type):
        self.tracker_type = tracker_type

        # Update the actual tracker
        self.create_tracker()

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

    def get_focus_error(self,cropped_image):
        self.isFocusOrder = 0
        Y_order=0
        if self.track_focus:
            focusMeasure = image_processing.YTracking_Objective_Function(cropped_image, self.color)
            Y_order,self.isFocusOrder = self.tracker_y.get_error(focusMeasure)
            # Disabling Y tracking for 3D PIV
            # self.isYorder = 0
            return Y_order

    def get_motion_commands(self, x_error, y_error, z_error):
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

    # Image related functions

    def set_image_props(self, image):
        try:
            imW, imH, channels = np.shape(image)

            if(channels>2):
                self.color = True
            else:
                self.color = False
        except:
            self.color = False


    def update_thresh_image(self, thresh_image_data):
        # Take thresh_image from the thresh_image stream and set it as current image.
        self.thresh_image = thresh_image_data

    def update_image_center_width(self, image):
        self.image_center, self.image_width = image_processing.get_image_center_width(image)

    def update_tracking_setpoint(self):

        self.setPoint_image = self.image_center + self.image_offset

    def update_image_offset(self, image_offset):
        self.image_offset = image_offset

    def set_searchArea(self):

        self.tracker_xz.searchArea = int(self.image_width/Tracking.SEARCH_AREA_RATIO)




class microcontroller_MultiplexSend(QObject):

    def __init__(self, microcontroller, tracking_widget, focus_tracking_widget):
        QObject.__init__(self)

        self.microcontroller = microcontroller
        self.tracking_widget = tracking_widget
        self.focus_tracking_widget = focus_tracking_widget

        self.X_error, self.Y_error, self.Z_error = 0,0,0
        self.track_obj_image = 0
        self.track_focus = 0
        self.liquidLensFreq = 0
        self.liquidLensAmp = 0
        self.homing_state = 0

        

    def multiplex_Send(self, X_error, Y_error, Z_error):

        # X_error, Y_error, Z_error (in full steps)
        self.X_error = X_error
        self.Y_error = Y_error
        self.Z_error = Z_error

        # Update the local copy with the state of non-motion-related data to be sent to uController.
        self.set_NonMotionCommands()
        
        # Send command to the microcontroller
        self.microcontroller.send_command([self.X_error, self.Y_error, self.Z_error, self.track_obj_image, 
                self.track_focus, self.homing_state, self.liquidLensFreq, 
                self.liquidLensAmp])
            

    def set_NonMotionCommands(self):
        self.track_obj_image = self.tracking_widget.track_obj_image
        self.homing_state = self.tracking_widget.homing_state

        self.track_focus = self.focus_tracking_widget.track_focus
        self.liquidLensFreq = self.focus_tracking_widget.liquidLensFreq
        self.liquidLensAmp = self.focus_tracking_widget.liquidLensAmp

    '''
    Avoid using signals here. Instead just pass a reference to the appropriate widgets whose data
    needs to be saved using dataSaver
    '''
    # def set_TrackingState(self, tracking_state):
    #     self.tracking_state = tracking_state

    # def set_FocusTrackingState(self, focus_tracking_state):
    #     self.focus_tracking_state = focus_tracking_state

    # def set_LiquidLensFreq(self, liquidLensFreq):
    #     self.liquidLensFreq = liquidLensFreq

    # def set_LiquidLensAmp(self, liquidLensAmp)
    #     self.liquidLensAmp = liquidLensAmp

    # def set_HomingState(self, homing_state):
    #     self.homing_state = homing_state



class trackingDataSaver(QObject):

    def __init__(self, tracking_widget, focus_tracking_widget):
        QObject.__init__(self)

        self.queueLen = 10

        self.queue = Queue(self.queueLen) # max 10 items in the queue

        # Whether an object is being tracked in the image stream
        self.image_tracking_state = 0
        # Whether stage control is through computer (auto) or user (manual)
        self.stage_control_manual = 0
        
        self.focus_tracking_state = 0
        self.liquidLensFreq = 0
        self.liquidLensAmp = 0
        
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

    

    # Define slots that set the current value of variables based on signals from tracking Controller

    # For other data which saves the states and values of different widgets, pass a reference to the widget

    def setObjectPosition_image(self, Obj_position_image):
        # [X_centroid, Z_centroid] or [X_centroid, Y_centroid]
        self.Obj_position_image = Obj_position_image

    def setObject_Time_Position(self, Obj_Time_Position):
        # [Time, X_objStage, Y_objStage, Z_objStage]
        # This is the actual position of the cell (3D track) (T,X,Y,Z)
        self.Obj_Time_Position = Obj_Time_Position

    def setLiquidLensFreq(self, freq):
        self.LiquidLensFreq_curr = freq

    def set_StartFocusTracking(self, start_focus_tracking):
        self.start_focus_tracking_curr = start_focus_tracking

    def setImageName(self, imageName, imaging_channel):
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


            

           



