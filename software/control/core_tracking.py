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
import pandas as pd

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

    save_data_signal = Signal()

    multiplex_send_signal = Signal(int, int, int)

    ''' 
    Connection map

    centroid_image -> ImageDisplayer.draw_object
    Rect_pt1_pt2 -> ImageDisplayer.draw_bbox
    plot_data -> PlotWidget
    multiplex_send_signal -> multiplex_Send
    save_data_signal -> DataSaver

    '''


    def __init__(self,microcontroller, InternalState , image_axis = ['X', 'Z'], focus_axis = ['Y'], focus_tracker = 'liq-lens'):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.InternalState = InternalState


        
        self.image_axis = image_axis
        self.focus_axis = focus_axis

       

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
        self.tracker_xz = tracking.Tracker_Image()
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

        self.internal_state_vars = ['Time', 'X_objStage','YobjStage','ZobjStage','Theta_stage','X_image', 'Z_image']

        




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

                
            self.objectFound, self.centroid, self.rect_pts = self.tracker_xz.track(image, thresh_image, start_flag = start_flag)
            

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
                # x_error, z_error are in mm
                x_error, z_error = units_converter.px_to_mm(self.posError_image[0], self.image_width), units_converter.px_to_mm(self.posError_image[1], self.image_width), 


                # get the object location along the optical axis. 
                # Is the object position necessary for this? Alternatively we can pass the centroid
                # and handle this downstream

                if(self.track_focus):
                    # Set the size of the cropped Image used for calculating focus measures
                    self.set_cropped_image_size()
                    # Update the focus phase
                    self.tracker_y.update_data(YfocusPhase)

                    # y-error in mm
                    y_error = self.tracker_y.get_focus_error(image, self.centroid)

                

                # Emit the detected centroid position so other widgets can access it.
                self.centroid_image.emit(self.centroid)
                self.rect_pts.emit(self.rect_pts)

                self.update_stage_position(Xpos_microController, Ypos_microController, 
                                            Thetapos_microController)

                self.update_image_position()

                self.update_obj_position()  

                # get motion commands

                X_order, Y_order, Z_order = self.get_motion_commands(x_error,y_error,z_error)

            else:
                X_order, Y_order, Z_order = 0,0,0            
                # X_order, Y_order, Z_order is in stepper motor steps
                
            # We want to send to the microcontroller at a constant rate, even if an object is not found

            # Send the motion commands and instruct the multiplex send object to send data 
            # to the microcontroller
            multiplex_send_signal.emit(X_order, Y_order, Z_order)


            # Update the Internal State Model
            self.update_internal_state()

            # Send a signal to the DataSaver module and instruct it to Save Data
            save_data_signal.emit()

            # 
            # Send the motion commands so they are available for the DataSaver
            # save_command_signal.emit(np.array([self.Time[-1], self.X_objStage[-1], 
            #         self.Y_objStage[-1], self.Z_objStage[-1], self.Theta_stage[-1], self.X_image[-1], self.Z_image[-1]]))



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
            self.Z_objStage.append(self.Z_objStage[-1]+(self.Z_image[-1]-self.Z_image[-2])- units_converter.rad_to_mm(self.Theta_stage[-1]-self.Theta_stage[-2],self.X_objStage[-1]))
        else:
            self.Z_objStage.append(0)


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

    def set_cropped_image_size(self):

        self.tracker_y.cropped_imSize = int(self.image_width/Tracking.CROPPED_IMG_RATIO)

    def get_latest_attr_value(self, key):

        temp = getattr(self, key)
        return temp[-1]

    def update_internal_state(self):

        for key in self.internal_state_vars:
            if(key in INTERNAL_STATE_VARIABLES):
                self.InternalState.data[key] = get_latest_attr_value(key)





class InternalState():
    '''
    This holds an up-to date internal state of GUI variables as well as Data from microcontroller

    '''
    def __init__(self):

        self.data = {key:[] for key in INTERNAL_STATE_VARIABLES}

    def initialise_internalState(self):
        # This assigns the default values for the internal state.

        for key in INTERNAL_STATE_VARIABLES:

            self.data[key] = INITIAL_VALUES[key]

        



class microcontroller_MultiplexSend(QObject):

    '''
    Command list to microcontroller

    X_order, Y_order, Z_order, track_obj_image, track_focus, liquidLensFreq, liquidLensAmpl, homing

    '''

    def __init__(self, microcontroller, internal_state):
        QObject.__init__(self)

        self.microcontroller = microcontroller
        self.internal_state = internal_state

        self.sendData = {key:[] for key in SEND_DATA}

        

    def multiplex_Send(self, X_order, Y_order, Z_order):

        # X_error, Y_error, Z_error (in full steps)
        self.sendData['X_order'] = X_order
        self.sendData['Y_order'] = Y_order
        self.sendData['Z_order'] = Z_order

        # Update the local copy with the state of non-motion-related data to be sent to uController.
        self.get_sendData()
        
        # Send command to the microcontroller
        self.microcontroller.send_command([sendData[key] for key in SEND_DATA])

    



    def get_sendData(self):

        for key in SEND_DATA:
            try:
                self.sendData[key] = self.internal_state[key]
            except:
                print('{} not found in Internal State model'.format(key))


            

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

    def __init__(self, internal_state):
        QObject.__init__(self)

        self.internal_state = internal_state

        self.base_path = './'
        self.experiment_ID = ''

        self.queueLen = 10

        self.queue = Queue(self.queueLen) # max 10 items in the queue

        self.saveDataNames = SAVE_DATA

        self.DataToQueue = {key:[] for key in self.saveDataNames}

        self.DataToSave = {key:[] for key in self.saveDataNames}

        self.image_names = None


        # Use a counter 
        self.counter = 0
        
        self.thread = Thread(target=self.process_queue)
        self.thread.start()

    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                self.DataToSave = self.queue.get(timeout=0.1)
                
                self.counter = self.counter + 1
                self.queue.task_done()
                self.image_lock.release()
            except:
                pass

    def enqueue(self):

        # Get the most recent internal state values
        for key in self.saveDataNames:
            self.DataToQueue[key] = self.internal_state[key]

        try:

            self.queue.put_nowait(self.DataToQueue)

        except:
            'Data queue full, current cycle data not saved'

    # def create_dataFile(self):




    def start_DataSaver(self):
        ''' 
        This function is triggered when an Acquisition is started in the "Tracking" uScope mode
        '''
        self.track_counter = 0



    def stop_DataSaver(self):

        self.stop_signal_received = True
        self.thread.join()


    # Function sets the image names for all the imaging channels    
    def setImageName(self, optical_path, image_name):
        pass


    def update_imaging_channels(self, imaging_channels):
        pass



class ImageSaver(QObject):

    stop_recording = Signal()

    imageName = Signal(str, str)

    '''
    Connections
    imageName -> DataSaver

    '''
    def __init__(self, internal_state, imaging_channel = None, image_format='.tif'):
        QObject.__init__(self)

        self.internal_state = internal_state

        # imaging-channel that is using this ImageSaver object
        self.imaging_channel = imaging_channel

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
        self.folder_counter = 0
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
                # create a new folder (base_path/imaging_channel/subFolderID/fileID)
                if file_ID == 0 or int(self.counter%self.max_num_image_per_folder)==0:
                    folder_images = os.path.join(self.base_path, self.experiment_ID, self.imaging_channel, '{:05d}'.format(folder_ID))
                    os.mkdir(folder_images)
                
                image_file_name = '{:07d}'.format(file_ID) + '.' + self.image_format

                saving_path = os.path.join(folder_images, image_file_name)
                
                # Emit the image name so DataSaver can save it along with the stage positions
                imageName.emit(image_file_name, self.imaging_channels)
                # Save the image
                cv2.imwrite(saving_path,image)
                self.counter = self.counter + 1
                self.queue.task_done()
                self.image_lock.release()
            except:
                pass
                            
    def enqueue(self,image, frame_ID, timestamp):
        try:
            self.queue.put_nowait([image,frame_ID,timestamp])
            if ( self.recording_time_limit>0 ) and ( time.time()-self.recording_start_time >= self.recording_time_limit ):
                self.stop_recording.emit()
            # when using self.queue.put(str_), program can be slowed down despite multithreading because of the block and the GIL
        except:
            print('imageSaver queue is full, image discarded')

    def set_base_path(self,path):
        self.base_path = path
        # Update internal state
        self.internal_state['base_path'] = path



    def set_recording_time_limit(self,time_limit):
        self.recording_time_limit = time_limit

    def start_new_experiment(self,experiment_ID):
        # generate unique experiment ID
        self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S')
        

        self.internal_state['experiment_ID'] = experiment_ID
        
        # Create and store metadata file
        self.create_metadata_file()

        self.recording_start_time = time.time()
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path, self.experiment_ID, self.imaging_channel))
        except:
            pass
        # reset the counter
        self.counter = 0

    def create_metadata_file(self):
        config_file = os.path.join(self.base_path, self.experiment_ID, 'metadata.csv')

        df = pd.DataFrame({'Objective':[self.internal_state['Objective']], 
                    'PixelPermm':[self.internal_state['Objective']['PixelPermm']],'Local time':[datetime.now().strftime('%Y-%m-%d, %H:%M:%S.%f')]})
        df.to_csv(config_file)

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()


            

           



