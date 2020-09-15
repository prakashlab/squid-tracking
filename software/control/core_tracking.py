# set QT_API environment variable
import os, sys
import traceback
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
from control.utils.units_converter import Units_Converter
import control.utils.byte_operations as byte_operations

import control.utils.CSV_Tool as CSV_Tool

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
	tracking_setPoint = Signal(np.ndarray)
	plot_data = Signal(np.ndarray)
	set_trackBusy = Signal(int)
	clear_trackBusy = Signal(int)

	start_tracking_signal = Signal()

	save_data_signal = Signal()

	''' 
	Connection map

	centroid_image -> ImageDisplayer.draw_object
	Rect_pt1_pt2 -> ImageDisplayer.draw_bbox
	plot_data -> PlotWidget
	multiplex_send_signal -> multiplex_Send
	save_data_signal -> DataSaver
	start_tracking_signal -> Tracking Widget

	'''
	def __init__(self, microcontroller, internal_state , image_axis = ['X', 'Y'], focus_axis = ['Z'], focus_tracker = 'liq-lens'):
		QObject.__init__(self)
		self.microcontroller = microcontroller
		self.internal_state = internal_state


		self.units_converter = Units_Converter()

		# Set the reference image width based on the camera sensor size used for calibration
		# This allows physical distances be calculated even if the image res is down-sampled.
		self.units_converter.set_calib_imWidth(CALIB_IMG_WIDTH)
		
		self.image_axis = image_axis
		self.focus_axis = focus_axis

		self.image = None

		# Focus Tracker type
		self.focus_tracker = focus_tracker

		self.track_focus = False

		# For testing
		self.track_obj_image = False

		self.start_flag = True

		self.objectFound = False

		self.tracking_triggered_prev = False

		# Image type of the tracking image stream
		self.color = False

		self.centroid = None
		self.rect_pts = None

		

		self.image_setPoint = None

		self.image_center = None

		self.image_width = 720

		self.posError_image = np.array([0,0])

		self.image_offset = np.array([0,0])

		# Create a tracking object that does the image-based tracking
		self.tracker_image = tracking.Tracker_Image()
		# Create a tracking object that does the focus-based tracking
		self.tracker_focus = tracking_focus.Tracker_Focus()

		# PID controller for each axis

		self.pid_controller_x = PID.PID()
		self.pid_controller_y = PID.PID()
		self.pid_controller_z = PID.PID()

		self.resetPID = True

		self.stage_auto = False
		self.stage_auto_prev = False

		self.tracking_frame_counter = 0

		# Deque data length
		self.dequeLen = 20

		#Time
		self.begining_Time = time.time()           #Time begin the first time we click on the start_tracking button
		self.Time = deque(maxlen=self.dequeLen)


		self.X_image = deque(maxlen=self.dequeLen)
		self.Y_image = deque(maxlen=self.dequeLen)
		# Subset of INTERNAL_STATE_MODEL that is updated by Tracking_Controller (self)
		self.internal_state_vars = ['Time','X_image', 'Y_image']

		
		self.tracker_focus.cropped_imSize = int(self.image_width/CROPPED_IMG_RATIO)





	# Triggered by signal from StreamHandler
	def on_new_frame(self,image, thresh_image = None):
		
		self.image = image

		# @@@testing
		# print('In Tracking controller new frame')

		tracking_triggered = self.internal_state.data['track_obj_image_hrdware']

		self.stage_auto = self.internal_state.data['track_obj_stage']

		# If image tracking is triggered using hardware button
		# Need to distinguish between Hardware button and Software button-based triggers
		if tracking_triggered and tracking_triggered != self.tracking_triggered_prev:
			''' @@@@@ Then emit the start_tracking signal to change the track button 
			state of the Tracking Widget.
			 EMIT (tracking_triggered)
			'''
			# This Toggles the state of the Track Button.
			self.start_tracking_signal.emit()
			self.internal_state.data['track_obj_image_hrdware'] = False

			
		self.tracking_triggered_prev = tracking_triggered

		''' 
			Note that this needs to be a local copy since on the internal_state 
			value changing due to a hardware button press
		'''
		if self.internal_state.data['track_obj_image'] == True:

			
			self.update_elapsed_time()

			# print('In track function')

			# Update image parameters
			# TO DO: Only call this when image resolution changes
			# DOnE!


			# initialize the tracker when a new track is started
			if self.tracking_frame_counter == 0 or self.objectFound == False:
				''' 
				First frame
				Get centroid using thresholding and initialize tracker based on this object.
				initialize the tracker
				'''

				self.start_flag = True

				# initialize the PID controller
				self.resetPID = True

				# Get initial parameters of the tracking image stream that are immutable
				self.set_image_props()

				self.update_image_center_width()

				self.update_tracking_setpoint()

			
			else:

				self.start_flag = False
				self.resetPID = False

				
			self.objectFound, self.centroid, self.rect_pts = self.tracker_image.track(image, thresh_image, start_flag = self.start_flag)
			
			self.tracking_frame_counter += 1
	
			#-----------------------------------------------------
			# Tests
			#-----------------------------------------------------
		 
			# print(self.objectFound, self.centroid, self.rect_pts)
			# cv2.circle(image,(self.centroid[0], self.centroid[1]), 20, (255,0,0), 2)
			# self.ptRect1=(self.rect_pts[0][0], self.rect_pts[0][1])
			# self.ptRect2=(self.rect_pts[1][0], self.rect_pts[1][1])
			# cv2.rectangle(image, self.ptRect1, self.ptRect2,(0,0,0) , 2) #cv2.rectangle(img, (20,20), (300,300),(0,0,255) , 2)#


			# cv2.imshow('Image with centroid', image)
			# cv2.waitKey(1)
			#-----------------------------------------------------


			# Deepak: Good to avoid core image processing here. 
			# This belongs in streamHandler, or a separate object.
			# crop the image, resize the image 
			# [to fill]

		

			# Things to do if an object is detected.
			if(self.objectFound):

				if ((self.stage_auto_prev == 0 and self.stage_auto == 1)):
					# If we switched from manual to auto stage control.
					self.resetPID = True

				self.stage_auto_prev = self.stage_auto

				# Find the object's position relative to the tracking set point on the image
				self.posError_image = self.centroid - self.image_setPoint

				# Get the error and convert it to mm
				# x_error, z_error are in mm
				for index, axis in enumerate(self.image_axis):
		
					x_error, y_error = self.units_converter.px_to_mm(self.posError_image[0], self.image_width), self.units_converter.px_to_mm(self.posError_image[1], self.image_width), 

				# Flip the sign of Z-error since image coordinates and physical coordinates are reversed.
				# z_error = -z_error

				# get the object location along the optical axis. 
				# Is the object position necessary for this? Alternatively we can pass the centroid
				# and handle this downstream
				if(self.track_focus):
				
					# Update the focus phase
					self.tracker_focus.update_data(FocusPhase)

					# y-error in mm
					z_error = self.tracker_focus.get_focus_error(image, self.centroid)
				else:
					z_error = 0

				

				# Emit the detected centroid position so other widgets can access it.
				self.centroid_image.emit(self.centroid)
				
				self.Rect_pt1_pt2.emit(self.rect_pts)


				self.update_image_position()

				# get motion commands
				# Error is in mm.
				# print('Image error: {}, {}, {} mm'.format(x_error, y_error, z_error))
				X_order, Y_order, Z_order = self.get_motion_commands_xyz(x_error,y_error,z_error)

				# New serial interface (send data directly to micro-controller object)
				print('Command sent to micro-controller: {}, {}, {}'.format(X_order, Y_order, Z_order))
				# self.microcontroller.send_motion_command_xytheta(X_order, Y_order, Theta_order)
				# self.microcontroller.send_motion_command_xyz(X_order, Y_order, Z_order)  

				self.microcontroller.move_x_nonblocking(X_order)
				self.microcontroller.move_y_nonblocking(Y_order)
				self.microcontroller.move_z_nonblocking(Z_order)  
		

			# Update the Internal State Model
			self.update_internal_state()

			# Send a signal to the DataSaver module and instruct it to Save Data
			self.save_data_signal.emit()

			
	# Triggered when you hit track_obj_image
	def initialise_track(self):

		# @@@ Testing
		print('Initializing track...')
		
		self.tracking_frame_counter = 0

		self.start_flag = True

		self.objectFound = False

		self.tracking_triggered_prev = False

		#Time
		self.begining_Time = time.time()           #Time begin the first time we click on the start_tracking button
		self.Time = deque(maxlen=self.dequeLen)

		# self.X_image = deque(maxlen=self.dequeLen)
		# self.Z_image = deque(maxlen=self.dequeLen)
		self.X_image = deque(maxlen=self.dequeLen)
		self.Y_image = deque(maxlen=self.dequeLen)

	def update_elapsed_time(self):

		self.Time.append(time.time() - self.begining_Time)


	def update_image_position(self):
		# Object position relative to image center (in mm)
		
		self.X_image.append(self.units_converter.px_to_mm(self.centroid[0] - self.image_center[0], self.image_width))
		self.Y_image.append(self.units_converter.px_to_mm(self.centroid[1] - self.image_center[1], self.image_width))

	def get_motion_commands_xyz(self, x_error, y_error, z_error):
		# Convert from mm to steps (these are rounded to the nearest integer).
		x_error_steps = int(Motion.STEPS_PER_MM_XY*x_error)
		y_error_steps = int(Motion.STEPS_PER_MM_XY*y_error)
		z_error_steps = int(Motion.STEPS_PER_MM_Z*z_error)

		if self.resetPID:
			self.pid_controller_x.initiate(x_error_steps,self.Time[-1]) #reset the PID
			self.pid_controller_y.initiate(y_error_steps,self.Time[-1]) #reset the PID
			self.pid_controller_z.initiate(z_error_steps,self.Time[-1]) #reset the PID
			
			X_order = 0
			Y_order = 0
			Z_order = 0

		else:
			X_order = self.pid_controller_x.update(x_error_steps,self.Time[-1])
			X_order = round(X_order,2)

			Y_order = self.pid_controller_y.update(y_error_steps,self.Time[-1])
			Y_order = y_error_steps #@@@ NonPID focus tracking; may need to reverse the sign
			Y_order = round(Y_order,2)

			Z_order = self.pid_controller_z.update(z_error_steps,self.Time[-1])
			Z_order = round(Z_order,2)


		return X_order, Y_order, Z_order


	# For Gravity Machine (X, Y, Theta tracking)
	# def get_motion_commands(self, x_error, y_error, z_error):
	# 	# Take an error signal and pass it through a PID algorithm

	# 	# Convert from mm to steps (these are rounded to the nearest integer).
	# 	x_error_steps = int(Motors.MAX_MICROSTEPS*self.units_converter.X_mm_to_step(x_error))
	# 	y_error_steps = int(Motors.MAX_MICROSTEPS*self.units_converter.Y_mm_to_step(y_error))
	# 	z_error_steps = int(Motors.MAX_MICROSTEPS*self.units_converter.Z_mm_to_step(z_error))

	# 	if self.resetPID:
	# 		self.pid_controller_x.initiate(x_error_steps,self.Time[-1]) #reset the PID
	# 		self.pid_controller_y.initiate(y_error_steps,self.Time[-1]) #reset the PID
	# 		self.pid_controller_z.initiate(theta_error_steps,self.Time[-1]) #reset the PID
			
	# 		X_order = 0
	# 		Y_order = 0
	# 		Theta_order = 0

	# 	else:
	# 		X_order = self.pid_controller_x.update(x_error_steps,self.Time[-1])
	# 		X_order = round(X_order,2)

	# 		Y_order = self.pid_controller_y.update(y_error_steps,self.Time[-1])
	# 		Y_order = y_error_steps #@@@ NonPID focus tracking; may need to reverse the sign
	# 		Y_order = round(Y_order,2)

	# 		Theta_order = self.pid_controller_z.update(theta_error_steps,self.Time[-1])
	# 		Theta_order = round(Theta_order,2)


	# 	return X_order, Y_order, Theta_order

	# Image related functions

	def set_image_props(self):
		try:
			imW, imH, channels = np.shape(self.image)

			if(channels>2):
				self.color = True
			else:
				self.color = False
		except:
			self.color = False


	def update_image_center_width(self):
		if(self.image is not None):
			self.image_center, self.image_width = image_processing.get_image_center_width(self.image)
			
			# Update search area
			self.set_searchArea()

			# The tracking set point is modified since it depends on the image center.
			self.update_tracking_setpoint()
			
			# print('New image width: {}'.format(self.image_width))


	def update_tracking_setpoint(self):

		self.image_setPoint = self.image_center + self.image_offset
		#@@@Testing
		# print('New tracking set point :{}'.format(self.image_setPoint))

	def update_image_offset(self, new_image_offset):
		self.image_offset = new_image_offset

		self.update_tracking_setpoint()
		#@@@Testing
		# print('Updated image offset to :{}'.format(self.image_offset))

	def set_searchArea(self):

		self.tracker_image.searchArea = int(self.image_width/Tracking.SEARCH_AREA_RATIO)
		# print('current search area : {}'.format(self.tracker_image.searchArea))

	def set_cropped_image_size(self, new_ratio):


		self.tracker_focus.cropped_imSize = int(self.image_width/new_ratio)

		# @@@ Testing
		# print('new cropped image size: {}'.format(self.tracker_focus.cropped_imSize))

	def get_latest_attr_value(self, key):

		temp = getattr(self, key)
		return temp[-1]

	def update_internal_state(self):

		for key in self.internal_state_vars:

			if(key in INTERNAL_STATE_VARIABLES):
				self.internal_state.data[key] = self.get_latest_attr_value(key)
			else:
				raise NameError('Key not found in Internal State')


	def send_focus_tracking(self, focus_tracking_flag):
		self.microcontroller.send_focus_tracking_command(focus_tracking_flag)


class InternalState():
	'''
	This holds an up-to date internal state of GUI variables as well as Data from microcontroller

	'''
	def __init__(self):

		self.data = {key:[] for key in INTERNAL_STATE_VARIABLES}

		self.initialise_internalState()

	def initialise_internalState(self):
		# This assigns the default values for the internal state.

		for key in INTERNAL_STATE_VARIABLES:

			self.data[key] = INITIAL_VALUES[key]

		
class microcontroller_Receiver(QObject):

	'''
	Receives data from microcontroller and updates the Internal state variables to the latest value
	Connection Map:
	StreamHandler (rec new image) -> getData_microcontroller
	'''
	update_display = Signal()

	def __init__(self, microcontroller, internal_state):
		QObject.__init__(self)

		self.microcontroller = microcontroller
		self.internal_state = internal_state
		self.units_converter = Units_Converter()
		self.RecData = {key:[] for key in REC_DATA}

		# Define a timer to read the Arduino at regular intervals
		self.timer_read_uController = QTimer()
		self.timer_read_uController.setInterval(MicrocontrollerDef.UCONTROLLER_READ_INTERVAL)
		self.timer_read_uController.timeout.connect(self.getData_microcontroller)
		self.timer_read_uController.start()

		self.stop_signal_received = False

		self.time_now = 0
		self.time_prev = 0

		self.x_pos = 0
		self.y_pos = 0
		self.z_pos = 0


		# self.read_Thread = Thread(target = self.getData_microcontroller)
		# self.read_Thread.start()

	# This function is triggered by the "rec new image signal" from StreamHandler
	def getData_microcontroller(self):
		# for debugging
		# while True:

		# 	if self.stop_signal_received:
		# 		return
		# 	self.time_now = time.time()

			# if(self.time_now - self.time_prev >= UCONTROLLER_READ_INTERVAL):
				
				# print(self.time_now)
				# print("Receiving data from uController")

		# print("Receiving data from uController")
		data = self.microcontroller.read_received_packet_nowait()

		if(data is not None):

			# Parse the data
			self.x_pos = byte_operations.unsigned_to_signed(data[0:3],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_XY 
			self.y_pos = byte_operations.unsigned_to_signed(data[3:6],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_XY
			self.z_pos = byte_operations.unsigned_to_signed(data[6:9],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_Z 

			self.RecData['X_stage'] = self.x_pos
			self.RecData['Y_stage'] = self.y_pos
			self.RecData['Z_stage'] = self.z_pos

			for key in REC_DATA:
				if(key in INTERNAL_STATE_VARIABLES):
					self.internal_state.data[key] = self.RecData[key]

			# print('Read packet ... parsing')
			# for key in REC_DATA:
			# 	self.RecData[key] = data[key]
			# 	# Update internal state
			# 	if(key in INTERNAL_STATE_VARIABLES):
			# 		self.internal_state.data[key] = data[key]

		else:
			pass

		self.update_display.emit()

	def stop(self):

		self.stop_signal_received = True


class TrackingDataSaver(QObject):

	''' 
	Signals and Slots

	Slots:

	enqueue: Adds dataline to queue

	stop_datasaver: Signal from "Acquisition panel".

	set_base_path: Set from "Acquisition panel".

	start_new_experiment: Triggered by "Acquisition panel".

	start_new_track: Triggered by "Track button".

	set_image_name: Signal from ImageSaver object.

	update_imaging_channels: Signal from "Microscope Mode Widget". 
	Only changes when no track is being acquired.

	'''
	signal_start_saving_image = Signal()

	def __init__(self, internal_state):
		QObject.__init__(self)

		self.internal_state = internal_state

		self.base_path = './'
		self.experiment_ID = ''

		self.queueLen = 10

		self.queue = Queue(self.queueLen) # max 10 items in the queue

		self.saveDataNames = SAVE_DATA

		self.saveDataNames_imageChannels = None 

		# Update Data fields with no:of imaging channels
		self.update_imaging_channels()



		self.DataToQueue = {key:[] for key in self.saveDataNames + self.internal_state.data['imaging channels']}

		# self.DataToSave_dict = {key:[] for key in self.saveDataNames + self.internal_state.data['imaging channels']}

		self.DataToSave_dict = None
		self.DataToSave = []

		self.current_image_name = {key:[] for key in self.internal_state.data['imaging channels']}

		# CSV register
		self.csv_register = CSV_Tool.CSV_Register(header = [self.saveDataNames_imageChannels])

		# Use a counter 
		self.counter = 0

		self.stop_signal_received = False
		
		self.thread = Thread(target=self.process_queue)
		self.thread.start()

		self.exp_folder_created = False

	def process_queue(self):
		while True:
			# stop the thread if stop signal is received
			if self.stop_signal_received:
				return
			# process the queue
			try:

				self.DataToSave_dict = self.queue.get(timeout=0.1)

				self.DataToSave = [self.DataToSave_dict[key] for key in self.DataToSave_dict.keys()]

				# print(self.DataToSave)
				# Register the data to a CSV file
				self.csv_register.write_line([self.DataToSave])

				self.counter = self.counter + 1
				self.queue.task_done()
			except:
				# traceback.print_exc()
				# print("Exception:", sys.exc_info()[0])
				pass

	def enqueue(self):

		# print('Placing data in save queue')
		# Get the most recent internal state values
		for key in self.saveDataNames:
			self.DataToQueue[key] = self.internal_state.data[key]

		# Get the most recent image name values
		for key in self.internal_state.data['imaging channels']:
			self.DataToQueue[key] = self.current_image_name[key]
			# Reset the current image name
			self.current_image_name[key] = ''


		try:
			self.queue.put_nowait(self.DataToQueue)

		except:
			'Data queue full, current cycle data not saved'


	# Stop signal from Acquisition Widget
	def stop_DataSaver(self):
		
		# self.queue.join()
		# self.thread.join()
		self.stop_signal_received = True


	def set_base_path(self,path):
		'''
		Base path needs to be set for the data first since we always save metadata even 
		without saving images in "Tracking Mode".

		In "Recording Mode" the base path would be set by the image-saver function
		'''
		self.base_path = path
		# Update internal state
		self.internal_state.data['base_path'] = path

	def start_new_experiment(self,experiment_ID):
		'''
		This is called when a new Acquisition is started.
		'''
		 # @@@ Testing
		print('Starting new experiment...')

		# generate unique experiment ID
		if(self.internal_state.data['Acquisition']==True):

			 # @@@ Testing
			print('Creating folders...')

			self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S')
			

			self.internal_state.data['experiment_ID'] = self.experiment_ID
			
		   

			# create a new folder to hold current experiment data
			try:
				os.mkdir(os.path.join(self.base_path, self.experiment_ID))
				self.exp_folder_created = True
			except:
				pass


			 # Create and store metadata file
			self.create_metadata_file()

		   
		
		# reset the counter
		self.track_counter = 0

		self.start_new_track()

		



	def start_new_track(self):
		'''
		Function is called when the track button is pressed. If 'Acquisition' button is also pressed
		this will save a new track file. Within a given Experiment Acquisition, each track button 
		press creates a new track file.
		'''
		 # @@@ Testing
		print('Starting new track...')

		# If a current track file is open then close it
		self.csv_register.close()

		if(self.internal_state.data['Acquisition']==True and self.exp_folder_created):


			file_name = os.path.join(self.base_path, self.experiment_ID, 'track{:03d}.csv'.format(self.track_counter))

			print(file_name)
			#Update the track counter
			self.track_counter += 1
			
			# If the file doesnt exist then create it
			if not os.path.exists(file_name):                                 #if it is the first time start_tracking is True while start_saving is true we initiate the new file
				self.csv_register.file_directory= file_name
				self.csv_register.start_write()
				print('Created new file {}'.format(file_name))

		else:
			pass


	def create_metadata_file(self):
		config_file = os.path.join(self.base_path, self.experiment_ID, 'metadata.csv')

		df = pd.DataFrame({'Objective':[self.internal_state.data['Objective']], 
					'PixelPermm':[OBJECTIVES[self.internal_state.data['Objective']]['PixelPermm']],'Local time':[datetime.now().strftime('%Y-%m-%d, %H:%M:%S.%f')]})
		
		df.to_csv(config_file)


	# Function sets the image names for all the imaging channels    
	def setImageName(self, image_channel, image_name):

		self.current_image_name[image_channel] = image_name
		

	def update_imaging_channels(self):
		'''
		Call this function to change the number of image name fields. 
		This can only be called when an Acquisition is not in progress.
		'''
		imaging_channels = self.internal_state.data['imaging channels']

		if(self.internal_state.data['Acquisition'] == False):
			self.saveDataNames_imageChannels = self.saveDataNames + [channel for channel in imaging_channels]

			# Update the headers of the CSV register
			self.csv_register = CSV_Tool.CSV_Register(header = [self.saveDataNames_imageChannels])

		else:
			print('Cannot change imaging channels when Acquisition is in progress!')


class ImageSaver(QObject):

	stop_recording = Signal()

	# Image Name Signal (str, str): Imaging Channel, Image Name
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
		 # Start a thread for saving images
		self.thread.start()
		print('Started image saver thread')
		

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
				# print('Processing save image queue...')

				[image,frame_ID,timestamp] = self.queue.get(timeout=0.1)
				self.image_lock.acquire(True)
				folder_ID = int(self.counter/self.max_num_image_per_folder)
				# The file names should be unique for gravity machine
				file_ID = self.counter
				# create a new folder (base_path/imaging_channel/subFolderID/fileID)
				if file_ID == 0 or int(self.counter%self.max_num_image_per_folder)==0:
					folder_images = os.path.join(self.base_path, self.experiment_ID, self.imaging_channel, '{:05d}'.format(folder_ID))
					os.mkdir(folder_images)
				
				image_file_name = '{:07d}'.format(file_ID) + self.image_format

				saving_path = os.path.join(folder_images, image_file_name)
				
				# Emit the image name so DataSaver can save it along with the stage positions
				self.imageName.emit(self.imaging_channel, image_file_name)
				
				# Save the image
				cv2.imwrite(saving_path,image)
				print('Wrote image {} to disk'.format(image_file_name))
				self.counter = self.counter + 1
				self.queue.task_done()
				self.image_lock.release()
			except:
				# traceback.print_exc()
				# print("Exception:", sys.exc_info()[0])
				pass
							
	def enqueue(self,image, frame_ID, timestamp):
		try:
			# print('Placing image in save queue')
			self.queue.put_nowait([image,frame_ID,timestamp])
			# if ( self.recording_time_limit>0 ) and ( time.time()-self.recording_start_time >= self.recording_time_limit ):
			#     self.stop_recording.emit()
			# when using self.queue.put(str_), program can be slowed down despite multithreading because of the block and the GIL
		except:
			print('imageSaver queue is full, image discarded')

		
	
	def set_base_path(self,path = None):
		'''
		Base path needs to be set by the DataSaver first since we always save metadata and timestamps
		even when not tracking
		'''
		if(path is not None):        
			self.base_path = path
			# Update internal state
			self.internal_state.data['base_path'] = path
		else:
			self.base_path = self.internal_state.data['base_path']


	def start_saving_images(self, experiment_ID = None):
		

		self.counter = 0
		self.folder_counter = 0
		self.recording_start_time = 0
		self.recording_time_limit = -1
		

		# Creates the folders for storing images

		if(experiment_ID is not None):
			# generate unique experiment ID
			self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S')
			

			self.internal_state.data['experiment_ID'] = self.experiment_ID
		
			
		else:
			self.experiment_ID = self.internal_state.data['experiment_ID'] 


		print(self.base_path)
		print(self.experiment_ID)
		print(self.imaging_channel)
		# create a new folder for each imaging channel
		os.makedirs(os.path.join(self.base_path, self.experiment_ID, self.imaging_channel))
		print('Created folder for {} channel'.format(self.imaging_channel))

	def set_recording_time_limit(self,time_limit):
		self.recording_time_limit = time_limit

	def stop_saving_images(self):
		# self.queue.join()
		self.stop_signal_received = True
 







			

		   



