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
	set_trackBusy = Signal(int)
	clear_trackBusy = Signal(int)
	save_data_signal = Signal()
	get_roi_bbox = Signal()
	signal_tracking_fps = Signal(int)
	signal_stop_tracking = Signal()

	''' 
	Connection map

	centroid_image -> ImageDisplayer.draw_object
	Rect_pt1_pt2 -> ImageDisplayer.draw_bbox
	multiplex_send_signal -> multiplex_Send
	save_data_signal -> DataSaver

	'''
	def __init__(self, microcontroller, internal_state, rotate_image_angle = 0, color = False):
		QObject.__init__(self)
		self.microcontroller = microcontroller
		self.internal_state = internal_state
		self.units_converter = Units_Converter()

		# Set the reference image width based on the camera sensor size used for calibration
		# This allows physical distances be calculated even if the image res is down-sampled.
		# @@@ This needs to be removed. We should be able to calculate this when the image size changes without
		# knowing the calib image width
		self.units_converter.set_calib_imWidth(CALIB_IMG_WIDTH)
	
		self.image = None

		# Focus Tracker type
		self.track_focus = False
		# For testing
		self.image_tracking_enabled = False
		self.is_first_frame = True
		self.objectFound = False

		self.centroid = None
		self.rect_pts = None

		self.image_setPoint = None
		self.image_center = None
		self.image_width = 720
		self.posError_image = np.array([0,0])
		self.image_offset = np.array([0,0])
		self.rotate_image_angle = rotate_image_angle

		# Create a tracking object that does the image-based tracking
		self.tracker_image = tracking.Tracker_Image(color = color)

		# PID controller for each axis
		self.pid_controller_x = PID.PID()
		self.pid_controller_y = PID.PID()
		self.pid_controller_theta = PID.PID()
		self.resetPID = True

		self.stage_tracking_enabled = False
		self.tracking_frame_counter = 0

		# Deque data length
		self.dequeLen = 20

		#Time
		self.begining_Time = time.time()           #Time begin the first time we click on the start_tracking button
		self.Time = deque(maxlen=self.dequeLen)

		self.focus_error = 0 # for PDAF focus tracking

		self.X_image = deque(maxlen=self.dequeLen)
		self.Z_image = deque(maxlen=self.dequeLen)

		self.X_stage = deque(maxlen=self.dequeLen)
		self.Y_stage = deque(maxlen=self.dequeLen)
		self.Theta_stage = deque(maxlen=self.dequeLen)

		# X, Y, Z represents the physical locations of the object - stage position + object offset in the image 
		self.X = deque(maxlen=self.dequeLen)
		self.Y = deque(maxlen=self.dequeLen)
		self.Z = deque(maxlen=self.dequeLen)

		# Subset of INTERNAL_STATE_MODEL that is updated by Tracking_Controller (self)
		self.internal_state_vars = ['Time','X_image', 'Z_image', 'X', 'Y', 'Z']		

		# For fps measurement
		self.timestamp_last = 0
		self.counter = 0
		self.fps_real = 0

	# called by StreamHandler through its sigal packet_image_for_tracking
	def on_new_frame(self, image, thresholded_image = None):

		self.image = image
		self._update_elapsed_time()
		self._update_image_center_width()
		self.stage_tracking_enabled = self.internal_state.data['stage_tracking_enabled']
		
		# check if it's a new track [or if the object being tracked was lost - removed in this update]
		if self.tracking_frame_counter == 0:
			self.is_first_frame = True
			self.resetPID = True
		else:
			self.is_first_frame = False
		
		# track the object in the image
		self.objectFound, self.centroid, self.rect_pts = self.tracker_image.track(image, thresholded_image, is_first_frame = self.is_first_frame)
		
		# check if tracking object in the image was successful, if not, terminated the track
		if self.objectFound:
			self.tracking_frame_counter += 1
		else:
			# tracking failed, stop tracking and emit the stop_tracking signal
			self.internal_state.data['image_tracking_enabled'] = False
			self.signal_stop_tracking.emit()
			self.reset_track()
			return

		# Find the object's position relative to the tracking set point on the image
		self.posError_image = self.centroid - self.image_setPoint
		# Get the error and convert it to mm
		# x_error, z_error are in mm		

		if(self.rotate_image_angle == 0):
			x_error, z_error = self.units_converter.px_to_mm(self.posError_image[0], self.image_width), self.units_converter.px_to_mm(self.posError_image[1], self.image_width)
		elif(self.rotate_image_angle == 90):
			z_error, x_error = self.units_converter.px_to_mm(self.posError_image[0], self.image_width), self.units_converter.px_to_mm(self.posError_image[1], self.image_width)
		# Flip the sign of Z-error since image coordinates and physical coordinates are reversed.
		# z_error = -z_error

		# get the object location along the optical axis. 
		# Is the object position necessary for this? Alternatively we can pass the centroid
		# and handle this downstream
		if(self.track_focus):
			pass
			'''
			y_error will be set by the y focus tracking controller, 
			which has a reference of this tracking controller, and can set self.focus_error and self.track_focus
			'''
		else:
			self.focus_error = 0

		# Emit the detected centroid position so other widgets can access it.
		self.centroid_image.emit(self.centroid)
		self.Rect_pt1_pt2.emit(self.rect_pts)

		X_stage, Y_stage, Theta_stage = self.internal_state.data['X_stage'], self.internal_state.data['Y_stage'], self.internal_state.data['Theta_stage']
		
		self.update_image_position()
		self.update_stage_position(X_stage, Y_stage, Theta_stage)
		self.update_obj_position()
		# get motion commands
		# Error is in mm.
		# print('Image error: {}, {}, {} mm'.format(x_error, y_error, z_error))

		if self.stage_tracking_enabled:
			X_order, Y_order, Theta_order = self.get_motion_commands(x_error,self.focus_error,z_error)

		# 202109@@@
		''' 
		# New serial interface (send data directly to micro-controller object)
		self.microcontroller.move_x_nonblocking(X_order*STAGE_MOVEMENT_SIGN_X)
		if LIQUID_LENS_FOCUS_TRACKING == False:
			self.microcontroller.move_y_nonblocking(Y_order*STAGE_MOVEMENT_SIGN_Y)
			# when doing focus tracking with liquid lens, because of the potential difference update rate, y_order is sent separately
		self.microcontroller.move_theta_nonblocking(Theta_order*STAGE_MOVEMENT_SIGN_THETA)  
		'''

		# Update the Internal State Model
		self.update_internal_state()

		# Send a signal to the DataSaver module and instruct it to Save Data
		if self.internal_state.data['Acquisition'] == True:
			# print('Sending data for saving...')
			self.save_data_signal.emit()

		self.measure_tracking_fps()

	def measure_tracking_fps(self):
		# measure real fps
		timestamp_now = round(time.time())
		if timestamp_now == self.timestamp_last:
			self.counter = self.counter+1
		else:
			self.timestamp_last = timestamp_now
			self.fps_real = self.counter
			self.counter = 0
			self.signal_tracking_fps.emit(self.fps_real)
			
	# Triggered when you hit image_tracking_enabled
	def reset_track(self):
		# @@@ Testing
		print('resetting track...')
		self.tracking_frame_counter = 0
		self.is_first_frame = True
		self.objectFound = False
		self.tracker_image.reset()

		#Time
		self.begining_Time = time.time()           #Time begin the first time we click on the start_tracking button
		self.Time = deque(maxlen=self.dequeLen)

		self.X_image = deque(maxlen=self.dequeLen)
		self.Z_image = deque(maxlen=self.dequeLen)

		self.X_stage = deque(maxlen=self.dequeLen)
		self.Y_stage = deque(maxlen=self.dequeLen)
		self.Theta_stage = deque(maxlen=self.dequeLen)

		for ii in range(self.dequeLen):
			self.X_stage.append(0)
			self.Y_stage.append(0)
			self.Theta_stage.append(0)

		self.X = deque(maxlen=self.dequeLen)
		self.Y = deque(maxlen=self.dequeLen)
		self.Z = deque(maxlen=self.dequeLen)

	def _update_elapsed_time(self):
		self.Time.append(time.time() - self.begining_Time)

	def update_stage_position(self,x,y,theta):
		self.X_stage.append(x)
		self.Y_stage.append(y)
		self.Theta_stage.append(theta)

	def update_image_position(self):
		# Object position relative to image center (in mm)
		self.X_image.append(self.units_converter.px_to_mm(self.centroid[0] - self.image_center[0], self.image_width))
		self.Z_image.append(self.units_converter.px_to_mm(self.centroid[1] - self.image_center[1], self.image_width))

	def update_obj_position(self):
		self.X.append(self.X_stage[-1] + self.X_image[-1])
		self.Y.append(self.Y_stage[-1])
		if(len(self.Time)>1):
			self.Z.append(self.Z[-1]+(self.Z_image[-1]-self.Z_image[-2]) + self.units_converter.rad_to_mm(self.Theta_stage[-1]-self.Theta_stage[-2],self.X[-1]))
		else:
			self.Z.append(0)

		# @@@ testing 
		# print('Virtual depth :{} mm'.format(round(self.Z[-1], 2)))
	
	def get_motion_commands(self, x_error, y_error, z_error):
		# Take an error signal and pass it through a PID algorithm
		# Convert from mm to steps (these are rounded to the nearest integer).
		x_error_steps = int(Motion.STEPS_PER_MM_X*x_error)
		y_error_steps = int(Motion.STEPS_PER_MM_Y*y_error)
		theta_error_steps = int(self.units_converter.Z_mm_to_step(z_error, self.X_stage[-1]))

		if self.resetPID:
			self.pid_controller_x.initiate(x_error_steps,self.Time[-1]) #reset the PID
			self.pid_controller_y.initiate(y_error_steps,self.Time[-1]) #reset the PID
			self.pid_controller_theta.initiate(theta_error_steps,self.Time[-1]) #reset the PID
			X_order = 0
			Y_order = 0
			Theta_order = 0
			self.resetPID = False

		else:
			X_order = self.pid_controller_x.update(x_error_steps,self.Time[-1])
			X_order = round(X_order,2)

			Y_order = self.pid_controller_y.update(y_error_steps,self.Time[-1])
			# Y_order = y_error_steps #@@@ NonPID focus tracking; may need to reverse the sign - no longer needed, to remove in the next update
			Y_order = round(Y_order,2)

			Theta_order = self.pid_controller_theta.update(theta_error_steps,self.Time[-1])
			Theta_order = round(Theta_order,2)

		return X_order, Y_order, Theta_order

	# Image related functions

	def _update_image_center_width(self):
		if(self.image is not None):
			self.image_center, self.image_width = image_processing.get_image_center_width(self.image)
			self._set_search_area()
			self._update_tracking_setpoint() # The tracking set point is modified since it depends on the image center.
			
	def _update_tracking_setpoint(self):
		if(self.image_center is not None):
			self.image_setPoint = self.image_center + self.image_offset

	def _update_image_offset(self, new_image_offset):
		self.image_offset = new_image_offset
		self._update_tracking_setpoint()

	def update_roi_bbox(self):
		self.get_roi_bbox.emit()

	def _set_search_area(self):
		self.tracker_image.searchArea = int(self.image_width/Tracking.SEARCH_AREA_RATIO)
		# print('current search area : {}'.format(self.tracker_image.searchArea))

	def set_cropped_image_size(self, new_ratio):
		pass

	def get_latest_attr_value(self, key):
		temp = getattr(self, key)
		return temp[-1]

	def update_internal_state(self):
		for key in self.internal_state_vars:
			if(key in INTERNAL_STATE_VARIABLES):
				self.internal_state.data[key] = self.get_latest_attr_value(key)
			else:
				print('>>>>>>' + key)
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
		print(self.data)
		
class microcontroller_Receiver(QObject):
	'''
	Receives data from microcontroller and updates the Internal state variables to the latest value
	Connection Map:
	StreamHandler (rec new image) -> getData_microcontroller
	'''
	update_stage_position = Signal(float, float, float)
	update_homing_state = Signal()
	start_tracking_signal = Signal()
	update_plot = Signal()

	def __init__(self, microcontroller, internal_state):
		QObject.__init__(self)

		self.microcontroller = microcontroller
		self.internal_state = internal_state
		self.units_converter = Units_Converter()
		self.readings_from_MCU = {key:[] for key in READINGS_FROM_MCU}

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
		self.theta_pos = 0

	def getData_microcontroller(self):
		# data = self.microcontroller.read_received_packet_nowait()
		data = None

		if(data is not None):
			# Parse the data
			if(data[0] == ord('M')):

				phase = byte_operations.data2byte_to_int(data[1], data[2])*2*np.pi/65535.

				if(MicrocontrollerDef.RUN_OPENLOOP == True):
					# X stage position (mm)					
					self.x_pos = byte_operations.unsigned_to_signed(data[3:6],MicrocontrollerDef.N_BYTES_POS)/(Motion.STEPS_PER_MM_X*Motion.MAX_MICROSTEPS) 
					# Y stage position (mm)
					self.y_pos = byte_operations.unsigned_to_signed(data[6:9],MicrocontrollerDef.N_BYTES_POS)/(Motion.STEPS_PER_MM_Y*Motion.MAX_MICROSTEPS)
					# Theta stage position (encoder counts to radians)
					self.theta_pos = 2*np.pi*byte_operations.unsigned_to_signed(data[9:12],MicrocontrollerDef.N_BYTES_POS)/(Motion.STEPS_PER_REV_THETA_SHAFT*Motion.MAX_MICROSTEPS) 

				elif(MicrocontrollerDef.RUN_OPENLOOP == False):
					# X stage position (mm)
					self.x_pos = X_ENCODER_SIGN*byte_operations.unsigned_to_signed(data[3:6],MicrocontrollerDef.N_BYTES_POS)/(Encoders.COUNTS_PER_MM_X) 
					# Y stage position (mm)
					self.y_pos = Y_ENCODER_SIGN*byte_operations.unsigned_to_signed(data[6:9],MicrocontrollerDef.N_BYTES_POS)/(Encoders.COUNTS_PER_MM_Y)
					# Theta stage position (encoder counts to radians)
					self.theta_pos = THETA_ENCODER_SIGN*2*np.pi*byte_operations.unsigned_to_signed(data[9:12],MicrocontrollerDef.N_BYTES_POS)/(Encoders.COUNTS_PER_REV_THETA) 

				self.readings_from_MCU['FocusPhase'] = phase
				self.readings_from_MCU['X_stage'] = self.x_pos
				self.readings_from_MCU['Y_stage'] = self.y_pos
				self.readings_from_MCU['Theta_stage'] = self.theta_pos

				self.update_stage_position.emit(self.x_pos, self.y_pos, self.theta_pos)

				for key in READINGS_FROM_MCU:
					if(key in INTERNAL_STATE_VARIABLES):
						self.internal_state.data[key] = self.readings_from_MCU[key]

			elif(data[0] == ord('F')):
				# print('Flag recvd')
				if(data[1] == ord('S')):
					# print('Automated stage tracking flag recvd: {}'.format(data[2]))
					self.internal_state.data['stage_tracking_enabled'] = data[2]
					
				elif(data[1] == ord('H')):
					print('Homing flag state recvd: {}'.format(data[2]))
					if(data[2] == 0):
						self.internal_state.data['homing-state'] = 'not-complete'

					elif(data[2] == 1):
						self.internal_state.data['homing-state'] = 'in-progress'
					elif(data[2] == 2):
						self.internal_state.data['homing-state'] = 'complete'
					
					print('Homing state: {}'.format(self.internal_state.data['homing-state']))

				elif(data[1] == ord('T')):
					print('Toggle image tracking signal recvd: {}'.format(data[2]))

					self.internal_state.data['enable_image_tracking_from_hardware_button'] = not(self.internal_state.data['enable_image_tracking_from_hardware_button'])
				
					self.start_tracking_signal.emit()


				elif(data[1] == ord('F')):
					print('Focus tracking flag changed in uController: {}'.format(data[2]))

				elif(data[1] == ord('C')):
					pass
					# print('Camera trigger flag changed: {}'.format(data[2]))
			# Send update plot signal
			self.update_plot.emit()
		else:
			pass

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
				# print('Datasaver stopped... returning')
				return
			# process the queue
			try:
				self.DataToSave_dict = self.queue.get(timeout=0.1)
				self.DataToSave = [self.DataToSave_dict[key] for key in self.DataToSave_dict.keys()]
				# print(self.DataToSave)
				# Register the data to a CSV file
				self.csv_register.write_line([self.DataToSave])
				# print('Wrote data to CSV file')
				self.counter = self.counter + 1
				self.queue.task_done()
			except:
				# traceback.print_exc()
				# print("Exception:", sys.exc_info()[0])
				pass

	def enqueue(self):
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
			# print('Placing data in save queue')
		except:
			print('Data queue full, current cycle data not saved')

	def close(self):
		# self.queue.join()
		self.stop_signal_received = True
		self.thread.join()



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

	def update_experiment_ID(self,experiment_ID):
		# temporary solution for the volumetric recording to use the experiment ID entered in the GUI
		self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S')
		self.internal_state.data['experiment_ID'] = self.experiment_ID
		print('[update the experiment ID to ' + self.experiment_ID + ']')

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
				# Set the stop_signal flag so data saving can begin. 
				if(self.stop_signal_received == True):
					self.stop_signal_received = False
					print('Starting data saver again...')
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
	def __init__(self, internal_state, imaging_channel = None, image_format='.tif', rotate_image_angle = 0, flip_image = None):
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

		self.rotate_image_angle = rotate_image_angle
		self.flip_image = flip_image
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
				
				# Image rotations
				if(self.rotate_image_angle != 0):
					# ROTATE_90_CLOCKWISE
					# ROTATE_90_COUNTERCLOCKWISE
					if(self.rotate_image_angle == 90):
						image = cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
					elif(self.rotate_image_angle == -90):
						image = cv2.rotate(image,cv2.ROTATE_90_COUNTERCLOCKWISE)
					elif(self.rotate_image_angle == 180):
						image = cv2.rotate(image, cv2.ROTATE_180)

				if(self.flip_image is not None):
					# flipcode = 0: flip vertically
					# flipcode > 0: flip horizontally
					# flipcode < 0: flip vertically and horizontally
					if(self.flip_image == 'Vertical'):
						image = cv2.flip(image, 0)
					elif(self.flip_image == 'Horizontal'):
						image = cv2.flip(image, 1)
					elif(self.flip_image == 'Both'):
						image = cv2.flip(image, -1)

				# Save the image
				cv2.imwrite(saving_path,image)
				# print('Wrote image {} to disk'.format(image_file_name))
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

	def close(self):
		self.queue.join()
		self.stop_signal_received = True
		self.thread.join()
 