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
	signal_update_plots = Signal()

	''' 
	Connection map

	centroid_image -> ImageDisplayer.draw_object
	Rect_pt1_pt2 -> ImageDisplayer.draw_bbox
	multiplex_send_signal -> multiplex_Send
	save_data_signal -> DataSaver

	'''
	def __init__(self, navigationController, microcontroller, internal_state, color = False):
		QObject.__init__(self)
		self.navigationController = navigationController
		self.microcontroller = microcontroller
		self.internal_state = internal_state
		self.image = None

		# Focus Tracker type
		self.track_focus = False
		# For testing
		self.image_tracking_enabled = False
		self.objectFound = False

		self.centroid = None
		self.rect_pts = None

		self.tracking_setpoint_image = None
		self.image_center = None
		self.image_width = None
		self.image_offset = np.array([0,0])

		# Create a tracking object that does the image-based tracking
		self.tracker_image = tracking.Tracker_Image(color = color)

		# PID controller for each axis
		self.pid_controller_x = PID.PID()
		self.pid_controller_y = PID.PID()
		self.pid_controller_z = PID.PID()

		self.stage_tracking_enabled = None
		self.tracking_frame_counter = None

		#Time
		self.t0 = time.time()           #Time begin the first time we click on the start_tracking button
		self.Time = None

		self.focus_error = 0 # for focus tracking; this variable is accessed by other objects

		self.X_image = None # unit: mm
		self.Y_image = None # unit: mm
		self.Z_image = None # unit: mm
		
		self.X_stage = None # unit: mm
		self.Y_stage = None # unit: mm
		self.Z_stage = None # unit: mm
		self.Theta_stage = None # unit: rad

		# X, Y, Z represents the physical locations of the object - stage position + object offset in the image 
		self.X = None # unit: mm
		self.Y = None # unit: mm
		self.Z = None # unit: mm

		self.current_radius = None # unit: mm

		# Subset of INTERNAL_STATE_MODEL that is updated by Tracking_Controller (self)
		self.internal_state_vars = ['Time','X_image', 'Z_image', 'X', 'Y', 'Z']		

		# For fps measurement
		self.timestamp_last = 0
		self.counter = 0
		self.fps_real = 0

		self.pixel_size_um_raw = None
		self.image_resizing_factor = None
		self.pixel_size_um_scaled = None

	# called by StreamHandler through its sigal packet_image_for_tracking
	def on_new_frame(self, image, thresholded_image = None):

		self.image = image
		self.Time = time.time() - self.t0 # update elapsed time
		self._update_image_center_width()
		self.stage_tracking_enabled = self.internal_state.data['stage_tracking_enabled']
		
		# check if it's a new track [or if the object being tracked was lost - removed in this update]
		if self.tracking_frame_counter == 0:
			is_first_frame = True
		else:
			is_first_frame = False

		# get stage position - to add z
		self._get_stage_position(is_first_frame=is_first_frame) 
		# note that for XTheta-Y tracking, Z_stage is calculated with the previous self.current_radius
		# which is available post first frame (for the first frame, set z = 0)
		
		# track the object in the image
		self.objectFound, self.centroid, self.rect_pts = self.tracker_image.track(image, thresholded_image, is_first_frame = is_first_frame)
		
		# check if tracking object in the image was successful, if not, terminated the track
		if self.objectFound:
			self.tracking_frame_counter += 1
		else:
			# tracking failed, stop tracking and emit the stop_tracking signal
			self.internal_state.data['image_tracking_enabled'] = False
			self.signal_stop_tracking.emit()
			return

		# emit the detected object position for display
		self.centroid_image.emit(self.centroid)
		self.Rect_pt1_pt2.emit(self.rect_pts)

		# find the object's position relative to the tracking set point on the image
		in_plane_position_error_pixel = self.centroid - self.tracking_setpoint_image
		in_plane_position_error_mm = in_plane_position_error_pixel*self.pixel_size_um_scaled/1000

		# assign in-plane position error based on the tracking configuration
		if TRACKING_CONFIG == 'XTheta_Y' or TRACKING_CONFIG == 'XZ_Y':
			x_error_mm = in_plane_position_error_mm[0]
			z_error_mm = in_plane_position_error_mm[1]
			# self.focus_error is updated by the focus tracking controller
			y_error_mm = self.focus_error
			# get position of the object in the image
			self.X_image = (self.centroid[0]-self.image_center[0])*self.pixel_size_um_scaled/1000
			self.Z_image = (self.centroid[1]-self.image_center[1])*self.pixel_size_um_scaled/1000
			# get position of the object in the lab frame
			self.X = self.X_stage + self.X_image
			self.Y = self.Y_stage # can include the offset calculated from focus tracking controller later
			self.Z = self.Z_stage + self.Z_image
			if TRACKING_CONFIG == 'XTheta_Y':
				self.current_radius = Chamber.R_HOME + self.X
		elif TRACKING_CONFIG == 'XY_Z':
			x_error_mm = in_plane_position_error_mm[0]
			y_error_mm = in_plane_position_error_mm[1]
			z_error_mm = self.focus_error
			# self.focus_error is updated by the focus tracking controller
			# get position of the object in the image
			self.X_image = (self.centroid[0]-self.image_center[0])*self.pixel_size_um_scaled/1000
			self.Y_image = (self.centroid[1]-self.image_center[1])*self.pixel_size_um_scaled/1000
			# get position of the object in the lab frame 
			self.X = self.X_stage + self.X_image
			self.Y = self.Y_stage + self.Y_image
			self.Z = self.Z_stage # can include the offset calculated from focus tracking controller later
		
		# stage tracking
		if self.stage_tracking_enabled:
			
			# get PID calculation result
			x_correction_mm,y_correction_mm,z_correction_mm = self._get_PID_feedback(x_error_mm,y_error_mm,z_error_mm,is_first_frame)
			
			# get motion commands
			x_correction_usteps = int(x_correction_mm/(SCREW_PITCH_X_MM/FULLSTEPS_PER_REV_X/self.navigationController.x_microstepping))
			y_correction_usteps = int(y_correction_mm/(SCREW_PITCH_Y_MM/FULLSTEPS_PER_REV_Y/self.navigationController.y_microstepping))
			if TRACKING_CONFIG == 'XTheta_Y':
				z_correction_theta = z_correction_mm/self.current_radius 
				z_correction_usteps = int(z_correction_theta/(2*np.pi/GEAR_RATIO_THETA/FULLSTEPS_PER_REV_THETA/self.navigationController.theta_microstepping))
			else:
				z_correction_usteps = int(z_correction_mm/(SCREW_PITCH_Z_MM/FULLSTEPS_PER_REV_Z/self.navigationController.z_microstepping))
		
			# send motion commands
			if TRACKING_CONFIG == 'XY_Z':
				self.microcontroller.move_x_usteps(x_correction_usteps)
				self.microcontroller.move_y_usteps(y_correction_usteps) 
				self.microcontroller.move_z_usteps(z_correction_usteps) # can move to the focus tracking controller
			elif TRACKING_CONFIG == 'XZ_Y':
				self.microcontroller.move_x_usteps(x_correction_usteps) # in-plane axis 0
				self.microcontroller.move_y_usteps(z_correction_usteps)	# in-plane axis 1
				self.microcontroller.move_z_usteps(y_correction_usteps) # focus axis - can move to the focus tracking controller
			elif TRACKING_CONFIG == 'XTheta_Y':
				self.microcontroller.move_x_usteps(x_correction_usteps) # in-plane axis 0
				self.microcontroller.move_y_usteps(z_correction_usteps) # in-plane axis 1
				self.microcontroller.move_z_usteps(y_correction_usteps) # focus axis - can move to the focus tracking controller

		# update the internal states
		self.update_internal_state()

		# update plots
		self.signal_update_plots.emit()

		# send a signal to the DataSaver module and instruct it to Save Data
		if self.internal_state.data['Acquisition'] == True:
			self.save_data_signal.emit()

		self._measure_tracking_fps()

	def _measure_tracking_fps(self):
		# measure real fps
		timestamp_now = round(time.time())
		if timestamp_now == self.timestamp_last:
			self.counter = self.counter+1
		else:
			self.timestamp_last = timestamp_now
			self.fps_real = self.counter
			self.counter = 0
			self.signal_tracking_fps.emit(self.fps_real)

	def _get_stage_position(self,is_first_frame):
		self.X_stage = self.internal_state.data['X_stage']
		self.Y_stage = self.internal_state.data['Y_stage']
		if TRACKING_CONFIG == 'XTheta_Y':
			if is_first_frame:
				self.Z_stage = 0
				self.Theta_stage = self.internal_state.data['Theta_stage']
			else:
				delta_theta = self.internal_state.data['Theta_stage'] - self.Theta_stage
				self.Theta_stage = self.internal_state.data['Theta_stage']
				self.Z_stage = self.Z_stage + self.current_radius*delta_theta
		else:
			self.Z_stage = self.internal_state.data['Z_stage']

	def _get_PID_feedback(self,x_error_mm,y_error_mm,z_error_mm,is_first_frame):
		if is_first_frame:
			self.pid_controller_x.initialize(x_error_mm,self.Time)
			self.pid_controller_y.initialize(y_error_mm,self.Time)
			self.pid_controller_z.initialize(z_error_mm,self.Time)
			x_correction_mm = 0
			y_correction_mm = 0
			z_correction_mm = 0
		else:
			x_correction_mm = self.pid_controller_x.update(x_error_mm,self.Time)
			y_correction_mm = self.pid_controller_y.update(y_error_mm,self.Time)
			z_correction_mm = self.pid_controller_z.update(z_error_mm,self.Time)
		return x_correction_mm,y_correction_mm,z_correction_mm
			
	# called before a new track is started
	def reset_track(self):
		self.tracking_frame_counter = 0
		self.objectFound = False
		self.tracker_image.reset()
		self.t0 = time.time()		
	
	# Image related functions
	def _update_image_center_width(self):
		if(self.image is not None):
			self.image_center, self.image_width = image_processing.get_image_center_width(self.image)
			self._set_search_area()
			self._update_tracking_setpoint() # The tracking set point is modified since it depends on the image center.
			
	def _update_tracking_setpoint(self):
		if(self.image_center is not None):
			self.tracking_setpoint_image = self.image_center + self.image_offset

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
				self.internal_state.data[key] = getattr(self,key)
				# print(key + ': ' + str(getattr(self,key)))
			else:
				print('>>>>>>' + key)
				raise NameError('Key not found in Internal State')

	def send_focus_tracking(self, focus_tracking_flag):
		self.microcontroller.send_focus_tracking_command(focus_tracking_flag)

	def update_pixel_size(self, pixel_size_um):
		self.pixel_size_um = pixel_size_um

	def update_image_resizing_factor(self,image_resizing_factor):
		self.image_resizing_factor = image_resizing_factor
		print('update tracking image resizing factor to ' + str(self.image_resizing_factor))
		self.pixel_size_um_scaled = self.pixel_size_um/self.image_resizing_factor


class StateUpdater(QObject):

	signal_joystick_button_pressed = Signal()
	signal_stage_tracking_status_changed = Signal()

	def __init__(self,navigationController,internal_state):
		QObject.__init__(self)
		self.navigationController = navigationController
		self.internal_state = internal_state

	# call back function
	def read_microcontroller(self,microcontroller):
		# get the stage positions in usteps or encoder counts
		x_pos, y_pos, z_pos, _ = microcontroller.get_pos()

		# assign the readings to the correct axes and convert the unit to mm or rad
		# x axis is the same across configurations
		if USE_ENCODER_X:
			x_pos_mm = x_pos*STAGE_POS_SIGN_X*ENCODER_STEP_SIZE_X_MM
		else:
			x_pos_mm = x_pos*STAGE_POS_SIGN_X*(SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X))
		self.internal_state.data['X_stage'] = x_pos_mm
		self.navigationController.signal_x_mm.emit(x_pos_mm)

		# Y/Z or Y/Theta depend on the microscope configuration
		# XY_Z
		if TRACKING_CONFIG == 'XY_Z':
			# Y axis
			if USE_ENCODER_Y:
				y_pos_mm = y_pos*STAGE_POS_SIGN_Y*ENCODER_STEP_SIZE_Y_MM
			else:
				y_pos_mm = y_pos*STAGE_POS_SIGN_Y*(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.internal_state.data['Y_stage'] = y_pos_mm
			self.navigationController.signal_y_mm.emit(y_pos_mm)
			# Z axis (focus axis)
			if USE_ENCODER_Z:
				z_pos_mm = z_pos*STAGE_POS_SIGN_Z*ENCODER_STEP_SIZE_Z_MM
			else:
				z_pos_mm = z_pos*STAGE_POS_SIGN_Z*(SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z))
			self.internal_state.data['Z_stage'] = z_pos_mm
			self.navigationController.signal_z_mm.emit(z_pos_mm)
		# XZ_Y
		elif TRACKING_CONFIG == 'XZ_Y':
			# Z axis
			if USE_ENCODER_Z: # microcontroller y-axis is connected to the z-stage
				z_pos_mm = y_pos*STAGE_POS_SIGN_Z*ENCODER_STEP_SIZE_Z_MM 
			else:
				z_pos_mm = y_pos*STAGE_POS_SIGN_Z*(SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z))
			self.internal_state.data['Z_stage'] = z_pos_mm
			self.navigationController.signal_z_mm.emit(z_pos_mm)
			# Y axis (focus axis)
			if USE_ENCODER_Y: # microcontroller z-axis is connected to the y-stage
				y_pos_mm = z_pos*STAGE_POS_SIGN_Y*ENCODER_STEP_SIZE_Y_MM
			else:
				y_pos_mm = z_pos*STAGE_POS_SIGN_Y*(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.internal_state.data['Y_stage'] = y_pos_mm
			self.navigationController.signal_y_mm.emit(y_pos_mm)
		# XTheta_Y
		elif TRACKING_CONFIG == 'XTheta_Y':
			# Y axis (focus axis)
			if USE_ENCODER_Y: # microcontroller z-axis is connected to the y-stage
				y_pos_mm = z_pos*STAGE_POS_SIGN_Y*ENCODER_STEP_SIZE_Y_MM
			else:
				y_pos_mm = z_pos*STAGE_POS_SIGN_Y*(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.internal_state.data['Y_stage'] = y_pos_mm
			self.navigationController.signal_y_mm.emit(y_pos_mm)
			# Theta axis
			if USE_ENCODER_THETA:
				theta_pos_rad = y_pos*STAGE_POS_SIGN_THETA*ENCODER_STEP_SIZE_THETA/GEAR_RATIO_THETA
			else:
				theta_pos_rad = y_pos*STAGE_POS_SIGN_THETA*(2*np.pi)/(FULLSTEPS_PER_REV_THETA*self.navigationController.theta_microstepping*GEAR_RATIO_THETA)
			self.internal_state.data['Theta_stage'] = theta_pos_rad
			self.navigationController.signal_theta_degree.emit(theta_pos_rad*360/(2*np.pi))

		# read push-buttons and switches
		if microcontroller.signal_joystick_button_pressed_event:
			self.signal_joystick_button_pressed.emit()
			print('joystick button pressed')
			microcontroller.signal_joystick_button_pressed_event = False
		if USE_HARDWARE_SWITCH:
			if microcontroller.switch_state != self.internal_state.data['stage_tracking_enabled']:
				self.internal_state.data['stage_tracking_enabled'] = microcontroller.switch_state
				self.signal_stage_tracking_status_changed.emit()

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
 