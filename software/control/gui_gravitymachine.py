# set QT_API environment variable
import os 
import sys
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import pyqtgraph.dockarea as dock

QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True) #enable highdpi scaling
QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True) #use highdpi icons

# Definitions
from control._def import *
# app specific libraries
import control.widgets as widgets
import control.widgets_tracking as widgets_tracking
import control.camera_TIS as camera_TIS
import control.camera as camera_Daheng
import control.core as core
import control.core_tracking as core_tracking
import control.microcontroller as microcontroller

# SIMULATION = True

class GravityMachine_GUI(QMainWindow):

	def __init__(self, simulation = False, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		self.setWindowTitle('Gravity Machine v2.0')

		self.imaging_channels = CAMERAS.keys()

		print('Available imaging channels: {}'.format(self.imaging_channels))

		#------------------------------------------------------------------
		# load other windows
		#------------------------------------------------------------------
		self.imageDisplayWindow = {}
		for key in self.imaging_channels:

			if(CAMERAS[key]['make']=='TIS'):
				self.imageDisplayWindow[key] = core.ImageDisplayWindow(key + ' Display', 
					DrawCrossHairs = True) 
			elif (CAMERAS[key]['make']=='Daheng'):
				self.imageDisplayWindow[key] = core.ImageDisplayWindow(key + ' Display', 
					DrawCrossHairs = True, rotate_image_angle=90) 

		
		self.imageDisplayWindow_ThresholdedImage = core.ImageDisplayWindow('Thresholded Image')
		
		#------------------------------------------------------------------
		# Load objects
		#------------------------------------------------------------------
		# cameras/image streams
		self.camera = {}
		if simulation is True:
			# Define a camera object for each unique image-stream.
			self.camera = {key:camera_Daheng.Camera_Simulation() for key in self.imaging_channels}
			# self.microcontroller = microcontroller.Microcontroller_Simulation()
			self.microcontroller = microcontroller.Microcontroller()

		else:
			# TIS Camera object
			for key in self.imaging_channels:

				if(CAMERAS[key]['make']=='TIS'):
					self.camera[key] = camera_TIS.Camera(serial=CAMERAS[key]['serial'], width = CAMERAS[key]['px_format'][0], 
						height = CAMERAS[key]['px_format'][1], framerate = CAMERAS[key]['fps'], color = CAMERAS[key]['is_color'])
				elif (CAMERAS[key]['make']=='Daheng'):
					self.camera[key] = camera_Daheng.Camera(sn = CAMERAS[key]['serial'])

			# DaHheng camera object
			# self.camera = {key:camera.Camera() for key in self.imaging_channels}
			self.microcontroller = microcontroller.Microcontroller()
		

		# Image stream handler
		self.streamHandler = {}
		for key in self.imaging_channels:
			if(CAMERAS[key]['make']=='TIS'):
				if(key == 'DF1'):
					self.streamHandler[key] = core.StreamHandler(camera = self.camera[key], imaging_channel = key, flip_image = 'Vertical')
				else:
					self.streamHandler[key] = core.StreamHandler(camera = self.camera[key], crop_width = CAMERAS[key]['px_format'][0], crop_height= CAMERAS[key]['px_format'][1], imaging_channel = key, flip_image = 'Vertical')

			else:
				self.streamHandler[key] = core.StreamHandler(camera = self.camera[key], crop_width = CAMERAS[key]['px_format'][0], crop_height= CAMERAS[key]['px_format'][1], imaging_channel = key, flip_image = 'Vertical')


		self.internal_state = core_tracking.InternalState()
		#-----------------------------------------------------------------------------------------------
		# Tracking-related objects
		#-----------------------------------------------------------------------------------------------		
		self.liveController = {key:core.LiveController(self.camera[key],self.microcontroller) for key in self.imaging_channels}
		self.navigationController = core.NavigationController(self.microcontroller)
		self.trackingController = core_tracking.TrackingController(self.microcontroller,self.internal_state)
		self.trackingDataSaver = core_tracking.TrackingDataSaver(self.internal_state)
		self.microcontroller_Rec = core_tracking.microcontroller_Receiver(self.microcontroller, self.internal_state) # Microcontroller Receiver object
		#-----------------------------------------------------------------------------------------------
		# Define an ImageSaver, and Image Display object for each image stream
		#-----------------------------------------------------------------------------------------------
		self.imageSaver = {}
		
		for key in self.imaging_channels:
			if CAMERAS[key]['is_color'] == False:
				self.imageSaver[key] = core_tracking.ImageSaver(self.internal_state, imaging_channel = key, image_format = '.tif', rotate_image_angle = 180)
			else:
				self.imageSaver[key] = core_tracking.ImageSaver(self.internal_state, imaging_channel = key, image_format = '.bmp', rotate_image_angle = 180)


		self.imageDisplay = {key: core.ImageDisplay() for key in self.imaging_channels}

		# Open the camera
		# camera start streaming
		for channel in self.imaging_channels:
			self.camera[channel].open()
			self.camera[channel].set_software_triggered_acquisition() #self.camera.set_continuous_acquisition()
			self.camera[channel].set_callback(self.streamHandler[channel].on_new_frame)
			self.camera[channel].enable_callback()
		#------------------------------------------------------------------
		# load widgets
		#------------------------------------------------------------------
		self.cameraSettingsWidget = {key: widgets.CameraSettingsWidget(self.camera[key],self.liveController[key]) for key in self.imaging_channels}
		self.liveControlWidget = widgets.LiveControlWidget(self.streamHandler[TRACKING],self.liveController, self.internal_state)
		self.navigationWidget = widgets_tracking.NavigationWidget(self.navigationController, self.internal_state)
		self.trackingControlWidget = widgets_tracking.TrackingControllerWidget(self.streamHandler[TRACKING], self.trackingController, self.trackingDataSaver, self.internal_state, self.imageDisplayWindow[TRACKING], self.microcontroller)
		self.PID_Group_Widget = widgets_tracking.PID_Group_Widget(self.trackingController)
		self.FocusTracking_Widget = widgets_tracking.FocusTracking_Widget(self.trackingController, self.internal_state, self.microcontroller)
		self.recordingControlWidget = widgets.RecordingWidget(self.streamHandler,self.imageSaver, self.internal_state, self.trackingDataSaver, self.imaging_channels)

		self.stageCalibrationWidget = widgets_tracking.StageCalibrationWidget(self.internal_state, self.microcontroller) 
		self.plotWidget = widgets.dockAreaPlot(self.internal_state)

		
		self.liveSettings_Tab = QTabWidget()
		self.liveSettings_Tab.addTab(self.liveControlWidget, 'Live controller')
		for key in self.imaging_channels:
			self.liveSettings_Tab.addTab(self.cameraSettingsWidget[key],key)

		
		# self.trackingControl_Tab = QTabWidget()
		# self.trackingControl_Tab.addTab(self.trackingControlWidget, 'Tracking')
		# self.trackingControl_Tab.addTab(self.PID_Group_Widget, 'PID')
		# self.trackingControl_Tab.setTabPosition(QTabWidget.North)

		self.SettingsTab = QTabWidget()
		self.SettingsTab.addTab(self.PID_Group_Widget, 'PID')
		self.SettingsTab.addTab(self.FocusTracking_Widget, 'Liquid Lens')
		self.SettingsTab.addTab(self.navigationWidget, 'Navigation')
		self.SettingsTab.addTab(self.stageCalibrationWidget, 'Calibration')
		self.SettingsTab.addTab(self.plotWidget, 'Plots')

		#------------------------------------------------------------------
		# Connections
		#------------------------------------------------------------------
		# Connections that involve all image streams
		for channel in self.imaging_channels:

			self.streamHandler[channel].signal_new_frame_received.connect(self.liveController[channel].on_new_frame)

			self.streamHandler[channel].image_to_display.connect(self.imageDisplayWindow[channel].display_image)
			self.streamHandler[channel].packet_image_to_write.connect(self.imageSaver[channel].enqueue)
			# self.imageDisplay[channel].image_to_display.connect(self.imageDisplayWindow[channel].display_image) # may connect streamHandler directly to imageDisplayWindow
			self.imageSaver[channel].imageName.connect(self.trackingDataSaver.setImageName)

			self.streamHandler[channel].signal_fps_save.connect(self.recordingControlWidget.update_save_fps)


		# Connections that involve only the tracking image stream
		self.streamHandler[TRACKING].thresh_image_to_display.connect(self.imageDisplayWindow_ThresholdedImage.display_image)
		self.streamHandler[TRACKING].packet_image_for_tracking.connect(self.trackingController.on_new_frame)
		# @@@ Currently the resolution-scaling only controls the TRACKING stream
		self.streamHandler[TRACKING].signal_working_resolution.connect(self.liveControlWidget.update_working_resolution)
		# Only display the image-display rate of the main/tracking image stream
		self.streamHandler[TRACKING].signal_fps_display.connect(self.liveControlWidget.update_display_fps)
		# self.streamHandler[TRACKING].signal_fps.connect(self.liveControlWidget.update_stream_fps)
		# self.trackingController.centroid_image.connect(self.imageDisplayWindow[TRACKING].draw_circle)
		self.trackingController.Rect_pt1_pt2.connect(self.imageDisplayWindow[TRACKING].draw_rectangle)
		self.trackingController.save_data_signal.connect(self.trackingDataSaver.enqueue)
		self.trackingController.signal_tracking_fps.connect(self.liveControlWidget.update_stream_fps)

		# Connections for all image-streams
		for channel in self.imaging_channels:
			self.streamHandler[channel].signal_fps.connect(self.cameraSettingsWidget[channel].update_stream_fps)
		# Connect roi from ImageDisplayWindow to TrackingController.
		self.trackingController.get_roi_bbox.connect(self.imageDisplayWindow[TRACKING].send_bbox)
		self.imageDisplayWindow[TRACKING].roi_bbox.connect(self.trackingController.tracker_image.set_roi_bbox)
		# self.microcontroller_Rec.update_display.connect(self.navigationWidget.update_display)
		self.trackingControlWidget.show_roi.connect(self.imageDisplayWindow[TRACKING].toggle_ROI_selector)
		self.microcontroller_Rec.update_stage_position.connect(self.navigationWidget.update_display)
		self.microcontroller_Rec.start_tracking_signal.connect(self.trackingControlWidget.handle_hardware_track_signal)
		self.recordingControlWidget.start_tracking_signal.connect(self.trackingControlWidget.trigger_track_button)
		# self.microcontroller_Rec.update_stage_position.connect(self.trackingController.update_stage_position)
		# Pixel per mm update due to objective change
		self.liveControlWidget.new_pixelpermm.connect(self.trackingController.units_converter.update_pixel_size)
		self.microcontroller_Rec.update_plot.connect(self.plotWidget.update_plots)

		# Dock area for displaying image-streams
		self.image_window = QMainWindow()
		image_display_dockArea = dock.DockArea()
		self.image_window.setCentralWidget(image_display_dockArea)

		self.image_window.setWindowTitle('Image display')

		image_window_docks = dict()

		last_channel = None
		for channel in self.imaging_channels:

			image_window_docks[channel] = dock.Dock(channel, autoOrientation = False)
			image_window_docks[channel].setTitle(channel)
			image_window_docks[channel].showTitleBar()
			print(image_window_docks[channel].title())
			# image_window_docks[channel].setOrientation(o = 'vertical', force = True)
			# image_window_docks[channel].setStretch(x = 1000, y= 1000)
			image_display_dockArea.addDock(image_window_docks[channel], 'bottom')


			image_window_docks[channel].addWidget(self.imageDisplayWindow[channel].widget)

			last_channel = channel

		# Add dock for the thresholded image
		thresholded_image_dock = dock.Dock('Thresholded', autoOrientation = False)
		image_display_dockArea.addDock(thresholded_image_dock, 'right', image_window_docks[last_channel])
		thresholded_image_dock.addWidget(self.imageDisplayWindow_ThresholdedImage.widget)
		#-----------------------------------------------------
		# Layout widgets
		#-----------------------------------------------------
		layout_right = QGridLayout() #layout = QStackedLayout()
		# layout.addWidget(self.cameraSettingsWidget,0,0)
		layout_right.addWidget(self.liveSettings_Tab,0,0)
		layout_right.addWidget(self.trackingControlWidget,1,0)
		layout_right.addWidget(self.SettingsTab,1,1)
		layout_right.addWidget(self.recordingControlWidget,0,1)

		# layout.addWidget(self.cameraSettings_Tab,0,1)
		# layout.addWidget(self.PID_Group_Widget,1,1)
		overall_layout = QHBoxLayout()
		# overall_layout.addWidget(image_display_dockArea)
		overall_layout.addLayout(layout_right)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(overall_layout)
		self.setCentralWidget(self.centralWidget)

		# Start all image-streams
		print('Starting image streams')
		# self.start_imageStreams()
		for channel in self.imaging_channels:
			self.camera[channel].start_streaming()
		print('Started image streams!')

		self.image_window.show()

	
	def show_image_window(self, channel):
		pass

	def start_imageStreams(self):
		for key in self.imaging_channels:
			self.camera[key].start_streaming()

	# @@@ TO DO
	def add_status_bar(self):
		# Status bar at the bottom
		self.statusBar = QStatusBar()
		self.homing_status = QLabel('Homing status:')
		self.stage_control = QLabel('Stage control:')

		self.statusBar.addPermanentWidget(self.homing_status)
		self.statusBar.addPermanentWidget(self.stage_control)

	# @@@ TO DO
	def update_status_bar(self):
		pass

	def closeEvent(self, event):

		reply = QMessageBox.question(self, 'Message',
			"Are you sure you want to exit?", QMessageBox.Yes | 
			QMessageBox.No, QMessageBox.Yes)

		if reply == QMessageBox.Yes:

			
		# self.softwareTriggerGenerator.stop() @@@ => 
			self.image_window.close()

			for key in self.imaging_channels:
				self.liveController[key].stop_live()
				self.camera[key].close()
				self.imageSaver[key].close()
				self.imageDisplay[key].close()
				self.imageDisplayWindow[key].close()
			
			self.trackingDataSaver.close()
			
			self.imageDisplayWindow_ThresholdedImage.close()

			self.microcontroller_Rec.stop()

			event.accept()

	
			
		else:
			event.ignore() 


		