# set QT_API environment variable
import os 
import sys
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True) #enable highdpi scaling
QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True) #use highdpi icons

# Definitions
from control._def import *
# app specific libraries
import control.widgets as widgets
import control.widgets_tracking as widgets_tracking
import control.camera_TIS as camera
import control.core as core
import control.core_tracking as core_tracking
import control.microcontroller_tracking as microcontroller_tracking

SIMULATION = True

class GravityMachineGUI(QMainWindow):

	# variables
	fps_software_trigger = 100

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		self.setWindowTitle('Gravity Machine')

		self.imaging_channels = CAMERAS.keys()

		print('Available imaging channels: {}'.format(self.imaging_channels))

		#------------------------------------------------------------------
		# load other windows
		#------------------------------------------------------------------
		self.imageDisplayWindow = {key:core.ImageDisplayWindow(key + ' Display', 
			DrawCrossHairs = True, rotate_image_angle=90) 
			for key in self.imaging_channels}

		

		self.imageDisplayWindow_ThresholdedImage = core.ImageDisplayWindow('Thresholded Image', rotate_image_angle=90)

		#------------------------------------------------------------------
		# load objects
		#------------------------------------------------------------------
		if SIMULATION is True:
			# Define a camera object for each unique image-stream.
			self.camera = {key:camera.Camera_Simulation() for key in self.imaging_channels}
			self.microcontroller = microcontroller_tracking.Microcontroller_Simulation()

		else:
			self.camera = {key:camera.Camera(serial=CAMERAS[key]['serial'], width = CAMERAS[key]['px_format'][0], 
				height = CAMERAS[key]['px_format'][1], framerate = CAMERAS[key]['fps']) for key in self.imaging_channels}
			self.microcontroller = microcontroller_tracking.Microcontroller()
		
		self.internal_state = core_tracking.InternalState()


		self.streamHandler = {key: core.StreamHandler(camera = self.camera[key], imaging_channel = key)
			for key in self.imaging_channels}	
		#-----------------------------------------------------------------------------------------------
		# Tracking-related objects
		#-----------------------------------------------------------------------------------------------		
		
		self.liveController = core.LiveController(self.camera[TRACKING],self.microcontroller)
		self.navigationController = core.NavigationController(self.microcontroller)
		#self.autofocusController = core.AutoFocusController(self.camera,self.navigationController,self.liveController)
		#self.multipointController = core.MultiPointController(self.camera,self.navigationController,self.liveController,self.autofocusController)
		self.trackingController = core_tracking.TrackingController(self.microcontroller,self.internal_state)
		self.trackingDataSaver = core_tracking.TrackingDataSaver(self.internal_state)
		
		# Microcontroller Receiver object
		self.microcontroller_Rec = core_tracking.microcontroller_Receiver(self.microcontroller, self.internal_state, self.trackingController)

		# Microcontroller Send object
		self.microcontroller_Sender = core_tracking.microcontroller_Sender(self.microcontroller, self.internal_state)

		#-----------------------------------------------------------------------------------------------
		# Define an ImageSaver, and Image Display object for each image stream
		#-----------------------------------------------------------------------------------------------
		self.imageSaver = {key: core_tracking.ImageSaver(self.internal_state, imaging_channel = key) for key in self.imaging_channels}
		self.imageDisplay = {key: core.ImageDisplay() for key in self.imaging_channels}

		# open the camera
		# camera start streaming
		self.camera[TRACKING].open()
		self.camera[TRACKING].set_software_triggered_acquisition() #self.camera.set_continuous_acquisition()
		self.camera[TRACKING].set_callback(self.streamHandler[TRACKING].on_new_frame)
		self.camera[TRACKING].enable_callback()




		#------------------------------------------------------------------
		# load widgets
		#------------------------------------------------------------------
		self.cameraSettingWidget = {key: widgets.CameraSettingsWidget(self.camera[key],self.liveController) for key in self.imaging_channels}


		# self.cameraSettingWidget = widgets.CameraSettingsWidget(self.camera,self.liveController)
		self.liveControlWidget = widgets.LiveControlWidget(self.streamHandler[TRACKING],self.liveController, self.trackingController, self.camera[TRACKING])
		self.navigationWidget = widgets_tracking.NavigationWidget(self.navigationController, self.internal_state, self.microcontroller_Sender)
		#self.autofocusWidget = widgets.AutoFocusWidget(self.autofocusController)
		self.trackingControlWidget = widgets_tracking.TrackingControllerWidget(self.streamHandler[TRACKING], self.trackingController, self.trackingDataSaver, self.internal_state, self.imageDisplayWindow[TRACKING])
		


		self.PID_Group_Widget = widgets_tracking.PID_Group_Widget(self.trackingController)

		self.FocusTracking_Widget = widgets_tracking.FocusTracking_Widget(self.trackingController, self.internal_state)


		self.recordingControlWidget = widgets.RecordingWidget(self.streamHandler,self.imageSaver, self.internal_state, self.trackingControlWidget, self.trackingDataSaver, self.imaging_channels)

		self.recordTabWidget = QTabWidget()
		self.recordTabWidget.addTab(self.recordingControlWidget, "Acquisition control")
		
		self.cameraSettings_Tab = QTabWidget()

		for key in self.imaging_channels:
			self.cameraSettings_Tab.addTab(self.cameraSettingWidget[key],key)

		# self.recordTabWidget.addTab(self.trackingControlWidget, "Tracking")
		#self.recordTabWidget.addTab(self.multiPointWidget, "Multipoint Acquisition")

		self.plotWidget = widgets.dockAreaPlot()

		#-----------------------------------------------------
		# layout widgets
		#-----------------------------------------------------
		layout = QGridLayout() #layout = QStackedLayout()
		# layout.addWidget(self.cameraSettingWidget,0,0)
		layout.addWidget(self.liveControlWidget,0,0)
		
		layout.addWidget(self.navigationWidget,0,1)

		layout.addWidget(self.trackingControlWidget,1,0)

		layout.addWidget(self.FocusTracking_Widget,2,0)



		# layout.addWidget(self.PID_Group_Widget,2,0)
		# layout.addWidget(self.navigationWidget,2,0)
		#layout.addWidget(self.autofocusWidget,3,0)
		layout.addWidget(self.recordTabWidget,1,1)

		layout.addWidget(self.plotWidget,2,1)

		layout.addWidget(self.cameraSettings_Tab,3,1,1,1)
		
		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		

		#------------------------------------------------------------------
		# make connections
		#------------------------------------------------------------------
		self.streamHandler[TRACKING].signal_new_frame_received.connect(self.liveController.on_new_frame)

		# uController is now handled by an in-built timer
		# self.streamHandler.signal_new_frame_received.connect(self.microcontroller_Rec.getData_microcontroller)
		
		# Connections that involve all image streams
		for channel in self.imaging_channels:

			self.streamHandler[channel].image_to_display.connect(self.imageDisplayWindow[channel].display_image)
			self.streamHandler[channel].packet_image_to_write.connect(self.imageSaver[channel].enqueue)
			# self.imageDisplay[channel].image_to_display.connect(self.imageDisplayWindow[channel].display_image) # may connect streamHandler directly to imageDisplayWindow
			self.imageSaver[channel].imageName.connect(self.trackingDataSaver.setImageName)


		# Connections that involve only the tracking image stream
		self.streamHandler[TRACKING].thresh_image_to_display.connect(self.imageDisplayWindow_ThresholdedImage.display_image)

		self.streamHandler[TRACKING].packet_image_for_tracking.connect(self.trackingController.on_new_frame)
		
		self.streamHandler[TRACKING].signal_fps.connect(self.liveControlWidget.update_stream_fps)
		self.streamHandler[TRACKING].signal_fps_display.connect(self.liveControlWidget.update_display_fps)

		self.streamHandler[TRACKING].signal_working_resolution.connect(self.liveControlWidget.update_working_resolution)


		self.trackingController.centroid_image.connect(self.imageDisplayWindow[TRACKING].draw_circle)
		self.trackingController.Rect_pt1_pt2.connect(self.imageDisplayWindow[TRACKING].draw_rectangle)
		

		self.trackingController.multiplex_send_signal.connect(self.microcontroller_Sender.multiplex_sendData)

		self.trackingController.save_data_signal.connect(self.trackingDataSaver.enqueue)
		

		self.microcontroller_Rec.update_display.connect(self.navigationWidget.update_display)
		# self.navigationController.xPos.connect(self.navigationWidget.label_Xpos.setNum)
		# self.navigationController.yPos.connect(self.navigationWidget.label_Ypos.setNum)
		# self.navigationController.zPos.connect(self.navigationWidget.label_Zpos.setNum)
		#self.autofocusController.image_to_display.connect(self.imageDisplayWindow.display_image)
		#self.multipointController.image_to_display.connect(self.imageDisplayWindow.display_image)

		# Show sub-windows:
		for key in self.imaging_channels:
			self.imageDisplayWindow[key].show()
		self.imageDisplayWindow_ThresholdedImage.show()


		print('Starting image streams')

		# self.start_imageStreams()
		self.camera[TRACKING].start_streaming()


		print('Started image streams!')

	def start_imageStreams(self):
		for key in self.imaging_channels:
			self.camera[key].start_streaming()

	def closeEvent(self, event):

		reply = QMessageBox.question(self, 'Message',
			"Are you sure you want to exit?", QMessageBox.Yes | 
			QMessageBox.No, QMessageBox.Yes)

		if reply == QMessageBox.Yes:

			
		# self.softwareTriggerGenerator.stop() @@@ => 


			self.liveController.stop_live()
			for key in self.imaging_channels:
				self.camera[key].close()
				self.imageSaver[key].stop_saving_images()
				self.imageDisplay[key].close()
				self.imageDisplayWindow[key].close()
			
			self.trackingDataSaver.stop_DataSaver()
			
			self.imageDisplayWindow_ThresholdedImage.close()

			self.microcontroller_Rec.stop()

			event.accept()

	
			
		else:
			event.ignore() 
		