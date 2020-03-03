# set QT_API environment variable
import os 
import sys
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *


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

		#------------------------------------------------------------------
		# load other windows
		#------------------------------------------------------------------
		self.imageDisplayWindow = core.ImageDisplayWindow('Main Display', DrawCrossHairs = True)
		self.imageDisplayWindow.show()

		self.imageDisplayWindow_ThresholdedImage = core.ImageDisplayWindow('Thresholded Image')
		self.imageDisplayWindow_ThresholdedImage.show()

		#------------------------------------------------------------------
		# load objects
		#------------------------------------------------------------------
		if SIMULATION is True:
			self.camera = camera.Camera_Simulation()
			self.microcontroller = microcontroller_tracking.Microcontroller_Simulation()
		else:
			self.camera = camera.Camera(sn=17910085)
			self.microcontroller = microcontroller.Microcontroller()
		
		self.internal_state = core_tracking.InternalState()

		
		self.streamHandler = core.StreamHandler(camera = self.camera)
		self.liveController = core.LiveController(self.camera,self.microcontroller)
		self.navigationController = core.NavigationController(self.microcontroller)
		#self.autofocusController = core.AutoFocusController(self.camera,self.navigationController,self.liveController)
		#self.multipointController = core.MultiPointController(self.camera,self.navigationController,self.liveController,self.autofocusController)
		self.trackingController = core_tracking.TrackingController(self.microcontroller,self.internal_state)
		

		self.trackingDataSaver = core_tracking.TrackingDataSaver(self.internal_state)
		self.imageSaver = core_tracking.ImageSaver(self.internal_state, imaging_channel = TRACKING_STREAM)
		self.imageDisplay = core.ImageDisplay()

		# open the camera
		# camera start streaming
		self.camera.open()
		self.camera.set_software_triggered_acquisition() #self.camera.set_continuous_acquisition()
		self.camera.set_callback(self.streamHandler.on_new_frame)
		self.camera.enable_callback()

		# Microcontroller Receiver object
		self.microcontroller_Rec = core_tracking.microcontroller_Receiver(self.microcontroller, self.internal_state, self.trackingController)

		# Microcontroller Send object
		self.microcontroller_Sender = core_tracking.microcontroller_Sender(self.microcontroller, self.internal_state)


		#------------------------------------------------------------------
		# load widgets
		#------------------------------------------------------------------
		self.cameraSettingWidget = widgets.CameraSettingsWidget(self.camera,self.liveController)
		self.liveControlWidget = widgets.LiveControlWidget(self.streamHandler,self.liveController, self.trackingController, self.camera)
		self.navigationWidget = widgets_tracking.NavigationWidget(self.navigationController, self.internal_state)
		#self.autofocusWidget = widgets.AutoFocusWidget(self.autofocusController)
		self.recordingControlWidget = widgets.RecordingWidget(self.streamHandler,self.imageSaver, self.internal_state, self.trackingDataSaver)
		self.trackingControlWidget = widgets_tracking.TrackingControllerWidget(self.streamHandler, self.trackingController, self.trackingDataSaver, self.internal_state, self.imageDisplayWindow)
		
		self.PID_Group_Widget = widgets_tracking.PID_Group_Widget(self.trackingController)

		self.FocusTracking_Widget = widgets_tracking.FocusTracking_Widget(self.trackingController, self.internal_state)

		self.recordTabWidget = QTabWidget()
		self.recordTabWidget.addTab(self.recordingControlWidget, "Simple Recording")
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
		
		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		

		#------------------------------------------------------------------
		# make connections
		#------------------------------------------------------------------
		self.streamHandler.signal_new_frame_received.connect(self.liveController.on_new_frame)

		self.streamHandler.signal_new_frame_received.connect(self.microcontroller_Rec.getData_microcontroller)
		
		self.streamHandler.image_to_display.connect(self.imageDisplay.enqueue)
		
		self.streamHandler.thresh_image_to_display.connect(self.imageDisplayWindow_ThresholdedImage.display_image)

		self.streamHandler.packet_image_to_write.connect(self.imageSaver.enqueue)
		self.streamHandler.packet_image_for_tracking.connect(self.trackingController.on_new_frame)
		self.imageDisplay.image_to_display.connect(self.imageDisplayWindow.display_image) # may connect streamHandler directly to imageDisplayWindow
		
		self.streamHandler.signal_fps.connect(self.liveControlWidget.update_stream_fps)
		self.streamHandler.signal_fps_display.connect(self.liveControlWidget.update_display_fps)

		self.streamHandler.signal_working_resolution.connect(self.liveControlWidget.update_working_resolution)


		self.trackingController.centroid_image.connect(self.imageDisplayWindow.draw_circle)
		self.trackingController.Rect_pt1_pt2.connect(self.imageDisplayWindow.draw_rectangle)
		

		self.trackingController.multiplex_send_signal.connect(self.microcontroller_Sender.multiplex_Send)

		self.trackingController.save_data_signal.connect(self.trackingDataSaver.enqueue)
		
		self.imageSaver.imageName.connect(self.trackingDataSaver.setImageName)

		self.microcontroller_Rec.update_display.connect(self.navigationWidget.update_display)
		# self.navigationController.xPos.connect(self.navigationWidget.label_Xpos.setNum)
		# self.navigationController.yPos.connect(self.navigationWidget.label_Ypos.setNum)
		# self.navigationController.zPos.connect(self.navigationWidget.label_Zpos.setNum)
		#self.autofocusController.image_to_display.connect(self.imageDisplayWindow.display_image)
		#self.multipointController.image_to_display.connect(self.imageDisplayWindow.display_image)

		self.camera.start_streaming()

	def closeEvent(self, event):

		reply = QMessageBox.question(self, 'Message',
			"Are you sure you want to exit?", QMessageBox.Yes | 
			QMessageBox.No, QMessageBox.Yes)

		if reply == QMessageBox.Yes:

			
		# self.softwareTriggerGenerator.stop() @@@ => 


			self.liveController.stop_live()
			self.camera.close()
			self.imageSaver.stop_saving_images()
			self.trackingDataSaver.stop_DataSaver()
			self.imageDisplay.close()
			self.imageDisplayWindow.close()
			self.imageDisplayWindow_ThresholdedImage.close()


			event.accept()

	
			
		else:
			event.ignore() 
		