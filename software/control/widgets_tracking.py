# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy
import numpy as np

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import pyqtgraph as pg
#import pyqtgraph.ptime as ptime
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel

from control._def import *

from control.utils import rangeslider as rangeslider

class TrackingControllerWidget(QFrame):
	'''
	Buttons to start image tracking
	Display window to show thresholded images
	Slider bars to threshold image
	Radio-buttons to choose trackers.
	Text boxes for base path and Experiment ID.

	'''
	def __init__(self, streamHandler, trackingController, trackingDataSaver, internal_state, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.base_path_is_set = False

		self.streamHandler = streamHandler
		self.trackingController = trackingController

		self.trackingDataSaver = trackingDataSaver

		self.internal_state = internal_state

		# self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

		self.add_components()


	def add_components(self):

		self.tracking_group = QGroupBox('Tracking settings', alignment = Qt.AlignCenter)

		tracking_group_layout = QHBoxLayout()

		# Image Tracking Button
		self.btn_track = QPushButton("Track")
		# self.btn_track.setStyleSheet('QPushButton {color: red;}')
		self.btn_track.setCheckable(True)
		self.btn_track.setChecked(False)
		self.btn_track.setDefault(False)

		# Image Tracker Dropdown
		self.dropdown_TrackerSelection = QComboBox()
		self.dropdown_TrackerSelection.addItems(TRACKERS)
		self.dropdown_TrackerSelection.setCurrentText(DEFAULT_TRACKER)
		self.trackingController.tracker_image.update_tracker_type(self.dropdown_TrackerSelection.currentText())

		tracking_group_layout.addWidget(self.dropdown_TrackerSelection)

		self.tracking_group.setLayout(tracking_group_layout)


		# Image offset settings
		self.tracking_setPoint_group = QGroupBox('Tracking set-point offset', alignment = Qt.AlignCenter)
		tracking_setPoint_layout = QGridLayout()

		self.label_x = QLabel('x (px)')
		# Image tracking offset - X axis
		self.tracking_setPoint_offset_x = QSpinBox()
		self.tracking_setPoint_offset_x.setMinimum(-round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_x.setMaximum(round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_x.setSingleStep(1)
		self.tracking_setPoint_offset_x.setValue(0)

		# Image tracking offset - Y axis
		self.label_y = QLabel('y (px)')

		self.tracking_setPoint_offset_y = QSpinBox()
		self.tracking_setPoint_offset_y.setMinimum(-round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_y.setMaximum(round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_y.setSingleStep(1)
		self.tracking_setPoint_offset_y.setValue(0)
		# layout

		tracking_setPoint_layout.addWidget(self.label_x,0,0,1,1)
		tracking_setPoint_layout.addWidget(self.tracking_setPoint_offset_x,1,0,1,1)
		tracking_setPoint_layout.addWidget(self.label_y, 0,1,1,1)
		tracking_setPoint_layout.addWidget(self.tracking_setPoint_offset_y, 1,1,1,1)

		self.tracking_setPoint_group.setLayout(tracking_setPoint_layout)


		

		# Range sliders for image color thresholding
		self.group_sliders = QGroupBox('Color thresholds', alignment = Qt.AlignCenter)
		layout_sliders = QGridLayout()
		
		self.label_Hue = QLabel('Hue')
		self.range_slider1 = rangeslider.QRangeSlider()
		self.range_slider1.setMax(255)
		self.label_Saturation=QLabel('Saturation')
		self.range_slider2=rangeslider.QRangeSlider()
		self.range_slider2.setMax(255)
		self.label_Vibrance=QLabel('Value')
		self.range_slider3=rangeslider.QRangeSlider()
		self.range_slider3.setMax(255)
		
		layout_sliders.addWidget(self.label_Hue,0,0,1,1)
		layout_sliders.addWidget(self.range_slider1,0,1,1,1)
		layout_sliders.addWidget(self.label_Saturation,1,0,1,1)
		layout_sliders.addWidget(self.range_slider2,1,1,1,1)
		layout_sliders.addWidget(self.label_Vibrance,2,0,1,1)
		layout_sliders.addWidget(self.range_slider3,2,1,1,1)
		self.group_sliders.setLayout(layout_sliders)
		self.group_sliders.setEnabled(True)


		# groupbox_track_settings = QGroupBox('Tracking Controller')

		groupbox_track_layout = QGridLayout()
		groupbox_track_layout.addWidget(self.btn_track, 0,0,1,1)
		# groupbox_track_layout.addWidget(self.dropdown_TrackerSelection, 0,1,1,1)
		groupbox_track_layout.addWidget(self.tracking_group,0,1,1,1)
		groupbox_track_layout.addWidget(self.tracking_setPoint_group,0,2,1,1)
		groupbox_track_layout.addWidget(self.group_sliders,1,0,1,3)

		# Track button connection
		self.btn_track.clicked.connect(self.do_track_button_tasks)

		# Choose tracker
		self.dropdown_TrackerSelection.currentIndexChanged.connect(self.update_tracker)

		# Image tracking setpoint
		self.tracking_setPoint_offset_x.valueChanged.connect(self.update_tracking_setPoints)
		self.tracking_setPoint_offset_y.valueChanged.connect(self.update_tracking_setPoints)


		self.range_slider1.startValueChanged.connect(self.sliders_move)
		self.range_slider2.startValueChanged.connect(self.sliders_move)
		self.range_slider3.startValueChanged.connect(self.sliders_move)
		self.range_slider1.endValueChanged.connect(self.sliders_move)
		self.range_slider2.endValueChanged.connect(self.sliders_move)
		self.range_slider3.endValueChanged.connect(self.sliders_move)

		

		self.setLayout(groupbox_track_layout)



	def do_track_button_tasks(self):

		if self.btn_track.isChecked():

			# Start a new track. If 'Aquire' is true this also creates a track file.
			# Internal state is changed after creating this file.
			self.trackingDataSaver.start_new_track()

		else:
			self.internal_state.data['track_obj_image'] = False
			# Resets the track deques and counters

			self.trackingController.initialise_track()

	# This function is connected to the signal from tracking Controller triggered by 
	# hardware start-tracking input.
	def handle_hardware_track_signal(self):

		self.btn_track.toggle()
		self.do_track_button_tasks()

	def update_tracker(self, index):

		self.trackingController.tracker_image.update_tracker_type(self.dropdown_TrackerSelection.currentText())

	def update_tracking_setPoints(self):

		value_x = self.tracking_setPoint_offset_x.value()
		value_y = self.tracking_setPoint_offset_y.value()

		self.trackingController.update_image_offset((value_x, value_y))


	def set_slider_defaults(self, LOWER =[0,0,0], UPPER = [255,255,255]):

		LOWER=np.array(LOWER,dtype="uint8")
		UPPER=np.array(UPPER,dtype="uint8")

		self.range_slider1.setRange(LOWER[0],UPPER[0])
		self.range_slider2.setRange(LOWER[1],UPPER[1])
		self.range_slider3.setRange(LOWER[2],UPPER[2])

	def sliders_move(self):
		LOWER=np.array([0,0,0],dtype="uint8")
		UPPER=np.array([255,255,255],dtype="uint8")
		
		LOWER[0],UPPER[0]=self.range_slider1.getRange()
		LOWER[1],UPPER[1]=self.range_slider2.getRange()
		LOWER[2],UPPER[2]=self.range_slider3.getRange()

		self.streamHandler.set_image_thresholds(np.uint8(LOWER), np.uint8(UPPER))



		# self.camera_functions[self.tracking_channel].lower_HSV=np.uint8(LOWER)
		# # self.object_tracking.lower_HSV=np.uint8(LOWER)
		# self.camera_functions[self.tracking_channel].upper_HSV=np.uint8(UPPER)
		# self.object_tracking.upper_HSV=np.uint8(UPPER		


# class PID_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass
class PID_Group_Widget(QGroupBox):

	def __init__(self, trackingController):
		super().__init__()

		self.setTitle('PID settings')

		self.trackingController = trackingController

		self.add_components()

	def add_components(self):

		self.PID_widget_x = PID_Widget('X')
		self.PID_widget_z = PID_Widget('Z')
		self.PID_widget_y = PID_Widget('Y')

		PID_imagePlane = QGroupBox('PID (Image Plane)')
		PID_imagePlane_layout = QHBoxLayout()

		PID_imagePlane_layout.addWidget(self.PID_widget_z)
		PID_imagePlane_layout.addWidget(self.PID_widget_x)

		PID_imagePlane.setLayout(PID_imagePlane_layout)

		PID_focus = QGroupBox('PID (Focus)')
		PID_focus_Layout = QHBoxLayout()
		PID_focus_Layout.addWidget(self.PID_widget_y)

		PID_focus.setLayout(PID_focus_Layout)

		hor_layout = QGridLayout()

		hor_layout.addWidget(PID_imagePlane,0,0,1,1)
		hor_layout.addWidget(PID_focus,0,1,1,1)


		self.setLayout(hor_layout)





		# Connections

		# X
		self.PID_widget_x.spinboxP.valueChanged.connect(self.trackingController.pid_controller_x.update_P)
		self.PID_widget_x.spinboxI.valueChanged.connect(self.trackingController.pid_controller_x.update_I)
		self.PID_widget_x.spinboxD.valueChanged.connect(self.trackingController.pid_controller_x.update_D)

		# Y
		self.PID_widget_y.spinboxP.valueChanged.connect(self.trackingController.pid_controller_y.update_P)
		self.PID_widget_y.spinboxI.valueChanged.connect(self.trackingController.pid_controller_y.update_I)
		self.PID_widget_y.spinboxD.valueChanged.connect(self.trackingController.pid_controller_y.update_D)

		# Theta
		self.PID_widget_z.spinboxP.valueChanged.connect(self.trackingController.pid_controller_theta.update_P)
		self.PID_widget_z.spinboxI.valueChanged.connect(self.trackingController.pid_controller_theta.update_I)
		self.PID_widget_z.spinboxD.valueChanged.connect(self.trackingController.pid_controller_theta.update_D)





class PID_Widget(QGroupBox):
	
	def __init__(self,name,Pmax=2,Dmax=1,Imax=1):
		super().__init__()
		
		self.setTitle(name)
	
		# Slider Groupe P
		defaultP = Pmax/2
		stepP = Pmax/100

		self.labelP = QLabel('P')
		self.hsliderP = QSlider(Qt.Horizontal)
		self.hsliderP.setRange(0,int(Pmax*100))
		self.hsliderP.setValue(int(defaultP*100))
		self.spinboxP = QDoubleSpinBox()
		self.spinboxP.setRange(0,round(Pmax,2))
		self.spinboxP.setSingleStep(round(stepP,2))
		self.spinboxP.setValue(round(defaultP,2))
		self.hsliderP.valueChanged.connect(self.spinBoxP_setValue)
		self.spinboxP.valueChanged.connect(self.hsliderP_setValue)
		sliderP_layout=QHBoxLayout()
		sliderP_layout.addWidget(self.labelP)
		sliderP_layout.addWidget(self.hsliderP)
		sliderP_layout.addWidget(self.spinboxP)
		group_sliderP=QWidget()
		group_sliderP.setLayout(sliderP_layout)
		

		defaultI = 0
		stepI = Imax/100
		# Slider Groupe I
		self.labelI = QLabel('I')
		self.hsliderI = QSlider(Qt.Horizontal)
		self.hsliderI.setRange(0,int(Imax*100))
		self.hsliderI.setValue(int(defaultI*100))
		self.spinboxI=QDoubleSpinBox()
		self.spinboxI.setSingleStep(round(stepI,2))
		self.spinboxI.setRange(0,int(Imax))
		self.spinboxI.setValue(round(defaultI,2))
		self.hsliderI.valueChanged.connect(self.spinBoxI_setValue)
		self.spinboxI.valueChanged.connect(self.hsliderI_setValue)
		sliderI_layout=QHBoxLayout()
		sliderI_layout.addWidget(self.labelI)
		sliderI_layout.addWidget(self.hsliderI)
		sliderI_layout.addWidget(self.spinboxI)
		group_sliderI=QWidget()
		group_sliderI.setLayout(sliderI_layout)
		
		# Slider Groupe D
		defaultD = Dmax/4
		stepD = Dmax/100

		self.labelD = QLabel('D')
		self.hsliderD = QSlider(Qt.Horizontal)
		self.hsliderD.setRange(0,int(Dmax*100))
		self.hsliderD.setValue(int(defaultD*100))
		self.spinboxD=QDoubleSpinBox()
		self.spinboxD.setRange(0,int(Dmax))
		self.spinboxI.setSingleStep(round(stepD,2))
		self.spinboxD.setValue(round(defaultD,2))
		self.hsliderD.valueChanged.connect(self.spinBoxD_setValue)
		self.spinboxD.valueChanged.connect(self.hsliderD_setValue)
		sliderD_layout=QHBoxLayout()
		sliderD_layout.addWidget(self.labelD)
		sliderD_layout.addWidget(self.hsliderD)
		sliderD_layout.addWidget(self.spinboxD)
		group_sliderD=QWidget()
		group_sliderD.setLayout(sliderD_layout)
		
				# Big PID group
		groupbox_layout_PID = QVBoxLayout()
		groupbox_layout_PID.addWidget(group_sliderP)   
		groupbox_layout_PID.addWidget(group_sliderI)
		groupbox_layout_PID.addWidget(group_sliderD)
		
		
		self.setLayout(groupbox_layout_PID)
	
	def spinBoxP_setValue(self,value):
		newvalue=float(value)/100.
		self.spinboxP.setValue(newvalue)

	def hsliderP_setValue(self,value):
		self.hsliderP.setValue(int(value*100)) 

	def spinBoxI_setValue(self,value):
		newvalue=float(value)/100.
		self.spinboxI.setValue(newvalue)

	def hsliderI_setValue(self,value):
		self.hsliderI.setValue(int(value*100)) 

	def spinBoxD_setValue(self,value):
		newvalue=float(value)/100.
		self.spinboxD.setValue(newvalue)

	def hsliderD_setValue(self,value):
		self.hsliderD.setValue(int(value*100))


# class FocusTracking_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass


# class PlotDisplay_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass
