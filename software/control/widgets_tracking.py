# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy
import numpy as np
import time

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

class TrackingControllerWidget(QGroupBox):
	'''
	Buttons to start image tracking
	Display window to show thresholded images
	Slider bars to threshold image
	Radio-buttons to choose trackers.
	Text boxes for base path and Experiment ID.

	'''
	show_roi = Signal(bool)

	def __init__(self, streamHandler, trackingController, trackingDataSaver, internal_state, ImageDisplayWindow, microcontroller, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.setTitle('Tracking Controller')

		self.base_path_is_set = False

		self.streamHandler = streamHandler
		self.trackingController = trackingController

		self.trackingDataSaver = trackingDataSaver

		self.internal_state = internal_state

		self.ImageDisplayWindow = ImageDisplayWindow

		self.microcontroller = microcontroller

		# self.add_components()
		# self.setFrameStyle(QFrame.Panel | QFrame.Raised)

		self.add_components()

		# Initialize states in underlying objects
		self.update_tracker_init_method()
		self.update_invert_image_flag()
		self.sliders_move()



	def add_components(self):

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

		# Invert thresholded image checkbox (useful when switching between BF and DF)
		self.invert_image_checkbox = QCheckBox('Invert image')
		self.invert_image_checkbox.setChecked(False)

		self.tracking_init_threshold = QRadioButton("Threshold")
		self.tracking_init_roi = QRadioButton("ROI")
		self.tracking_init_threshold.setChecked(True)

		# Layout
		self.tracking_init_group = QGroupBox('Tracking init method')
		self.tracking_init_layout = QGridLayout()
		self.tracking_init_layout.addWidget(self.tracking_init_threshold,0,0)
		self.tracking_init_layout.addWidget(self.tracking_init_roi,1,0)
		self.tracking_init_layout.addWidget(self.invert_image_checkbox,0,1)
		self.tracking_init_group.setLayout(self.tracking_init_layout)

		# Image offset settings
		self.label_x = QLabel('x (px)')
		# Image tracking offset - X axis
		self.tracking_setPoint_offset_x = QSpinBox()
		self.tracking_setPoint_offset_x.setMinimum(-round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_x.setMaximum(round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_x.setSingleStep(1)
		self.tracking_setPoint_offset_x.setValue(0)

		# Image tracking offset - Z axis
		self.label_y = QLabel('z (px)')
		self.tracking_setPoint_offset_y = QSpinBox()
		self.tracking_setPoint_offset_y.setMinimum(-round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_y.setMaximum(round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_y.setSingleStep(1)
		self.tracking_setPoint_offset_y.setValue(0)

		# Sliders for image segmentation
		self.label_Hue = QLabel('Hue')
		self.range_slider1 = rangeslider.QRangeSlider()
		self.range_slider1.setMax(255)
		self.label_Saturation=QLabel('Saturation')
		self.range_slider2=rangeslider.QRangeSlider()
		self.range_slider2.setMax(255)
		self.label_Vibrance=QLabel('Value')
		self.range_slider3=rangeslider.QRangeSlider()
		self.range_slider3.setMax(255)

		# Sub-blocks layout
		tracking_group_layout = QHBoxLayout()
		tracking_group_layout.addWidget(QLabel('Tracker selection'))
		tracking_group_layout.addWidget(self.dropdown_TrackerSelection)
		
		self.tracking_setPoint_group = QGroupBox('Tracking set-point offset', alignment = Qt.AlignCenter)
		tracking_setPoint_layout = QGridLayout()
		tracking_setPoint_layout.addWidget(self.label_x,0,0)
		tracking_setPoint_layout.addWidget(self.tracking_setPoint_offset_x,0,1)
		tracking_setPoint_layout.addWidget(self.label_y, 1,0)
		tracking_setPoint_layout.addWidget(self.tracking_setPoint_offset_y, 1,1)
		self.tracking_setPoint_group.setLayout(tracking_setPoint_layout)

		# Range sliders for image color thresholding
		self.group_sliders = QGroupBox('Color thresholds', alignment = Qt.AlignCenter)
		layout_sliders = QGridLayout()
		
		layout_sliders.addWidget(self.label_Hue,0,0,1,1)
		layout_sliders.addWidget(self.range_slider1,0,1,1,1)
		layout_sliders.addWidget(self.label_Saturation,1,0,1,1)
		layout_sliders.addWidget(self.range_slider2,1,1,1,1)
		layout_sliders.addWidget(self.label_Vibrance,2,0,1,1)
		layout_sliders.addWidget(self.range_slider3,2,1,1,1)
		self.group_sliders.setLayout(layout_sliders)
		self.group_sliders.setEnabled(True)

		# Overall layout
		groupbox_track_layout = QGridLayout()
		groupbox_track_layout.addWidget(self.btn_track, 0,0,1,1)
		# groupbox_track_layout.addWidget(self.dropdown_TrackerSelection, 0,1,1,1)
		groupbox_track_layout.addLayout(tracking_group_layout,0,1,1,1)
		groupbox_track_layout.addWidget(self.tracking_setPoint_group,1,0,1,1)
		groupbox_track_layout.addWidget(self.tracking_init_group,1,1,1,1)
		groupbox_track_layout.addWidget(self.group_sliders,2,0,1,2)
		
		# Track button connection
		self.btn_track.clicked.connect(self.do_track_button_tasks)

		# Choose tracker
		self.dropdown_TrackerSelection.currentIndexChanged.connect(self.update_tracker)

		# Image tracking setpoint
		self.tracking_setPoint_offset_x.valueChanged.connect(self.update_tracking_setPoints)
		self.tracking_setPoint_offset_y.valueChanged.connect(self.update_tracking_setPoints)

		self.invert_image_checkbox.clicked.connect(self.update_invert_image_flag)

		self.tracking_init_threshold.clicked.connect(self.update_tracker_init_method)
		self.tracking_init_roi.clicked.connect(self.update_tracker_init_method)

		self.range_slider1.startValueChanged.connect(self.sliders_move)
		self.range_slider2.startValueChanged.connect(self.sliders_move)
		self.range_slider3.startValueChanged.connect(self.sliders_move)
		self.range_slider1.endValueChanged.connect(self.sliders_move)
		self.range_slider2.endValueChanged.connect(self.sliders_move)
		self.range_slider3.endValueChanged.connect(self.sliders_move)

		self.setLayout(groupbox_track_layout)


	def do_track_button_tasks(self):

		if self.btn_track.isChecked():

			# Start a new track. If 'Acquire' is true this also creates a track file.
			# Internal state is changed after creating this file.
				# Update the internal_state to indicate that object should be tracked using image proc
			self.internal_state.data['track_obj_image'] = True

			print('Set track_obj_image to : {}'.format(self.internal_state.data['track_obj_image']))
			
			if(self.tracking_init_roi.isChecked()):
				self.trackingController.update_roi_bbox()

			self.trackingDataSaver.start_new_track()
			self.streamHandler.start_tracking()

		else:
			self.streamHandler.stop_tracking()
			self.internal_state.data['track_obj_image'] = False
			# Resets the track deques and counters
			self.trackingController.initialise_track()

	# This function is connected to the signal from tracking Controller triggered by 
	# hardware start-tracking input.
	def handle_hardware_track_signal(self):

		self.btn_track.toggle()
		self.do_track_button_tasks()

	def handle_aquisition_widget_track_signal(self):
		self.btn_track.setChecked(True)

	def update_invert_image_flag(self):

		if(self.invert_image_checkbox.isChecked()):
			self.streamHandler.update_invert_image_flag(True)
		else:
			self.streamHandler.update_invert_image_flag(False)

	def update_tracker_init_method(self):

		if(self.tracking_init_threshold.isChecked()):
			self.trackingController.tracker_image.update_init_method("threshold")
			self.show_roi.emit(False)
		elif(self.tracking_init_roi.isChecked()):
			self.trackingController.tracker_image.update_init_method("roi")
			self.show_roi.emit(True)



	def update_tracker(self, index):

		self.trackingController.tracker_image.update_tracker_type(self.dropdown_TrackerSelection.currentText())

	def update_tracking_setPoints(self):

		value_x = self.tracking_setPoint_offset_x.value()
		value_y = self.tracking_setPoint_offset_y.value()

		self.trackingController.update_image_offset((value_x, value_y))

		'''
		Changing the tracking set point also changes the cross-hair location 
		displayed on the window (so a user can position the object precisely 
		where they want in the frame)
		'''
		self.ImageDisplayWindow.update_image_offset((value_x, value_y))



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


class StageCalibrationWidget(QFrame):
	
	def __init__(self, internal_state, microcontroller,  main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.internal_state = internal_state
		self.microcontroller = microcontroller
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):


		# Stage zeroing buttons
		self.zero_X = QPushButton('Zero X-stage')
		
		self.zero_Y = QPushButton('Zero Y-stage')
	
		self.zero_Theta = QPushButton('Zero Theta-stage')
	
		
		# Homing Button
		self.homing_button = QPushButton('Run Homing')

		stage_control = QVBoxLayout()

		stage_control.addWidget(self.homing_button)
		stage_control.addWidget(self.zero_X)
		stage_control.addWidget(self.zero_Y)
		stage_control.addWidget(self.zero_Theta)

		self.setLayout(stage_control)


		# Connections
		self.zero_X.clicked.connect(self.zero_X_stage)
		self.zero_Y.clicked.connect(self.zero_Y_stage)
		self.zero_Theta.clicked.connect(self.zero_Theta_stage)

		self.homing_button.clicked.connect(self.homing_button_click)

	def zero_X_stage(self):

		self.microcontroller.send_stage_zero_command(0)
	
	def zero_Y_stage(self):

		self.microcontroller.send_stage_zero_command(1)

	def zero_Theta_stage(self):

		self.microcontroller.send_stage_zero_command(3)

	def homing_button_click(self):
		# Send homing command to microcontroller
		self.microcontroller.send_homing_command()

	def update_homing_state():
		self.homing_button.setText(self.internal_state.data['homing-state'])


class NavigationWidget(QFrame):
    def __init__(self, navigationController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigationController = navigationController
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.label_Xpos = QLabel()
        self.label_Xpos.setNum(0)
        self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dX = QDoubleSpinBox()
        self.entry_dX.setMinimum(0) 
        self.entry_dX.setMaximum(5) 
        self.entry_dX.setSingleStep(0.2)
        self.entry_dX.setValue(0)
        self.btn_moveX_forward = QPushButton('Forward')
        self.btn_moveX_forward.setDefault(False)
        self.btn_moveX_backward = QPushButton('Backward')
        self.btn_moveX_backward.setDefault(False)
        
        self.label_Ypos = QLabel()
        self.label_Ypos.setNum(0)
        self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dY = QDoubleSpinBox()
        self.entry_dY.setMinimum(0)
        self.entry_dY.setMaximum(5)
        self.entry_dY.setSingleStep(0.2)
        self.entry_dY.setValue(0)
        self.btn_moveY_forward = QPushButton('Forward')
        self.btn_moveY_forward.setDefault(False)
        self.btn_moveY_backward = QPushButton('Backward')
        self.btn_moveY_backward.setDefault(False)

        self.label_Thetapos = QLabel()
        self.label_Thetapos.setNum(0)
        self.label_Thetapos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dTheta = QDoubleSpinBox()
        self.entry_dTheta.setMinimum(0) 
        self.entry_dTheta.setMaximum(2*np.pi) 
        self.entry_dTheta.setSingleStep(0.01)
        self.entry_dTheta.setValue(0)
        self.btn_moveTheta_forward = QPushButton('Forward')
        self.btn_moveTheta_forward.setDefault(False)
        self.btn_moveTheta_backward = QPushButton('Backward')
        self.btn_moveTheta_backward.setDefault(False)
        
        grid_line0 = QGridLayout()
        grid_line0.addWidget(QLabel('X (mm)'), 0,0)
        grid_line0.addWidget(self.label_Xpos, 0,1)
        grid_line0.addWidget(self.entry_dX, 0,2)
        grid_line0.addWidget(self.btn_moveX_forward, 0,3)
        grid_line0.addWidget(self.btn_moveX_backward, 0,4)

        grid_line1 = QGridLayout()
        grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
        grid_line1.addWidget(self.label_Ypos, 0,1)
        grid_line1.addWidget(self.entry_dY, 0,2)
        grid_line1.addWidget(self.btn_moveY_forward, 0,3)
        grid_line1.addWidget(self.btn_moveY_backward, 0,4)

        grid_line2 = QGridLayout()
        grid_line2.addWidget(QLabel('Theta (radians)'), 0,0)
        grid_line2.addWidget(self.label_Thetapos, 0,1)
        grid_line2.addWidget(self.entry_dTheta, 0,2)
        grid_line2.addWidget(self.btn_moveTheta_forward, 0,3)
        grid_line2.addWidget(self.btn_moveTheta_backward, 0,4)

        self.grid = QGridLayout()
        self.grid.addLayout(grid_line0,0,0)
        self.grid.addLayout(grid_line1,1,0)
        self.grid.addLayout(grid_line2,2,0)
        self.setLayout(self.grid)

        self.btn_moveX_forward.clicked.connect(self.move_x_forward)
        self.btn_moveX_backward.clicked.connect(self.move_x_backward)
        self.btn_moveY_forward.clicked.connect(self.move_y_forward)
        self.btn_moveY_backward.clicked.connect(self.move_y_backward)
        self.btn_moveTheta_forward.clicked.connect(self.move_theta_forward)
        self.btn_moveTheta_backward.clicked.connect(self.move_theta_backward)
        
    def move_x_forward(self):
        self.navigationController.move_x(self.entry_dX.value())
    def move_x_backward(self):
        self.navigationController.move_x(-self.entry_dX.value())
    def move_y_forward(self):
        self.navigationController.move_y(self.entry_dY.value())
    def move_y_backward(self):
        self.navigationController.move_y(-self.entry_dY.value())
    def move_theta_forward(self):
        self.navigationController.move_theta(self.entry_dTheta.value())
    def move_theta_backward(self):
        self.navigationController.move_theta(-self.entry_dTheta.value())
    
    def update_display(self, X_stage, Y_stage, Theta_stage):
    	self.label_Xpos.setText('{:.02f}'.format(round(X_stage,2)))
    	self.label_Ypos.setText('{:.02f}'.format(round(Y_stage,2)))
    	self.label_Thetapos.setText('{:.02f}'.format(round(Theta_stage,2)))


class PID_Group_Widget(QFrame):

	def __init__(self, trackingController):
		super().__init__()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

		# self.setTitle('PID settings')

		self.trackingController = trackingController

		self.add_components()

	def add_components(self):

		self.PID_widget_x = PID_Widget('X')
		self.PID_widget_z = PID_Widget('Z')
		self.PID_widget_y = PID_Widget('Y')

		# PID_imagePlane = QGroupBox('PID (Image Plane)')
		# PID_imagePlane_layout = QHBoxLayout()

		# PID_imagePlane_layout.addWidget(self.PID_widget_x)
		# PID_imagePlane_layout.addWidget(self.PID_widget_y)

		# PID_imagePlane.setLayout(PID_imagePlane_layout)

		# PID_focus = QGroupBox('PID (Focus)')
		# PID_focus_Layout = QHBoxLayout()
		# PID_focus_Layout.addWidget(self.PID_widget_z)

		# PID_focus.setLayout(PID_focus_Layout)

		hor_layout = QGridLayout()

		hor_layout.addWidget(self.PID_widget_x,0,0,1,1)
		hor_layout.addWidget(self.PID_widget_y,0,1,1,1)
		hor_layout.addWidget(self.PID_widget_z,0,2,1,1)


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
		defaultP = Pmax/10
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
		defaultD = Dmax/10
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


class FocusTracking_Widget(QFrame):

	def __init__(self, trackingController, internal_state, microcontroller, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.trackingController = trackingController
		self.internal_state = internal_state
		self.microcontroller = microcontroller

		self.add_components()
		

	def add_components(self):

		self.button_FocusTracking = QPushButton('Start Focus Tracking')
		self.button_FocusTracking.setCheckable(True)
		self.button_FocusTracking.setChecked(False)

		# cropRatio
		self.label_crop_ratio = QLabel('Cropping ratio')
		self.hslider_crop_ratio = QSlider(Qt.Horizontal)
		self.hslider_crop_ratio.setRange(1,50)
		self.hslider_crop_ratio.setValue(FocusTracking['Cropped image ratio']['default'])
		self.spinbox_crop_ratio=QSpinBox()
		self.spinbox_crop_ratio.setRange(1,50)
		self.spinbox_crop_ratio.setValue(FocusTracking['Cropped image ratio']['default'])
	
		slider_crop_ratio_layout=QHBoxLayout()
		slider_crop_ratio_layout.addWidget(self.label_crop_ratio)
		slider_crop_ratio_layout.addWidget(self.hslider_crop_ratio)
		slider_crop_ratio_layout.addWidget(self.spinbox_crop_ratio)
		group_slider_crop_ratio=QWidget()
		group_slider_crop_ratio.setLayout(slider_crop_ratio_layout)

		# Liquid lens freq
		self.label_lensFreq = QLabel('Liquid lens frequency (Hz)')
		self.hslider_lensFreq = QSlider(Qt.Horizontal)
		self.hslider_lensFreq.setRange(100*liquidLens['Freq']['min'],100*liquidLens['Freq']['max'])
		self.hslider_lensFreq.setValue(100*liquidLens['Freq']['default'])
		self.spinbox_lensFreq=QDoubleSpinBox()
		self.spinbox_lensFreq.setRange(liquidLens['Freq']['min'],liquidLens['Freq']['max'])
		self.spinbox_lensFreq.setSingleStep(liquidLens['Freq']['step'])
		self.spinbox_lensFreq.setValue(liquidLens['Freq']['default'])
		
		slider_lensFreq_layout=QHBoxLayout()
		slider_lensFreq_layout.addWidget(self.label_lensFreq)
		slider_lensFreq_layout.addWidget(self.hslider_lensFreq)
		slider_lensFreq_layout.addWidget(self.spinbox_lensFreq)
		group_slider_lensFreq=QWidget()
		group_slider_lensFreq.setLayout(slider_lensFreq_layout)

		# Liquid lens amplitude
		self.label_lensAmpl = QLabel('Liquid lens amplitude (mm)')
		self.hslider_lensAmpl = QSlider(Qt.Horizontal)
		self.hslider_lensAmpl.setRange(100*liquidLens['Amp']['min'],100*liquidLens['Amp']['max'])
		self.hslider_lensAmpl.setValue(2*liquidLens['Amp']['default'])
		self.spinbox_lensAmpl=QDoubleSpinBox()
		self.spinbox_lensAmpl.setRange(liquidLens['Amp']['min'], liquidLens['Amp']['max'])
		self.spinbox_lensAmpl.setSingleStep(liquidLens['Amp']['step'])
		self.spinbox_lensAmpl.setValue(2*liquidLens['Amp']['default'])
		
		slider_lensAmpl_layout=QHBoxLayout()
		slider_lensAmpl_layout.addWidget(self.label_lensAmpl)
		slider_lensAmpl_layout.addWidget(self.hslider_lensAmpl)
		slider_lensAmpl_layout.addWidget(self.spinbox_lensAmpl)
		group_slider_lensAmpl=QWidget()
		group_slider_lensAmpl.setLayout(slider_lensAmpl_layout)

		self.groupbox_FocusTracking = QGroupBox('Focus Tracking')

		# layout
		groupbox_layout_FocusTracking = QGridLayout()
		groupbox_layout_FocusTracking.addWidget(self.button_FocusTracking,0,0,1,1)
		groupbox_layout_FocusTracking.addWidget(group_slider_crop_ratio,0,1,1,1)
		groupbox_layout_FocusTracking.addWidget(group_slider_lensFreq,1,0,1,2)  
		groupbox_layout_FocusTracking.addWidget(group_slider_lensAmpl,2,0,1,2)
		# groupbox_layout_YTracking.addWidget(group_slider_lensGain) 
		# self.groupbox_YTracking.setLayout(groupbox_layout_YTracking)

		self.setLayout(groupbox_layout_FocusTracking)

		# Connections
		self.button_FocusTracking.clicked.connect(self.button_focusTracking_clicked)

		self.hslider_crop_ratio.valueChanged.connect(self.spinbox_crop_ratio_setValue)
		self.spinbox_crop_ratio.valueChanged.connect(self.hslider_crop_ratio_setValue)

		self.hslider_lensFreq.valueChanged.connect(self.spinbox_lensFreq_setValue)
		self.spinbox_lensFreq.valueChanged.connect(self.hslider_lensFreq_setValue)

		self.hslider_lensAmpl.valueChanged.connect(self.spinbox_lensAmpl_setValue)
		self.spinbox_lensAmpl.valueChanged.connect(self.hslider_lensAmpl_setValue)




	def button_focusTracking_clicked(self):
		
		if self.button_FocusTracking.isChecked():
			
			# Set the internal state value
			self.internal_state.data['track_focus'] = True

			self.microcontroller.send_focus_tracking_command(True)

			# Start the liquid lens sweep
			self.trackingController.tracker_focus.liquid_lens.start()
			self.button_FocusTracking.setText("Stop Focus Tracking")

			
		else:
			# Set the internal state value
			self.internal_state.data['track_focus'] = False
			self.microcontroller.send_focus_tracking_command(False)
			self.trackingController.tracker_focus.liquid_lens.stop()
			self.button_FocusTracking.setText("Start Focus Tracking")
			
	def spinbox_crop_ratio_setValue(self, value):
		newvalue=int(value)

		self.spinbox_crop_ratio.setValue(newvalue)

		self.trackingController.set_cropped_image_size(newvalue)

	def hslider_crop_ratio_setValue(self, value):
		newvalue=int(value)

		self.hslider_crop_ratio.setValue(newvalue)



	def spinbox_lensAmpl_setValue(self,value):
		newvalue=float(value)/100.
		self.spinbox_lensAmpl.setValue(newvalue)

		

		# Also send the amplitude change to the liquid lens
		# @@@@@@ To Implement @@@@@@@

		self.trackingController.tracker_focus.set_Amp(newvalue/2)

		self.trackingController.tracker_focus.liquid_lens.set_Amp(newvalue/2)


		# self.object_tracking.liquid_lens_ampl=newvalue/2
		# self.object_tracking.ytracker.set_ampl(newvalue/2)
		# # Now we need to also send the new amplitude to the liquid lens
		# self.object_tracking.liquid_lens.changeAmp(newvalue/2)

	def hslider_lensAmpl_setValue(self,value):
		self.hslider_lensAmpl.setValue(int(value*100))

	# Setting liquid lens current functions (for optical characterization)
	# def spinbox_lensAmpl_setValue(self,value):
	# 	newvalue = int(value)
	# 	self.spinbox_lensAmpl.setValue(newvalue)
	# 	# self.object_tracking.liquid_lens_ampl=newvalue/2
	# 	# self.object_tracking.ytracker.set_ampl(newvalue/2)
	# 	# Now we need to also send the new amplitude to the liquid lens
	# 	# self.object_tracking.liquid_lens.changeAmp(newvalue/2)
	# 	self.object_tracking.liquid_lens.sendCurrent(newvalue)

	# def hslider_lensAmpl_setValue(self,value):
	# 	self.hslider_lensAmpl.setValue(int(value))

	def spinbox_lensFreq_setValue(self,value):
		newvalue=float(value)/100.
		self.spinbox_lensFreq.setValue(newvalue)

		self.trackingController.tracker_focus.set_Freq(newvalue)

		self.trackingController.tracker_focus.liquid_lens.set_Freq(newvalue)

		# Send new frequency value to microcontroller
		self.microcontroller.send_liquid_lens_freq(newvalue)

		# Also send the amplitude change to the liquid lens
		# @@@@@@ To Implement @@@@@@@

		# self.object_tracking.liquid_lens_freq=newvalue 
		# # self.set_Y_buffers_lenght()
		# self.object_tracking.ytracker.set_freq(newvalue)
		# Now we need to also send the new frequency to the liquid lens

		# self.object_tracking.liquid_lens.changeFreq(newvalue)

	def hslider_lensFreq_setValue(self,value):
		new_value = int(value*100)
		self.hslider_lensFreq.setValue(new_value)





# class PlotDisplay_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass
