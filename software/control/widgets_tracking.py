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

class TrackingControllerWidget(QFrame):
	'''
	Buttons to start image tracking
	Display window to show thresholded images
	Slider bars to threshold image
	Radio-buttons to choose trackers.
	Text boxes for base path and Experiment ID.

	'''
	def __init__(self, streamHandler, trackingController, trackingDataSaver, internal_state, ImageDisplayWindow, microcontroller, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.base_path_is_set = False

		self.streamHandler = streamHandler
		self.trackingController = trackingController

		self.trackingDataSaver = trackingDataSaver

		self.internal_state = internal_state

		self.ImageDisplayWindow = ImageDisplayWindow

		self.microcontroller = microcontroller

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

			# Start a new track. If 'Acquire' is true this also creates a track file.
			# Internal state is changed after creating this file.
				# Update the internal_state to indicate that object should be tracked using image proc
			self.internal_state.data['track_obj_image'] = True

			print('Set track_obj_image to : {}'.format(self.internal_state.data['track_obj_image']))
			
			self.microcontroller.send_tracking_command(True)

			self.trackingDataSaver.start_new_track()
			self.streamHandler.start_tracking()

		else:
			self.streamHandler.stop_tracking()
			self.internal_state.data['track_obj_image'] = False

			# Send the track_obj_image flag to uController
			self.microcontroller.send_tracking_command(False)
			# Resets the track deques and counters

			self.trackingController.initialise_track()

	# This function is connected to the signal from tracking Controller triggered by 
	# hardware start-tracking input.
	def handle_hardware_track_signal(self):

		self.btn_track.toggle()
		self.do_track_button_tasks()

	def handle_aquisition_widget_track_signal(self):
		self.btn_track.setChecked(True)


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



		# self.camera_functions[self.tracking_channel].lower_HSV=np.uint8(LOWER)
		# # self.object_tracking.lower_HSV=np.uint8(LOWER)
		# self.camera_functions[self.tracking_channel].upper_HSV=np.uint8(UPPER)
		# self.object_tracking.upper_HSV=np.uint8(UPPER		


class NavigationWidget(QFrame):
	
	def __init__(self, navigationController, internal_state, microcontroller,  main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.navigationController = navigationController
		self.internal_state = internal_state
		self.microcontroller = microcontroller
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		# Stage position display 

		self.pos_X_label = pg.ValueLabel(siPrefix=True, suffix = 'm')
		self.pos_X_label.setValue(0)


		self.pos_Y_label = pg.ValueLabel(siPrefix=True, suffix = 'm')
		self.pos_Y_label.setValue(0)
	

		self.pos_Theta_label = pg.ValueLabel(siPrefix=True, suffix = 'rad')
		self.pos_Theta_label.setValue(0)

		stage_pos_layout = QGridLayout()

		stage_pos_layout.addWidget(QLabel('X-stage (mm)'),0,0)
		stage_pos_layout.addWidget(self.pos_X_label, 0,1)
		stage_pos_layout.addWidget(QLabel('Y-stage (mm)'),1,0)
		stage_pos_layout.addWidget(self.pos_Y_label, 1,1)
		stage_pos_layout.addWidget(QLabel('Rotational-stage (deg)'),2,0)
		stage_pos_layout.addWidget(self.pos_Theta_label, 2,1)

		self.stage_position = QGroupBox('Stage positions')

		self.stage_position.setLayout(stage_pos_layout)


		# Stage zeroing buttons
		self.zero_X = QPushButton('Zero X-stage')
		
		self.zero_Y = QPushButton('Zero Y-stage')
	
		self.zero_Theta = QPushButton('Zero Rotation-stage')
	
		
		# Homing Button
		self.homing_button = pg.FeedbackButton('Run Homing')

		stage_control = QVBoxLayout()

		stage_control.addWidget(self.homing_button)
		stage_control.addWidget(self.zero_X)
		stage_control.addWidget(self.zero_Y)
		stage_control.addWidget(self.zero_Theta)

		self.stage_control_group = QGroupBox('Stage control')

		self.stage_control_group.setLayout(stage_control)

		layout = QGridLayout()

		layout.addWidget(self.stage_position, 0,0,1,1)
		layout.addWidget(self.stage_control_group, 0,1,1,1)
		

		self.setLayout(layout)


		# Connections
		self.zero_X.clicked.connect(self.zero_X_stage)
		self.zero_Y.clicked.connect(self.zero_Y_stage)
		self.zero_Theta.clicked.connect(self.zero_Theta_stage)

		self.homing_button.clicked.connect(self.homing_button_click)

	def zero_X_stage(self):

		self.internal_state.data['Zero_stage'] = 1
		self.microcontroller.send_stage_zero_command('X')
	
	def zero_Y_stage(self):

		self.internal_state.data['Zero_stage'] = 2
		self.microcontroller.send_stage_zero_command('Y')

	def zero_Theta_stage(self):

		self.internal_state.data['Zero_stage'] = 3
		self.microcontroller.send_stage_zero_command('T')

	# Triggered by microController_Receiever
	def update_display(self):
		
		self.pos_X_label.setValue(self.internal_state.data['X_stage']*1e-3)
		self.pos_Y_label.setValue(self.internal_state.data['Y_stage']*1e-3)
		self.pos_Theta_label.setValue(self.internal_state.data['Theta_stage'])

	def homing_button_click(self):

		# Update the internal homing command state
		self.internal_state.data['homing_command'] = True

		# Send homing command to microcontroller
		self.microcontroller.send_homing_command()

		# self.homing_button.processing('Homing stages...')

		#@@@@ Hard-coding this to check button function
		# time.sleep(2.0)
		# self.internal_state.data['homing_state'] = True

	# Can implement later if necessary
	def homing_button_feedback(self):

		if(self.internal_state.data['homing_state']):
			self.homing_button.success('Homing completed!')
			self.homing_button.setText('Homing complete')

		else:
			self.homing_button.failure('Homing failed!')



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
