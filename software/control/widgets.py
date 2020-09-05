# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

import pyqtgraph as pg
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel
import control.utils.dockareaStyle as dstyle



from collections import deque

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

class CameraSettingsWidget(QFrame):

	def __init__(self, camera, liveController, main=None, *args, **kwargs):

		super().__init__(*args, **kwargs)
		self.camera = camera
		self.liveController = liveController
		# add components to self.grid
		self.add_components()        
		# set frame style
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		# add buttons and input fields
		self.entry_exposureTime = QDoubleSpinBox()
		self.entry_exposureTime.setMinimum(self.camera.EXPOSURE_TIME_MS_MIN) 
		self.entry_exposureTime.setMaximum(self.camera.EXPOSURE_TIME_MS_MAX) 
		self.entry_exposureTime.setSingleStep(1)
		self.entry_exposureTime.setValue(20)
		# self.camera.set_exposure_time(20)

		self.entry_analogGain = QDoubleSpinBox()
		self.entry_analogGain.setMinimum(self.camera.GAIN_MIN) 
		self.entry_analogGain.setMaximum(self.camera.GAIN_MAX) 
		self.entry_analogGain.setSingleStep(self.camera.GAIN_STEP)
		self.entry_analogGain.setValue(10)
		self.camera.set_analog_gain(10)

		self.entry_exposureTime_Preset = QDoubleSpinBox()
		self.entry_exposureTime_Preset.setMinimum(self.camera.EXPOSURE_TIME_MS_MIN)
		self.entry_exposureTime_Preset.setMaximum(self.camera.EXPOSURE_TIME_MS_MAX)
		self.entry_exposureTime_Preset.setSingleStep(1)
		self.entry_exposureTime_Preset.setValue(20)
		# self.liveController.set_exposure_time_preset(20)

		self.entry_analogGain_Preset = QDoubleSpinBox()
		self.entry_analogGain_Preset.setMinimum(self.camera.GAIN_MIN) 
		self.entry_analogGain_Preset.setMaximum(self.camera.GAIN_MAX) 
		self.entry_analogGain_Preset.setSingleStep(self.camera.GAIN_STEP)
		self.entry_analogGain_Preset.setValue(0)
		# self.liveController.set_analog_gain_preset(0)


		self.btn_Preset = QPushButton("Preset")
		self.btn_Preset.setDefault(False)
   

		# connection
		self.btn_Preset.clicked.connect(self.load_preset)
		
		self.entry_exposureTime.valueChanged.connect(self.camera.set_exposure_time)
		self.entry_analogGain.valueChanged.connect(self.camera.set_analog_gain)
		self.entry_exposureTime_Preset.valueChanged.connect(self.liveController.set_exposure_time_bfdf_preset)
		self.entry_analogGain_Preset.valueChanged.connect(self.liveController.set_analog_gain_bfdf_preset)
		
		# layout
		grid_ctrl = QGridLayout()
		grid_ctrl.addWidget(QLabel('Exposure Time (ms)'), 0,0)
		grid_ctrl.addWidget(self.entry_exposureTime, 0,1)
		grid_ctrl.addWidget(QLabel('Analog Gain'), 1,0)
		grid_ctrl.addWidget(self.entry_analogGain, 1,1)

		grid_ctrl_preset = QGridLayout()
		grid_ctrl_preset.addWidget(self.entry_exposureTime_Preset, 0,0)
		grid_ctrl_preset.addWidget(self.entry_analogGain_Preset, 0,1)
		grid_ctrl_preset.addWidget(self.btn_Preset, 0,2)
	  

		self.grid = QGridLayout()
		self.grid.addLayout(grid_ctrl,0,0)
		self.grid.addLayout(grid_ctrl_preset,1,0)

		self.setLayout(self.grid)

	def load_preset(self):
		self.entry_exposureTime.setValue(self.entry_exposureTime_Preset.value())
		self.entry_exposureTime.repaint() # update doesn't work
		self.entry_analogGain.setValue(self.entry_analogGain_Preset.value())
		self.entry_analogGain.repaint()


class LiveControlWidget(QGroupBox):
	'''
	Widget controls salient microscopy parameters such as:
		- Live Button
		- Checkboxes to choose active image-streams.
		- Objective
		- Display resolution slider
	'''
	def __init__(self, streamHandler, liveController, trackingController, camera, imageDisplayWindow, imageDisplayWindow_threshImage, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.setTitle('Live Controller')
		self.liveController = liveController
		self.streamHandler = streamHandler
		self.trackingController = trackingController
		self.camera = camera
		self.imageDisplayWindow = imageDisplayWindow
		self.imageDisplayWindow_threshImage = imageDisplayWindow_threshImage
		self.imaging_channels = CAMERAS.keys()


		self.objective = DEFAULT_OBJECTIVE
		
		self.add_components()
		self.update_pixel_size()

		# self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		self.btn_live = QPushButton("Live")
		self.btn_live.setCheckable(True)
		self.btn_live.setChecked(False)
		self.btn_live.setDefault(False)

		self.checkbox = {}
		for channel in self.imaging_channels:
			self.checkbox[channel] = QCheckBox(channel)

		# 0,1 : choose tracking objective
		self.dropdown_objectiveSelection = QComboBox()
		self.dropdown_objectiveSelection.addItems(list(OBJECTIVES.keys()))
		self.dropdown_objectiveSelection.setCurrentText(DEFAULT_OBJECTIVE)

		# Display resolution slider
		self.slider_resolutionScaling = QSlider(Qt.Horizontal)
		self.slider_resolutionScaling.setTickPosition(QSlider.TicksBelow)
		self.slider_resolutionScaling.setMinimum(10)
		self.slider_resolutionScaling.setMaximum(100)
		self.slider_resolutionScaling.setValue(WORKING_RES_DEFAULT*100)
		self.slider_resolutionScaling.setSingleStep(10)

		self.display_workingResolution = QLCDNumber()
		self.display_workingResolution.setNumDigits(6)
		self.display_workingResolution.display(0.0)

		# @@@ To implement: Multi-image channel support.
		# Each image channel can be checked ON or OFF so that we can switch between multiple imaging modalities
		# This will be a checkbox, where the tracking stream is checked by default.



		# connections

		self.slider_resolutionScaling.valueChanged.connect(self.streamHandler.set_working_resolution_scaling)
		self.slider_resolutionScaling.valueChanged.connect(self.update_image_properties_tracking)
		# self.dropdown_modeSelection.currentIndexChanged.connect(self.update_microscope_mode)
		self.dropdown_objectiveSelection.currentIndexChanged.connect(self.update_pixel_size)
		self.btn_live.clicked.connect(self.toggle_live)

		for channel in self.imaging_channels:
			self.checkbox[channel].clicked.connect(self.update_active_channels)


		# Layout

		# checkbox layout
		checkbox_group = QGroupBox('Imaging channels')
		checkbox_layout = QGridLayout()
		for column, channel in enumerate(self.imaging_channels):
			checkbox_layout.addWidget(self.checkbox[channel],0,column)
		# checkbox_group.setLayout(checkbox_layout)

		objective_layout = QHBoxLayout()
		objective_layout.addWidget(QLabel('Objective'))
		objective_layout.addWidget(self.dropdown_objectiveSelection)
	  
		working_resolution_group = QGroupBox('Working resolution')
		working_resolution_layout = QHBoxLayout()
		working_resolution_layout.addWidget(self.slider_resolutionScaling)
		working_resolution_layout.addWidget(self.display_workingResolution)
		working_resolution_group.setLayout(working_resolution_layout)

		self.grid = QGridLayout()
		self.grid.addWidget(self.btn_live,0,0)
		self.grid.addLayout(objective_layout,0,1)
		self.grid.addLayout(checkbox_layout,1,0)
		self.grid.addWidget(working_resolution_group,1,1)

		self.setLayout(self.grid)

	
	# Slot connected to signal from trackingController.
	def update_working_resolution(self, value):

		self.display_workingResolution.display(value)

	def update_pixel_size(self):
		self.objective = self.dropdown_objectiveSelection.currentText()
		print('new objective: {}'.format(self.objective))
		new_pixel_size = OBJECTIVES[self.objective]['PixelPermm']
		self.trackingController.units_converter.update_pixel_size(new_pixel_size)
		
	def toggle_live(self,pressed):
		if pressed:
			for channel in self.imaging_channels:
				self.checkbox[channel].setEnabled(False)
				if(self.checkbox[channel].isChecked()):
					self.liveController[channel].start_live()
					
		else:
			for channel in self.imaging_channels:
				self.liveController[channel].stop_live()
				self.checkbox[channel].setEnabled(True)

	def update_image_properties_tracking(self):
		# If the image resolution is changed on the fly then restart the image tracker.
		self.trackingController.start_flag = True
		# Also update the image sizes for use in tracking.
		self.trackingController.update_image_center_width()
	# def update_microscope_mode(self,index):
	#     self.liveController.turn_off_illumination()
	#     self.liveController.set_microscope_mode(self.dropdown_modeSelection.currentText())

	def update_active_channels(self):
		print('Updating active channels')
		for channel in self.imaging_channels:
			if(self.checkbox[channel].isChecked()):
				# Make window active/Show
				self.imageDisplayWindow[channel].show()
				if(channel==TRACKING):
					self.imageDisplayWindow_threshImage.show()
				print('Show {} window'.format(channel))
			elif (self.checkbox[channel].isChecked()==False):
				# Hide the window.
				self.imageDisplayWindow[channel].hide()
				if(channel==TRACKING):
					self.imageDisplayWindow_threshImage.hide()
				print('Hide {} window'.format(channel))

class StreamControlWidget(QFrame):
	'''
	Widget controls image/video-stream parameters:
		- Trigger mode (Hardware, Software, Continuous acquisition)
		- Trigger FPS (Set and Actual). Set value only matters during Software trigger. 
		- Display fps (Set and Actual).
	'''
	def __init__(self, streamHandler, liveController, camera, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.liveController = liveController
		self.streamHandler = streamHandler
		self.camera = camera

		self.fps_display = FPS['display']['default']
		self.fps_trigger = FPS['trigger_software']['default']

		self.streamHandler.set_display_fps(self.fps_display)
		self.liveController.set_trigger_fps(self.fps_trigger)
		
		self.add_components()

	def add_components(self):

		# Widgets

		# Trigger Mode
		self.triggerMode = None
		self.dropdown_triggerMode = QComboBox()
		self.dropdown_triggerMode.addItems([TriggerMode.SOFTWARE,TriggerMode.HARDWARE,TriggerMode.CONTINUOUS])
		self.dropdown_triggerMode.setCurrentText(TriggerMode.SOFTWARE)

		# Trigger FPS
		self.entry_triggerFPS = QDoubleSpinBox()
		self.entry_triggerFPS.setMinimum(FPS['trigger_software']['min']) 
		self.entry_triggerFPS.setMaximum(FPS['trigger_software']['max']) 
		self.entry_triggerFPS.setSingleStep(1)
		self.entry_triggerFPS.setValue(self.fps_trigger)

		# Trigger FPS actual
		self.actual_streamFPS = QLCDNumber()
		self.actual_streamFPS.setNumDigits(4)
		self.actual_streamFPS.display(0.0)

		# Entry display fps
		self.entry_displayFPS = QDoubleSpinBox()
		self.entry_displayFPS.setMinimum(FPS['display']['min']) 
		self.entry_displayFPS.setMaximum(FPS['display']['max']) 
		self.entry_displayFPS.setSingleStep(1)
		self.entry_displayFPS.setValue(FPS['display']['default'])

		 # Display fps actual
		self.actual_displayFPS = QLCDNumber()
		self.actual_displayFPS.setNumDigits(4)
		self.actual_displayFPS.display(0.0)


		# Layout
		triggerMode_layout = QHBoxLayout()
		triggerMode_layout.addWidget(QLabel('Trigger mode'))
		triggerMode_layout.addWidget(self.dropdown_triggerMode)


		trigger_fps_group = QGroupBox('Trigger FPS')
		trigger_fps_layout = QGridLayout()
		
		trigger_fps_layout.addWidget(QLabel('Set'),0,0)
		trigger_fps_layout.addWidget(self.entry_triggerFPS, 0,1)
		trigger_fps_layout.addWidget(QLabel('Actual'),0,2)
		trigger_fps_layout.addWidget(self.actual_streamFPS, 0,3)
		trigger_fps_group.setLayout(trigger_fps_layout)

		display_fps_group = QGroupBox('Display FPS')
		display_fps_layout = QGridLayout()
		
		display_fps_layout.addWidget(QLabel('Set'),0,0)
		display_fps_layout.addWidget(self.entry_displayFPS, 0,1)
		display_fps_layout.addWidget(QLabel('Actual'),0,2)
		display_fps_layout.addWidget(self.actual_displayFPS, 0,3)
		display_fps_group.setLayout(display_fps_layout)

		# Overall layout
		self.grid = QGridLayout()
		# self.grid.addLayout(microscope_mode_layout,0,0)
		self.grid.addLayout(triggerMode_layout,0,0)
		self.grid.addWidget(trigger_fps_group,1,0)
		self.grid.addWidget(display_fps_group,2,0)
		self.setLayout(self.grid)

		# Connections
		self.dropdown_triggerMode.currentIndexChanged.connect(self.update_trigger_mode)
		self.entry_displayFPS.valueChanged.connect(self.streamHandler.set_display_fps)
		self.entry_triggerFPS.valueChanged.connect(self.liveController.set_trigger_fps)

	 

	 # Slot connected to signal from streamHandler.
	def update_display_fps(self, value):

		self.actual_displayFPS.display(value)


	# Slot connected to signal from streamHandler.
	def update_stream_fps(self, value):

		self.actual_streamFPS.display(value)

	def update_trigger_mode(self):
		self.triggerMode = self.dropdown_triggerMode.currentText()

		if(self.triggerMode == TriggerMode.SOFTWARE):
			print('Trigger mode to: {}'.format(self.triggerMode))
			self.camera.set_software_triggered_acquisition()

		elif(self.triggerMode == TriggerMode.HARDWARE):
			print('Trigger mode to: {}'.format(self.triggerMode))
			self.camera.set_hardware_triggered_acquisition()
		else:
			print('Trigger mode to: {}'.format(self.triggerMode))
			self.camera.set_continuous_acquisition()

	  






class RecordingWidget(QFrame):
	def __init__(self, streamHandler, imageSaver, internal_state, trackingControllerWidget, trackingDataSaver = None, imaging_channels = TRACKING, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		# In general imageSaver, streamHandler are dicts corresponding to each image_channel
		self.imageSaver = imageSaver # for saving path control
		self.streamHandler = streamHandler
		self.internal_state = internal_state 
		self.trackingDataSaver = trackingDataSaver
		self.trackingControllerWidget = trackingControllerWidget
		self.imaging_channels = imaging_channels

		self.tracking_flag = False
		self.recordingOnly_flag = False

		self.save_dir_base = DEFAULT_SAVE_FOLDER
		self.base_path_is_set = False
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.btn_setSavingDir = QPushButton('Browse')
		self.btn_setSavingDir.setDefault(False)
		self.btn_setSavingDir.setIcon(QIcon('icon/folder.png'))
		
		self.lineEdit_savingDir = QLineEdit()
		self.lineEdit_savingDir.setReadOnly(True)
		self.lineEdit_savingDir.setText('Choose a base saving directory')

		self.lineEdit_experimentID = QLineEdit()

		self.checkbox = {}
		self.entry_saveFPS = {}
		self.actual_saveFPS = {}
		self.entry_timeLimit = {}

		# Check-boxes to select the image channels to save
		for channel in self.imaging_channels:

			self.checkbox[channel] = QCheckBox(channel)

			# SpinBox for specifying save FPS of each stream

			self.entry_saveFPS[channel] = QDoubleSpinBox()
			self.entry_saveFPS[channel].setMinimum(0.02) 
			self.entry_saveFPS[channel].setMaximum(200) 
			self.entry_saveFPS[channel].setSingleStep(1)
			self.entry_saveFPS[channel].setValue(1)
			self.streamHandler[channel].set_save_fps(1)

			# LCD for displaying the actual save FPS
			self.actual_saveFPS[channel] = QLCDNumber()
			self.actual_saveFPS[channel].setNumDigits(4)
			self.actual_saveFPS[channel].display(0.0)

			# SpinBox for specifying recording time limit for each stream
			self.entry_timeLimit[channel] = QSpinBox()
			self.entry_timeLimit[channel].setMinimum(-1) 
			self.entry_timeLimit[channel].setMaximum(60*60*24*30) 
			self.entry_timeLimit[channel].setSingleStep(1)
			self.entry_timeLimit[channel].setValue(-1)

		

		self.radioButton_tracking = QRadioButton("Track+Record")
		self.radioButton_recording = QRadioButton("Record")

		self.btn_record = QPushButton("Record")
		self.btn_record.setCheckable(True)
		self.btn_record.setChecked(False)
		self.btn_record.setDefault(False)

		grid_line1 = QGridLayout()
		grid_line1.addWidget(QLabel('Saving Path'))
		grid_line1.addWidget(self.lineEdit_savingDir, 0,1)
		grid_line1.addWidget(self.btn_setSavingDir, 0,2)

		grid_line2 = QGridLayout()
		grid_line2.addWidget(QLabel('Experiment ID'), 0,0)
		grid_line2.addWidget(self.lineEdit_experimentID,0,1)


		self.tracking_recording_group = QGroupBox()

		tracking_recording_layout = QHBoxLayout()
		tracking_recording_layout.addWidget(self.btn_record)
		tracking_recording_layout.addWidget(self.radioButton_tracking)
		tracking_recording_layout.addWidget(self.radioButton_recording)

		self.tracking_recording_group.setLayout(tracking_recording_layout)


		
		imaging_channel_box = QGroupBox('Imaging channels')

		box_layout = QGridLayout()

		box_layout.addWidget(QLabel('Channel'), 0,0,1,1)
		box_layout.addWidget(QLabel('Save FPS'), 0,1,1,1)
		box_layout.addWidget(QLabel('Actual FPS'), 0,2,1,1)
		box_layout.addWidget(QLabel('Time limit'), 0,3,1,1)

		for row, channel in enumerate(self.imaging_channels):
			box_layout.addWidget(self.checkbox[channel], row+1, 0, 1, 1)
			box_layout.addWidget(self.entry_saveFPS[channel], row+1, 1, 1, 1)
			box_layout.addWidget(self.actual_saveFPS[channel], row+1, 2, 1, 1)
			box_layout.addWidget(self.entry_timeLimit[channel], row+1, 3, 1, 1)


		self.grid = QGridLayout()
		
		self.grid.addLayout(box_layout,0,0,1,1)
	   
		self.grid.addLayout(grid_line1,1,0,1,1)
		self.grid.addLayout(grid_line2,2,0,1,1)

		# self.grid.addWidget(self.btn_record,3,0,1,1)
		self.grid.addWidget(self.tracking_recording_group,3,0,1,1)
		

		self.setLayout(self.grid)

		# add and display a timer - to be implemented
		# self.timer = QTimer()

		# connections
		self.btn_setSavingDir.clicked.connect(self.set_saving_dir)
		self.btn_record.clicked.connect(self.toggle_recording)

		self.radioButton_recording.clicked.connect(self.set_tracking_recording_flag)
		self.radioButton_tracking.clicked.connect(self.set_tracking_recording_flag)



		for channel in self.imaging_channels:
			self.entry_saveFPS[channel].valueChanged.connect(self.streamHandler[channel].set_save_fps)
			self.entry_timeLimit[channel].valueChanged.connect(self.imageSaver[channel].set_recording_time_limit)
			self.imageSaver[channel].stop_recording.connect(self.stop_recording)

	def set_saving_dir(self, use_default_dir = False):
		
		if(use_default_dir is False):
			dialog = QFileDialog()
			self.save_dir_base = dialog.getExistingDirectory(None, "Select Folder")

		# Set base path for image saver
		for channel in self.imaging_channels:
			if(self.checkbox[channel].isChecked()):
				self.imageSaver[channel].set_base_path(self.save_dir_base)
		
		# set the base path for the data saver
		if(self.trackingDataSaver is not None):
			self.trackingDataSaver.set_base_path(self.save_dir_base)
		
		self.lineEdit_savingDir.setText(self.save_dir_base)
		self.base_path_is_set = True

	def toggle_recording(self,pressed):
		if self.base_path_is_set == False and self.save_dir_base is None:
			self.btn_record.setChecked(False)
			msg = QMessageBox()
			msg.setText("Please choose base saving directory first")
			msg.exec_()
			return
		elif self.save_dir_base is not None:
			self.set_saving_dir(use_default_dir = True)

		if pressed:
			self.internal_state.data['Acquisition'] = True
			self.lineEdit_experimentID.setEnabled(False)
			self.btn_setSavingDir.setEnabled(False)

			for channel in self.imaging_channels:
				self.checkbox[channel].setEnabled(False)

			
			if(self.trackingDataSaver is not None and self.recordingOnly_flag==False):
				self.trackingControllerWidget.btn_track.setChecked(True)
				self.trackingDataSaver.start_new_experiment(self.lineEdit_experimentID.text())
			else:
				pass

			for channel in self.imaging_channels:

				if(self.checkbox[channel].isChecked()):
					self.imageSaver[channel].start_saving_images()
					self.streamHandler[channel].start_recording()

		else:
			self.internal_state.data['Acquisition']= False
			
			for channel in self.imaging_channels:
				self.streamHandler[channel].stop_recording()
				self.checkbox[channel].setEnabled(True)
			
			self.lineEdit_experimentID.setEnabled(True)
			self.btn_setSavingDir.setEnabled(True)
			

	# stop_recording can be called by imageSaver
	def stop_recording(self):
		self.lineEdit_experimentID.setEnabled(True)
		self.btn_record.setChecked(False)
		for channel in self.imaging_channels:
				self.streamHandler[channel].stop_recording()
				self.checkbox[channel].setEnabled(True)
		self.btn_setSavingDir.setEnabled(True)

	def set_tracking_recording_flag(self):

		if(self.radioButton_tracking.isChecked()):
			self.recordingOnly_flag = False
			print('Set mode to Tracking+Rec')
		elif(self.radioButton_recording.isChecked()):
			self.recordingOnly_flag = True
			print('Set mode to Recording only')

	def update_save_fps(self, channel, real_fps):

		self.actual_saveFPS[channel].display(real_fps)



'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Plot widget
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class dockAreaPlot(dock.DockArea):
	def __init__(self, parent=None):
		super().__init__(parent)
		DockLabel.updateStyle = dstyle.updateStylePatched

		self.plots = {key:PlotWidget(key) for key in PLOT_VARIABLES.keys()}
		
		self.docks = {key:dock.Dock(key) for key in PLOT_VARIABLES.keys()}

		for key in PLOT_VARIABLES.keys():

			self.docks[key].addWidget(self.plots[key])
		
		# Layout of the plots
		
		self.addDock(self.docks['X'])
		self.addDock(self.docks['Z'],'right',self.docks['X'])

		prev_key = 'Z'
		for key in PLOT_VARIABLES:
			if key not in DEFAULT_PLOTS:

				self.addDock(self.docks[key],'above',self.docks[prev_key])
				prev_key = key

	def initialise_plot_area(self):

		for key in self.plots.keys():
			self.plot[key].initialise_plot()


class PlotWidget(pg.GraphicsLayoutWidget):
	def __init__(self,title, parent=None):
		super().__init__(parent)
		self.title=title
		#plot Zobj
		self.Abscissa=deque(maxlen=20)
		self.Ordinate=deque(maxlen=20)
		
		self.Abs=[]
		self.Ord=[]
		self.plot1=self.addPlot(title=title)
		self.curve=self.plot1.plot(self.Abs,self.Ord)

		self.plot1.enableAutoRange('xy', True)
		self.plot1.showGrid(x=True, y=True)
		
		
	def update_plot(self,data):
		
		self.Abscissa.append(data[0])

		self.Ordinate.append(data[1])

		self.label = PLOT_UNITS[self.title]
			
		self.Abs=list(self.Abscisse)
		self.Ord=list(self.Ordonnee)

		self.curve.setData(self.Abs,self.Ord)

	def initialise_plot(self):
		self.Abscissa=deque(maxlen=20)
		self.Ordinate=deque(maxlen=20)
		self.Abs=[]
		self.Ord=[]
		self.curve.setData(self.Abs,self.Ord)

# class NavigationWidget(QFrame):
#     def __init__(self, navigationController, main=None, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         self.navigationController = navigationController
#         self.add_components()
#         self.setFrameStyle(QFrame.Panel | QFrame.Raised)

#     def add_components(self):
#         self.label_Xpos = QLabel()
#         self.label_Xpos.setNum(0)
#         self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
#         self.entry_dX = QDoubleSpinBox()
#         self.entry_dX.setMinimum(0) 
#         self.entry_dX.setMaximum(5) 
#         self.entry_dX.setSingleStep(0.2)
#         self.entry_dX.setValue(0)
#         self.btn_moveX_forward = QPushButton('Forward')
#         self.btn_moveX_forward.setDefault(False)
#         self.btn_moveX_backward = QPushButton('Backward')
#         self.btn_moveX_backward.setDefault(False)
		
#         self.label_Ypos = QLabel()
#         self.label_Ypos.setNum(0)
#         self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
#         self.entry_dY = QDoubleSpinBox()
#         self.entry_dY.setMinimum(0)
#         self.entry_dY.setMaximum(5)
#         self.entry_dY.setSingleStep(0.2)
#         self.entry_dY.setValue(0)
#         self.btn_moveY_forward = QPushButton('Forward')
#         self.btn_moveY_forward.setDefault(False)
#         self.btn_moveY_backward = QPushButton('Backward')
#         self.btn_moveY_backward.setDefault(False)

#         self.label_Zpos = QLabel()
#         self.label_Zpos.setNum(0)
#         self.label_Zpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
#         self.entry_dZ = QDoubleSpinBox()
#         self.entry_dZ.setMinimum(0) 
#         self.entry_dZ.setMaximum(1000) 
#         self.entry_dZ.setSingleStep(0.2)
#         self.entry_dZ.setValue(0)
#         self.btn_moveZ_forward = QPushButton('Forward')
#         self.btn_moveZ_forward.setDefault(False)
#         self.btn_moveZ_backward = QPushButton('Backward')
#         self.btn_moveZ_backward.setDefault(False)
		
#         grid_line0 = QGridLayout()
#         grid_line0.addWidget(QLabel('X (mm)'), 0,0)
#         grid_line0.addWidget(self.label_Xpos, 0,1)
#         grid_line0.addWidget(self.entry_dX, 0,2)
#         grid_line0.addWidget(self.btn_moveX_forward, 0,3)
#         grid_line0.addWidget(self.btn_moveX_backward, 0,4)

#         grid_line1 = QGridLayout()
#         grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
#         grid_line1.addWidget(self.label_Ypos, 0,1)
#         grid_line1.addWidget(self.entry_dY, 0,2)
#         grid_line1.addWidget(self.btn_moveY_forward, 0,3)
#         grid_line1.addWidget(self.btn_moveY_backward, 0,4)

#         grid_line2 = QGridLayout()
#         grid_line2.addWidget(QLabel('Z (um)'), 0,0)
#         grid_line2.addWidget(self.label_Zpos, 0,1)
#         grid_line2.addWidget(self.entry_dZ, 0,2)
#         grid_line2.addWidget(self.btn_moveZ_forward, 0,3)
#         grid_line2.addWidget(self.btn_moveZ_backward, 0,4)

#         self.grid = QGridLayout()
#         self.grid.addLayout(grid_line0,0,0)
#         self.grid.addLayout(grid_line1,1,0)
#         self.grid.addLayout(grid_line2,2,0)
#         self.setLayout(self.grid)

#         self.btn_moveX_forward.clicked.connect(self.move_x_forward)
#         self.btn_moveX_backward.clicked.connect(self.move_x_backward)
#         self.btn_moveY_forward.clicked.connect(self.move_y_forward)
#         self.btn_moveY_backward.clicked.connect(self.move_y_backward)
#         self.btn_moveZ_forward.clicked.connect(self.move_z_forward)
#         self.btn_moveZ_backward.clicked.connect(self.move_z_backward)
		
#     def move_x_forward(self):
#         self.navigationController.move_x(self.entry_dX.value())
#         print('move x')
#     def move_x_backward(self):
#         self.navigationController.move_x(-self.entry_dX.value())
#     def move_y_forward(self):
#         self.navigationController.move_y(self.entry_dY.value())
#     def move_y_backward(self):
#         self.navigationController.move_y(-self.entry_dY.value())
#     def move_z_forward(self):
#         self.navigationController.move_z(self.entry_dZ.value()/1000)
#     def move_z_backward(self):
#         self.navigationController.move_z(-self.entry_dZ.value()/1000)

class AutoFocusWidget(QFrame):
	def __init__(self, autofocusController, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.autofocusController = autofocusController
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.entry_delta = QDoubleSpinBox()
		self.entry_delta.setMinimum(0.2) 
		self.entry_delta.setMaximum(20) 
		self.entry_delta.setSingleStep(0.2)
		self.entry_delta.setValue(3)
		self.autofocusController.set_deltaZ(3)

		self.entry_N = QSpinBox()
		self.entry_N.setMinimum(3) 
		self.entry_N.setMaximum(20) 
		self.entry_N.setSingleStep(1)
		self.entry_N.setValue(10)
		self.autofocusController.set_N(10)

		self.btn_autofocus = QPushButton('Autofocus')
		self.btn_autofocus.setDefault(False)
		self.btn_autofocus.setCheckable(True)
		self.btn_autofocus.setChecked(False)

		# layout
		grid_line0 = QGridLayout()
		grid_line0.addWidget(QLabel('delta Z (um)'), 0,0)
		grid_line0.addWidget(self.entry_delta, 0,1)
		grid_line0.addWidget(QLabel('N Z planes'), 0,2)
		grid_line0.addWidget(self.entry_N, 0,3)
		grid_line0.addWidget(self.btn_autofocus, 0,4)

		self.grid = QGridLayout()
		self.grid.addLayout(grid_line0,0,0)
		self.setLayout(self.grid)
		
		# connections
		self.btn_autofocus.clicked.connect(self.autofocusController.autofocus)
		self.entry_delta.valueChanged.connect(self.autofocusController.set_deltaZ)
		self.entry_N.valueChanged.connect(self.autofocusController.set_N)
		self.autofocusController.autofocusFinished.connect(self.autofocus_is_finished)

	def autofocus_is_finished(self):
		self.btn_autofocus.setChecked(False)

