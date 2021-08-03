# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

import pyqtgraph as pg
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel
import control.utils.dockareaStyle as dstyle

import numpy as np
from collections import deque

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

class VolumetricImagingWidget(QFrame):
    
    def __init__(self, VolumetricImagingController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.volumetricImagingController = VolumetricImagingController
        self.volumetricImagingController.signal_volumetric_imaging_stopped.connect(self.slot_volumetric_imaging_stopped)
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        
        grid1 = QGridLayout()

        self.hslider_current_min = QSlider(Qt.Horizontal)
        self.hslider_current_min.setRange(-250,250)
        self.hslider_current_min.setValue(-20)
        self.entry_current_min=QDoubleSpinBox()
        self.entry_current_min.setRange(-250,250)
        self.entry_current_min.setSingleStep(0.2)
        self.entry_current_min.setValue(-20)
        hbox_current_min = QHBoxLayout()
        hbox_current_min.addWidget(QLabel('Min Current (mA)'))
        hbox_current_min.addWidget(self.hslider_current_min)
        hbox_current_min.addWidget(self.entry_current_min)
        self.hslider_current_min.valueChanged.connect(self.entry_current_min.setValue)
        self.entry_current_min.valueChanged.connect(self.volumetricImagingController.set_liquid_lens_scanning_current_min)
        self.entry_current_min.valueChanged.connect(self.check_current_max)
        grid1.addLayout(hbox_current_min,0,0,1,4)

        self.hslider_current_max = QSlider(Qt.Horizontal)
        self.hslider_current_max.setRange(-250,250)
        self.hslider_current_max.setValue(-20)
        self.entry_current_max=QDoubleSpinBox()
        self.entry_current_max.setRange(-250,250)
        self.entry_current_max.setSingleStep(0.2)
        self.entry_current_max.setValue(-20)
        hbox_current_max = QHBoxLayout()
        hbox_current_max.addWidget(QLabel('Max Current (mA)'))
        hbox_current_max.addWidget(self.hslider_current_max)
        hbox_current_max.addWidget(self.entry_current_max)
        self.hslider_current_max.valueChanged.connect(self.entry_current_max.setValue)
        self.entry_current_max.valueChanged.connect(self.volumetricImagingController.set_liquid_lens_scanning_current_max)
        self.entry_current_max.valueChanged.connect(self.check_current_min)
        grid1.addLayout(hbox_current_max,1,0,1,4)

        self.dropdown_frequency = QComboBox()
        self.dropdown_frequency.addItems(['0','0.1','0.2','0.5','1','2','5','8','10','12.5','16','20','25'])
        self.dropdown_frequency.setCurrentText(str(self.volumetricImagingController.frequency_Hz))
        self.dropdown_frequency.currentTextChanged.connect(lambda frequency:self.volumetricImagingController.set_liquid_lens_scanning_frequency(float(frequency)))
        grid1.addWidget(QLabel('Frequency (Hz)'),2,0,1,1)
        grid1.addWidget(self.dropdown_frequency,2,1,1,1)

        self.entry_number_of_planes_per_volume = QSpinBox()
        self.entry_number_of_planes_per_volume.setRange(1,1000)
        self.entry_number_of_planes_per_volume.setSingleStep(1)
        self.entry_number_of_planes_per_volume.setValue(self.volumetricImagingController.number_of_planes_per_volume)
        self.entry_number_of_planes_per_volume.valueChanged.connect(self.volumetricImagingController.set_number_of_planes_per_volume)
        grid1.addWidget(QLabel('Number of Planes Per Volume'),2,2,1,1)
        grid1.addWidget(self.entry_number_of_planes_per_volume,2,3,1,1)

        self.entry_number_of_requested_volumes = QSpinBox()
        self.entry_number_of_requested_volumes.setRange(0,100000)
        self.entry_number_of_requested_volumes.setSingleStep(1)
        self.entry_number_of_requested_volumes.setValue(0)
        self.entry_number_of_requested_volumes.valueChanged.connect(self.volumetricImagingController.set_number_of_requested_volumes)
        grid1.addWidget(QLabel('Number of Volumes'),3,0,1,1)
        grid1.addWidget(self.entry_number_of_requested_volumes,3,1,1,1)

        self.checkbox_save_images = QCheckBox('Save images')
        self.checkbox_save_images.setChecked(False)
        #grid1.addWidget(QLabel('Save Images'),3,2,1,1)
        grid1.addWidget(self.checkbox_save_images,3,2,1,2)
     
        self.btn_toggle_volumetric_imaging = QPushButton('Start Volumetric Imaging')
        self.btn_toggle_volumetric_imaging.setCheckable(True)
        self.btn_toggle_volumetric_imaging.setChecked(False)
        self.btn_toggle_volumetric_imaging.setDefault(False)
        self.btn_toggle_volumetric_imaging.clicked.connect(self.toggle_volumetric_imaging)
        grid1.addWidget(self.btn_toggle_volumetric_imaging,4,0,1,4)

        self.btn_toggle_recording = QPushButton('Start Recording')
        self.btn_toggle_recording.setCheckable(True)
        self.btn_toggle_recording.setChecked(False)
        self.btn_toggle_recording.setDefault(False)
        self.btn_toggle_recording.clicked.connect(self.toggle_recording)
        grid1.addWidget(self.btn_toggle_recording,5,0,1,4)

        # space
        grid1.addWidget(QLabel(''),6,0,1,4)

        self.btn_toggle_focus_measure_calculation = QPushButton('Enable Focus Measure Calculation')
        self.btn_toggle_focus_measure_calculation.setCheckable(True)
        self.btn_toggle_focus_measure_calculation.setChecked(False)
        self.btn_toggle_focus_measure_calculation.setDefault(False)
        self.btn_toggle_focus_measure_calculation.clicked.connect(self.volumetricImagingController.enable_focus_measure_calculation)
        grid1.addWidget(self.btn_toggle_focus_measure_calculation,7,0,1,2)

        self.btn_toggle_focus_tracking = QPushButton('Enable Focus Tracking')
        self.btn_toggle_focus_tracking.setCheckable(True)
        self.btn_toggle_focus_tracking.setChecked(False)
        self.btn_toggle_focus_tracking.setDefault(False)
        grid1.addWidget(self.btn_toggle_focus_tracking,7,2,1,2)

        self.display_defocus = QLCDNumber()
        self.display_defocus.setNumDigits(4)
        grid1.addWidget(QLabel('Defocus'),8,0,1,1)
        grid1.addWidget(self.display_defocus,8,1,1,1)

        self.entry_focus_offset = QDoubleSpinBox()
        self.entry_focus_offset.setRange(-250,250)
        self.entry_focus_offset.setSingleStep(0.2)
        self.entry_focus_offset.setValue(0)
        grid1.addWidget(QLabel('Focus Offset'),8,2,1,1)
        grid1.addWidget(self.entry_focus_offset,8,3,1,1)

        self.display_defocus_um = QLCDNumber()
        self.display_defocus_um.setNumDigits(4)
        self.display_error = QLCDNumber()
        self.display_error.setNumDigits(4)

        grid2 = QGridLayout()
        # self.grid2.addWidget(QLabel('tracking range min (um)'),0,0)
        # self.grid2.addWidget(self.entry_tracking_range_min_um,0,1)
        self.hslider_phase_delay = QSlider(Qt.Horizontal)
        self.hslider_phase_delay.setRange(0,90)
        self.hslider_phase_delay.setValue(0)
        self.entry_phase_delay=QDoubleSpinBox()
        self.entry_phase_delay.setRange(0,90)
        self.entry_phase_delay.setSingleStep(0.1)
        self.entry_phase_delay.setValue(0)
        hbox_phase_delay = QHBoxLayout()
        hbox_phase_delay.addWidget(QLabel('Phase Delay (degree)'))
        hbox_phase_delay.addWidget(self.hslider_phase_delay)
        hbox_phase_delay.addWidget(self.entry_phase_delay)
        self.hslider_phase_delay.valueChanged.connect(self.entry_phase_delay.setValue)
        self.entry_phase_delay.valueChanged.connect(self.volumetricImagingController.set_phase_delay)
        grid2.addLayout(hbox_phase_delay,1,0,1,4)

        grid3 = QGridLayout()
        # self.grid2.addWidget(QLabel('tracking range min (um)'),0,0)
        # self.grid2.addWidget(self.entry_tracking_range_min_um,0,1)
        self.hslider_current_DC = QSlider(Qt.Horizontal)
        self.hslider_current_DC.setRange(-250,250)
        self.hslider_current_DC.setValue(0)
        self.entry_current_DC=QDoubleSpinBox()
        self.entry_current_DC.setRange(-250,250)
        self.entry_current_DC.setSingleStep(0.1)
        self.entry_current_DC.setValue(0)
        hbox_current_DC = QHBoxLayout()
        hbox_current_DC.addWidget(QLabel('DC Mode Current (mA)'))
        hbox_current_DC.addWidget(self.hslider_current_DC)
        hbox_current_DC.addWidget(self.entry_current_DC)
        self.hslider_current_DC.valueChanged.connect(self.entry_current_DC.setValue)
        self.entry_current_DC.valueChanged.connect(self.volumetricImagingController.set_liquid_lens_current)
        grid3.addLayout(hbox_current_DC,1,0,1,4)

        vbox = QVBoxLayout()
        vbox.addLayout(grid1)
        vbox.addStretch()
        vbox.addLayout(grid2)
        vbox.addLayout(grid3)
        self.setLayout(vbox)

    def update_allowed_number_of_planes_per_volume(self,value):
        self.entry_number_of_planes_per_volume.setMaximum(value)

    def check_current_max(self,current_min):
        if current_min > self.entry_current_max.value():
            self.hslider_current_max.setValue(current_min)

    def check_current_min(self,current_max):
        if current_max < self.entry_current_min.value():
            self.hslider_current_min.setValue(current_max)

    def toggle_volumetric_imaging(self,pressed):
        if pressed == True:
            if self.checkbox_save_images.isChecked():
                self.volumetricImagingController.set_flag_record_acquisition(True)
                self.btn_toggle_recording.setChecked(True)
                self.btn_toggle_recording.setText('Stop Recording')
            self.btn_toggle_volumetric_imaging.setText('Stop Volumetric Imaging')
            self.volumetricImagingController.start_volumetric_imaging()
            self.disable_volumetric_imaging_settings()
        else:
            self.volumetricImagingController.stop_volumetric_imaging()
            if self.checkbox_save_images.isChecked():
                self.volumetricImagingController.set_flag_record_acquisition(False)
                self.checkbox_save_images.setChecked(False)
                self.btn_toggle_recording.setChecked(False)
                self.btn_toggle_recording.setText('Start Recording')
            self.btn_toggle_volumetric_imaging.setText('Start Volumetric Imaging')
            self.enable_volumetric_imaging_settings()

    def toggle_recording(self,pressed):
        if pressed == True:
            self.volumetricImagingController.start_recording()
            self.checkbox_save_images.setChecked(True)
            self.btn_toggle_recording.setText('Stop Recording')
        else:
            self.volumetricImagingController.stop_recording()
            self.checkbox_save_images.setChecked(False)
            self.btn_toggle_recording.setText('Start Recording')

    def slot_volumetric_imaging_stopped(self):
        if self.checkbox_save_images.isChecked():
            self.checkbox_save_images.setChecked(False)
            self.btn_toggle_recording.setText('Start Recording')
            self.btn_toggle_recording.setChecked(False)
        if self.btn_toggle_volumetric_imaging.isChecked():
            self.btn_toggle_volumetric_imaging.setText('Start Volumetric Imaging')
            self.btn_toggle_volumetric_imaging.setChecked(False)
            self.enable_volumetric_imaging_settings()

    def enable_caculation(self,pressed):
    	if pressed:
    		self.PDAFController.enable_caculation(True)
    	else:
    		if self.btn_enable_tracking.isChecked():
    			self.btn_enable_calculation.setChecked(True)
    		else:
    			self.PDAFController.enable_caculation(False)

    def enable_tracking(self,pressed):
    	if pressed:
    		if self.btn_enable_calculation.isChecked() == False:
    			self.btn_enable_calculation.setChecked(True)
    			self.PDAFController.enable_caculation(True)
    		self.PDAFController.enable_tracking(True)
    	else:
    		self.PDAFController.enable_tracking(False)

    def disable_volumetric_imaging_settings(self):
        self.hslider_current_min.setEnabled(False)
        self.entry_current_min.setEnabled(False)
        self.hslider_current_max.setEnabled(False)
        self.entry_current_max.setEnabled(False)
        self.dropdown_frequency.setEnabled(False)
        self.entry_number_of_planes_per_volume.setEnabled(False)
        self.entry_number_of_requested_volumes.setEnabled(False)
        self.checkbox_save_images.setEnabled(False)
        self.btn_toggle_focus_measure_calculation.setEnabled(True)
        self.btn_toggle_focus_tracking.setEnabled(True)
        self.entry_focus_offset.setEnabled(True)
        # self.hslider_phase_delay
        # self.entry_phase_delay
        self.hslider_current_DC.setEnabled(False)
        self.entry_current_DC.setEnabled(False)

    def enable_volumetric_imaging_settings(self):
        self.hslider_current_min.setEnabled(True)
        self.entry_current_min.setEnabled(True)
        self.hslider_current_max.setEnabled(True)
        self.entry_current_max.setEnabled(True)
        self.dropdown_frequency.setEnabled(True)
        self.entry_number_of_planes_per_volume.setEnabled(True)
        self.entry_number_of_requested_volumes.setEnabled(True)
        self.checkbox_save_images.setEnabled(True)
        self.btn_toggle_focus_measure_calculation.setEnabled(False)
        self.btn_toggle_focus_tracking.setEnabled(False)
        self.entry_focus_offset.setEnabled(False)
        # self.hslider_phase_delay
        # self.entry_phase_delay
        self.hslider_current_DC.setEnabled(True)
        self.entry_current_DC.setEnabled(True)


class PlotWidget(pg.GraphicsLayoutWidget):
    def __init__(self, window_title='',parent=None):
        super().__init__(parent)
        self.plotWidget = self.addPlot(title = 'Focus Measure')
    def plot(self,x,y):
        self.plotWidget.plot(x,y,clear=True)

class FocusMeasureDisplayWindow(QMainWindow):
    def __init__(self, window_title=''):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.centralWidget = QWidget()        
        self.plotWidget = PlotWidget()
        layout = QGridLayout()
        layout.addWidget(self.plotWidget, 0, 0) 
        self.centralWidget.setLayout(layout)
        self.setCentralWidget(self.centralWidget)
