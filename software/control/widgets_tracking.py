# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

class TrackingControllerWidget(QFrame):
	'''
	Buttons to start image tracking
	Display window to show thresholded images
	Slider bars to threshold image
	Radio-buttons to choose trackers.
	Text boxes for base path and Experiment ID.

	'''
	def __init__(self, microcontroller, navigationController, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.base_path_is_set = False
		# self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)


# class PID_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass



# class FocusTracking_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass


# class PlotDisplay_Widget(QFrame):

# 	def __init__(self, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		pass
