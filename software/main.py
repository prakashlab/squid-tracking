# set QT_API environment variable
import os 
import sys
import argparse
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

# Qt style sheets
from aqua.qsshelper import QSSHelper

# app specific libraries
#import control.gui as gui
#import control.gui_2cameras_async as gui
import control.gui_squid_tracking as gui

parser = argparse.ArgumentParser()
parser.add_argument("--simulation", help="Run the GUI with simulated image streams.", action = 'store_true')
args = parser.parse_args()

if __name__ == "__main__":

	app = QApplication([])
	
	# Main GUI window
	if(args.simulation):
		win = gui.SquidTracking_GUI(simulation = True)
	else:
		win = gui.SquidTracking_GUI()

	# Style sheet
	qss = QSSHelper.open_qss(os.path.join('aqua', 'aqua.qss'))
	win.setStyleSheet(qss)

	win.show()

	app.exec_() #

	sys.exit()

