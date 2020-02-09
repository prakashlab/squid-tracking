# set QT_API environment variable
import os 
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
import control.gui_tiscamera as gui

if __name__ == "__main__":

	app = QApplication([])
	
	# Main GUI window
	win = gui.OctopiGUI()

	# Style sheet
	qss = QSSHelper.open_qss(os.path.join('aqua', 'aqua.qss'))
	win.setStyleSheet(qss)

	win.show()
	app.exec_() #sys.exit(app.exec_())
