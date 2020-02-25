#####################################################################################################################
'''
TESTS
'''
#####################################################################################################################

import os
import control.core as core
import control.core_tracking as core_tracking
from control.microcontroller_tracking import Microcontroller_Simulation
import cv2

def SimulateImageStream():

    # Call instances of the StreamHandler and Tracking_Controller objects

    # Load objects
    internal_state = core_tracking.InternalState()

    microcontroller = Microcontroller_Simulation()
        
    streamHandler = core.StreamHandler(trackingStream = True)

    trackingController = core_tracking.TrackingController(microcontroller = microcontroller, 
        internal_state = internal_state)

    microcontroller_sender = core_tracking.microcontroller_Sender(microcontroller, internal_state)

    microcontroller_receiver = core_tracking.microcontroller_Receiver(microcontroller, internal_state)


    # Make connections between objects
    streamHandler.packet_image_for_tracking.connect(trackingController.on_new_frame)

    streamHandler.signal_new_frame_received.connect(microcontroller_receiver.getData_microcontroller)

    trackingController.multiplex_send_signal.connect(microcontroller_sender.multiplex_Send)


    # Tested and Tracking Works
    # path = '/Users/deepak/Dropbox/GravityMachine/ExperimentResults/TestData/seacucumber9_PIV'

    path = '/Users/deepak/Dropbox/GravityMachine/ExperimentResults/TestData/seacucmber4_auto_verylong_goodtrack/images'

    FileList = os.listdir(path)


    for file in FileList:

        image = cv2.imread(os.path.join(path, file),0)

        cv2.imshow('frame', image)
        # cv2.waitKey(1)



        # Pass the image to streamHandler
        streamHandler.on_new_frame_from_simulation(image = image)


if __name__ == '__main__':

    SimulateImageStream()