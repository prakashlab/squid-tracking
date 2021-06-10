import os

# TRACKING_CONFIG = 'XYZ'
TRACKING_CONFIG = 'XYT'

class TriggerMode:
    SOFTWARE = 'Software Trigger'
    HARDWARE = 'Hardware Trigger'
    CONTINUOUS = 'Continuous Acqusition'
    def __init__(self):
        pass

class MicroscopeMode:
    BFDF = 'BF/DF'
    FLUORESCENCE = 'Fluorescence'
    FLUORESCENCE_PREVIEW = 'Fluorescence Preview'
    def __init__(self):
        pass

class WaitTime:
    BASE = 0.1
    X = 0.4     # per mm
    Y = 0.4	 # per mm
    Z = 0.2     # per mm
    def __init__(self):
        pass

class AF:
    STOP_THRESHOLD = 0.85
    CROP_WIDTH = 500
    CROP_HEIGHT = 500
    def __init__(self):
        pass

class Chamber:
    # Chamber dimensions in mm
    WIDTH = 5
    R_I = 85
    R_O = 110
    LENGTH = (R_O - R_I)
    R_HOME = 93.10          # Measured for Gravity Machine (Rachel). 

    def __init__(self):
        pass

class Motion:
    # squid
    STEPS_PER_MM_XY = 1600 # microsteps
    STEPS_PER_MM_Z = 5333  # microsteps
   
    # Gravity Machine
    STEPS_PER_REV_X = 200
    MM_PER_REV_X = 1
    STEPS_PER_MM_X = round(STEPS_PER_REV_X/MM_PER_REV_X)



    STEPS_PER_REV_Y = 200
    MM_PER_REV_Y = 1
    STEPS_PER_MM_Y = round(STEPS_PER_REV_Y/MM_PER_REV_Y)

    STEPS_PER_REV_Z = 200
    MM_PER_REV_Z = 1
    STEPS_PER_MM_Z = round(STEPS_PER_REV_Z/MM_PER_REV_Z)

    STEPS_PER_REV_THETA_MOTOR = 200

    GEAR_RATIO_THETA = 99+1044/float(2057) 
    
    STEPS_PER_REV_THETA_SHAFT = round(GEAR_RATIO_THETA*STEPS_PER_REV_THETA_MOTOR)

    MAX_MICROSTEPS = 16

    def __init__(self):
        pass

class Encoders:

    COUNTS_PER_MM_X = 500 # 1um per count RLS miniature linear encoder
    COUNTS_PER_MM_Y = 500

    COUNTS_PER_REV_THETA_MOTOR = 600

    COUNTS_PER_REV_THETA = COUNTS_PER_REV_THETA_MOTOR*Motion.GEAR_RATIO_THETA


    def __init__(self):
        pass





class Acquisition:
    CROP_WIDTH = 3000
    CROP_HEIGHT = 3000
    NUMBER_OF_FOVS_PER_AF = 3
    IMAGE_FORMAT = 'png'
    IMAGE_DISPLAY_SCALING_FACTOR = 0.25
    DX = 1
    DY = 1
    DZ = 3

    def __init__(self):
        pass

class Tracking:
    SEARCH_AREA_RATIO = 10
    
    CROPPED_IMG_RATIO = 10

    BBOX_SCALE_FACTOR = 1.2

    DEFAULT_TRACKER = "csrt"

    INIT_METHODS = ["threshold", "roi"]
    
    DEFAULT_INIT_METHOD = "threshold"

    
    def __init__(self):
        pass


class MicrocontrollerDef:
     # Time interval for reading micro Controller (ms)
    UCONTROLLER_READ_INTERVAL = 25 
    MSG_LENGTH = 12
    CMD_LENGTH = 4
    N_BYTES_POS = 3
    RUN_OPENLOOP = False # Determines whether stepper/encoders are used to calculate stage positions.

    def __init__(self):
        pass

class PID_parameters:

    MAX_DISTANCE = 2 # Max distance (in mm) for truncating PID command

    STEP_PER_MM_TYPICAL = 200

    PID_OUTPUT_MAX = MAX_DISTANCE*STEP_PER_MM_TYPICAL*Motion.MAX_MICROSTEPS



# class FocusTracking:

#     # in Hz
#     LIQLENS_FREQ_MIN = 0.1
#     LIQLENS_FREQ_MAX = 20
#     LIQLENS_FREQ_STEP = 0.1
#     LIQLENS_FREQ_DEFAULT = 2

#     # in mm
#     LIQLENS_AMP_MIN = 0.01
#     LIQLENS_AMP_MAX = 0.5
#     LIQLENS_AMP_STEP = 0.01
#     LIQLENS_AMP_DEFAULT = 0.05


#     def __init__(self):
#         pass

# Default saving location
DEFAULT_SAVE_FOLDER = os.path.join(os.environ['HOME'], 'GravityMachine')

if(not os.path.exists(DEFAULT_SAVE_FOLDER)):
    os.makedirs(DEFAULT_SAVE_FOLDER)



# Width of Image used for Pixel Size Calibration. 
CALIB_IMG_WIDTH = 1920

WORKING_RES_DEFAULT = 0.5

TRACKERS = ['nearest-nbr', 'csrt', 'kcf', 'mil', 'tld', 'medianflow','mosse','daSiamRPN']
DEFAULT_TRACKER = 'nearest-nbr'



CROPPED_IMG_RATIO = 10

FocusTracking = {'Cropped image ratio':{'default':10}}

# Gravity machine (Image calibration 30Dec2020)
OBJECTIVES = {'2x':{'magnification':2, 'NA':0.10, 'PixelPermm':217}, '4x':{'magnification':4, 'NA':0.13, 'PixelPermm':432}, '10x':{'magnification':10, 'NA':0.25, 'PixelPermm':1066}, '20x':{'magnification':20, 'NA':0.4, 'PixelPermm':4008}, '40x':{'magnification':40, 'NA':0.6,'PixelPermm':8016}}

DEFAULT_OBJECTIVE = '4x'
  

CAMERAS = {'DF1':{'make':'TIS','serial':"08910100", 'px_format':(2048, 1536), 'color_format': 'GRAY8', 'fps': 75, 'is_color': False}, 'DF2':{'make':'TIS','serial':"08910102", 'px_format':(2048, 1536), 'color_format': 'GRAY8', 'fps': 75, 'is_color': False}}

# CAMERAS = {'DF1':{'make':'Daheng','serial':"FU0200070043", 'px_format':None, 'color_format': 'GRAY8', 'fps': 30}}




TRACKING = 'DF1'

FPS = {'display':{'min':1, 'max':30, 'default':15}, 
        'trigger_hardware':{'min':1, 'max':CAMERAS[TRACKING]['fps'], 
            'default':50}, 
        'trigger_software':{'min':1, 'max':120, 'default':15}, 
        'save':{'min':1, 'max':100, 'default':10}}


liquidLens = {'type': 'optotune', 'Freq':{'default':2, 'min':0.1, 'max':20, 'step':0.1, 'units':'Hz'}, 
    'Amp':{'default':0.05, 'min':0, 'max':0.5, 'step':0.01, 'units':'mm'}, 'currentScaleFactor':1/(0.0003) }


if TRACKING_CONFIG == 'XYT':
    INTERNAL_STATE_VARIABLES = ['Time', 'X_objStage', 'Y_objStage', 'Z_objStage', 'X_stage', 'Y_stage',
        'Theta_stage', 'X_image', 'Z_image', 'track_obj_image','track_obj_image_hrdware', 'track_focus', 'track_obj_stage', 
        'Acquisition', 'homing_status',  'Zero_stage', 'liquidLens_Freq', 'liquidLens_Amp', 'FocusPhase', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID']

    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X_objStage', 'Y_objStage', 'Z_objStage', 'Theta_stage', 'X_image', 
        'Z_image', 'track_focus', 'track_obj_stage','liquidLens_Freq', 'liquidLens_Amp', 'FocusPhase']

    MOTION_COMMANDS = ['X_order', 'Y_order', 'Theta_order']

    SEND_DATA = ['liquidLens_Freq', 'track_focus', 'track_obj_image' , 'X_order', 'Y_order', 'Theta_order', 'Zero_stage']


    REC_DATA = ['FocusPhase', 'X_stage', 'Y_stage', 'Theta_stage', 'track_obj_image_hrdware', 'track_obj_stage', 'homing_status']


    INITIAL_VALUES = {'Time':0, 'X_objStage':0, 'Y_objStage':0, 'Z_objStage':0, 'X_stage':0.0, 'Y_stage':0.0,
        'Theta_stage':0.0, 'X_image':0, 'Z_image':0, 'track_obj_image':False, 'track_obj_image_hrdware':False, 'track_focus':False, 
        'track_obj_stage':False, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'liquidLens_Freq': liquidLens['Freq']['default'], 
        'liquidLens_Amp': liquidLens['Amp']['default'] , 'FocusPhase':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track'}

    PLOT_VARIABLES = {'X':'X_objStage','Y':'Y_objStage', 'Z':'Z_objStage', 'Theta':'Theta_stage', 'Phase':'FocusPhase'}

    PLOT_COLORS = {'X':'r','Y':'g', 'Z':'b', 'Theta':'c', 'Phase':'w'}

    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm', 'Theta':'radians','Phase':'radians'}
    assert list(PLOT_VARIABLES.keys()) == list(PLOT_COLORS.keys())
    assert list(PLOT_VARIABLES.keys()) == list(PLOT_UNITS.keys())

    DEFAULT_PLOTS = ['X', 'Z']

elif TRACKING_CONFIG == 'XYZ':
    INTERNAL_STATE_VARIABLES = ['Time', 'X_objStage', 'Y_objStage', 'Z_objStage', 'X_stage', 'Y_stage',
        'Z_stage', 'X_image', 'Y_image', 'track_obj_image','track_obj_image_hrdware', 'track_focus', 'track_obj_stage', 
        'Acquisition', 'homing_status',  'Zero_stage', 'liquidLens_Freq', 'liquidLens_Amp', 'FocusPhase', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID']

    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X_stage', 'Y_stage', 'Z_stage', 'X_image', 
        'Y_image', 'track_focus', 'track_obj_stage','liquidLens_Freq', 'liquidLens_Amp', 'FocusPhase']

    MOTION_COMMANDS = ['X_order', 'Y_order', 'Z_order']

    SEND_DATA = ['liquidLens_Freq', 'track_focus' , 'track_obj_image' , 'X_order', 'Y_order', 'Z_order', 'Zero_stage']

    REC_DATA = ['X_stage', 'Y_stage', 'Z_stage']


    INITIAL_VALUES = {'Time':0, 'X_objStage':0, 'Y_objStage':0, 'Z_objStage':0, 'X_stage':0, 'Y_stage':0,
        'Z_stage':0, 'X_image':0, 'Y_image':0, 'track_obj_image':False, 'track_obj_image_hrdware':False, 'track_focus':False, 
        'track_obj_stage':False, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'liquidLens_Freq': liquidLens['Freq']['default'], 
        'liquidLens_Amp': liquidLens['Amp']['default'] , 'FocusPhase':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track'}

    PLOT_VARIABLES = {'X':'X_objStage','Y':'Y_objStage', 'Z':'Z_objStage', 'Phase':'FocusPhase'}

    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm','Phase':'radians'}

    DEFAULT_PLOTS = ['X', 'Y']

assert INTERNAL_STATE_VARIABLES == list(INITIAL_VALUES.keys()), "Variable mismatch: One or more state variables may not be initialized"

