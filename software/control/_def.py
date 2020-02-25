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

class Motion:
    STEPS_PER_MM_XY = 1600
    STEPS_PER_MM_Z = 5333
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

    DEFAULT_TRACKER = "csrt"

    
    def __init__(self):
        pass

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

RESOLUTION_WIDTH = 1920

TRACKERS = ['nearest-nbr', 'csrt', 'daSIAMRPN']
DEFAULT_TRACKER = 'csrt'

CROPPED_IMG_RATIO = 10

FocusTracking = {'Cropped image ratio':{'default':10}}

OBJECTIVES = {'type':{'10x':{'magnification':10, 'NA':0.17, 'PixelPermm':1122}, 
'20x':{'magnification':20, 'NA':0.17, 'PixelPermm':2244}}, 'default':'10x'}
  


cameras = {'DF':{'serial':[], 'px_format':(4000,3000), 'color_format': 'RGBx', 'fps': 120}, 
    'FL':{'serial':[], 'px_format':(4000,3000), 'color_format': 'RGBx', 'fps': 120}}

liquidLens = {'type': 'optotune', 'Freq':{'default':2, 'min':0.1, 'max':20, 'step':0.1, 'units':'Hz'}, 
    'Amp':{'default':0.05, 'min':0, 'max':0.5, 'step':0.01, 'units':'mm'}, 'currentScaleFactor':1/(0.0003) }



OPTICAL_PATHS = {'modes':{'DF single':['DF'], 'DF_FL':['DF', 'FL'], 'DF_BF':['DF', 'BF']}, 'default':'DF single'}



INTERNAL_STATE_VARIABLES = ['Time', 'X_objStage', 'Y_objStage', 'Z_objStage', 'X_stage', 'Y_stage',
    'Theta_stage', 'X_image', 'Z_image', 'track_obj_image', 'track_focus', 'track_obj_stage', 
    'Acquisition', 'homing_command', 'homing_state', 'liquidLens_Freq', 'liquidLens_Amp', 'FocusPhase', 'imaging channels', 'uScope mode', 
    'Objective', 'basePath', 'experimentID']

# Based on the number of imaging channels, there will also be 1 or more image names saved.
SAVE_DATA = ['Time', 'X_objStage', 'Y_objStage', 'Z_objStage', 'Theta_stage', 'X_image', 
    'Z_image', 'track_focus', 'track_obj_stage','liquidLens_Freq', 'liquidLens_Amp', 'FocusPhase']

MOTION_COMMANDS = ['X_order', 'Y_order', 'Theta_order']

SEND_DATA = ['X_order', 'Y_order', 'Theta_order', 'track_obj_image', 'track_focus', 'homing_state']

REC_DATA = ['FocusPhase', 'deltaX_stage', 'deltaY_stage', 'deltaTheta_stage', 'track_obj_image', 'track_obj_stage']

INITIAL_VALUES = {'Time':0, 'X_objStage':0, 'Y_objStage':0, 'Z_objStage':0, 'X_stage':0, 'Y_stage':0,
    'Theta_stage':0, 'X_image':0, 'Z_image':0, 'track_obj_image':False, 'track_focus':False, 
    'track_obj_stage':False, 'Acquisition':False, 'homing_command':False, 'homing_state':False, 'liquidLens_Freq': liquidLens['Freq']['default'], 
    'liquidLens_Amp': liquidLens['Amp']['default'] , 'FocusPhase':0, 'imaging channels':['DF', 'FL'], 
    'uScope mode': 'Tracking', 'Objective':'10x', 'basePath':'/', 'experimentID':'track'}

PLOT_VARIABLES = {'X':'X_objStage','Y':'Y_objStage', 'Z':'Z_objStage', 'Theta':'Theta_stage'}

PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm', 'Theta':'radians'}

DEFAULT_PLOTS = ['X', 'Z']

# 
print(INTERNAL_STATE_VARIABLES)
print(INITIAL_VALUES.keys())
assert INTERNAL_STATE_VARIABLES == list(INITIAL_VALUES.keys()), "Variable mismatch: One or more state variables may not be initialized"

