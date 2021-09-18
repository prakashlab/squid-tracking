import os
import glob

# TRACKING_CONFIG = 'XYZ'
TRACKING_CONFIG = 'XYT'

class TriggerMode:
    SOFTWARE = 'Software Trigger'
    HARDWARE = 'Hardware Trigger'
    CONTINUOUS = 'Continuous Acqusition'

class MicroscopeMode:
    BFDF = 'BF/DF'
    FLUORESCENCE = 'Fluorescence'
    FLUORESCENCE_PREVIEW = 'Fluorescence Preview'

# this one can be redefined in machine specific files
class Chamber:
    # Chamber dimensions in mm
    WIDTH = 5
    R_I = 85
    R_O = 110
    LENGTH = (R_O - R_I)
    R_HOME = 93.10          # Measured for Gravity Machine (Rachel). 

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

class Encoders:

    COUNTS_PER_MM_X = 500 # 1um per count RLS miniature linear encoder
    COUNTS_PER_MM_Y = 500
    COUNTS_PER_REV_THETA_MOTOR = 600
    COUNTS_PER_REV_THETA = COUNTS_PER_REV_THETA_MOTOR*Motion.GEAR_RATIO_THETA

class Acquisition:
    CROP_WIDTH = 3000
    CROP_HEIGHT = 3000
    NUMBER_OF_FOVS_PER_AF = 3
    IMAGE_FORMAT = 'png'
    IMAGE_DISPLAY_SCALING_FACTOR = 0.25
    DX = 1
    DY = 1
    DZ = 3

class Tracking:
    SEARCH_AREA_RATIO = 10
    CROPPED_IMG_RATIO = 10
    BBOX_SCALE_FACTOR = 1.2
    DEFAULT_TRACKER = "csrt"
    INIT_METHODS = ["threshold", "roi"]
    DEFAULT_INIT_METHOD = "threshold"

class MicrocontrollerDef:
     # Time interval for reading micro Controller (ms)
    UCONTROLLER_READ_INTERVAL = 25 
    MSG_LENGTH = 12
    CMD_LENGTH = 4
    N_BYTES_POS = 3
    RUN_OPENLOOP = False # Determines whether stepper/encoders are used to calculate stage positions.

class PID_parameters:
    MAX_DISTANCE = 2 # Max distance (in mm) for truncating PID command
    STEP_PER_MM_TYPICAL = 200
    PID_OUTPUT_MAX = MAX_DISTANCE*STEP_PER_MM_TYPICAL*Motion.MAX_MICROSTEPS

class PDAF:
    ROI_ratio_width_default = 2.5
    ROI_ratio_height_default = 2.5
    x_offset_default = 31
    y_offset_default = -41
    shift_to_distance_um_default = -5.0


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

##################################################
#### Default Configurations - to be overriden ####
##################################################
STAGE_MOVEMENT_SIGN_X = 1
STAGE_MOVEMENT_SIGN_Y = 1
STAGE_MOVEMENT_SIGN_THETA = 1
X_ENCODER_SIGN = 1
Y_ENCODER_SIGN = 1
THETA_ENCODER_SIGN = 1
TWO_CAMERA_PDAF = True
VOLUMETRIC_IMAGING = True
USE_SEPARATE_TRIGGER_CONTROLLER = False
TRIGGERCONTROLLER_SERIAL_NUMBER = None
CAMERAS = {'DF1':{'make':'Daheng','serial':'FW0200050063','px_format':(2560,2048),'color_format':'GRAY8','fps': 60,'is_color':False,'rotate image angle':0,'flip image':'Horizental'},\
           'DF2':{'make':'Daheng','serial':'FW0200050068','px_format':(2560,2048),'color_format':'GRAY8','fps':60,'is_color':False,'rotate image angle':0,'flip image':'Horizental'}, \
           'volumetric imaging':{'make':'Daheng','serial':'FW0200050061','px_format':(600,600),'color_format':'GRAY8','fps':30,'is_color':False,'rotate image angle':0,'flip image':'Horizental'}}
OBJECTIVES = {'2x':{'magnification':2, 'NA':0.10, 'PixelPermm':217}, '4x':{'magnification':4, 'NA':0.13, 'PixelPermm':432}, '10x':{'magnification':10, 'NA':0.25, 'PixelPermm':1066}, '20x':{'magnification':20, 'NA':0.4, 'PixelPermm':4008}, '40x':{'magnification':40, 'NA':0.6,'PixelPermm':8016}}
DEFAULT_OBJECTIVE = '4x'

##########################################################
#### start of loading machine specific configurations ####
##########################################################
config_files = glob.glob('.' + '/' + 'configuration*.txt')
if config_files:
    print('load machine-specific configuration')
    exec(open(config_files[0]).read())

########################################################
#### end of loading machine specific configurations ####
########################################################

TRACKING = 'DF1'

FPS = {'display':{'min':1, 'max':30, 'default':15}, 
        'trigger_hardware':{'min':1, 'max':CAMERAS[TRACKING]['fps'], 
            'default':50}, 
        'trigger_software':{'min':1, 'max':120, 'default':15}, 
        'save':{'min':1, 'max':100, 'default':10}}

if TRACKING_CONFIG == 'XYT':
    INTERNAL_STATE_VARIABLES = ['Time', 'X', 'Y', 'Z', 'X_stage', 'Y_stage',
        'Theta_stage', 'X_image', 'Z_image', 'image_tracking_enabled','enable_image_tracking_from_hardware_button', 'track_focus', 'stage_tracking_enabled', 
        'Acquisition', 'homing_status',  'Zero_stage', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID']
    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X', 'Y', 'Z', 'Theta_stage', 'X_image', 
        'Z_image', 'track_focus', 'stage_tracking_enabled']
    # MOTION_COMMANDS = ['X_order', 'Y_order', 'Theta_order']
    # SEND_DATA = ['liquidLens_Freq', 'track_focus', 'image_tracking_enabled' , 'X_order', 'Y_order', 'Theta_order', 'Zero_stage']
    READINGS_FROM_MCU = ['FocusPhase', 'X_stage', 'Y_stage', 'Theta_stage', 'enable_image_tracking_from_hardware_button', 'stage_tracking_enabled', 'homing_status']
    INITIAL_VALUES = {'Time':0, 'X':0, 'Y':0, 'Z':0, 'X_stage':0.0, 'Y_stage':0.0,
        'Theta_stage':0.0, 'X_image':0, 'Z_image':0, 'image_tracking_enabled':False, 'enable_image_tracking_from_hardware_button':False, 'track_focus':False, 
        'stage_tracking_enabled':False, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track'}
    PLOT_VARIABLES = {'X':'X','Y':'Y', 'Z':'Z', 'Theta':'Theta_stage', 'Phase':'FocusPhase'}
    PLOT_COLORS = {'X':'r','Y':'g', 'Z':'b', 'Theta':'c', 'Phase':'w'}
    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm', 'Theta':'radians','Phase':'radians'}
    assert list(PLOT_VARIABLES.keys()) == list(PLOT_COLORS.keys())
    assert list(PLOT_VARIABLES.keys()) == list(PLOT_UNITS.keys())
    DEFAULT_PLOTS = ['X', 'Z']
elif TRACKING_CONFIG == 'XYZ':
    INTERNAL_STATE_VARIABLES = ['Time', 'X', 'Y', 'Z', 'X_stage', 'Y_stage',
        'Z_stage', 'X_image', 'Y_image', 'image_tracking_enabled','enable_image_tracking_from_hardware_button', 'track_focus', 'stage_tracking_enabled', 
        'Acquisition', 'homing_status',  'Zero_stage', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID']
    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X_stage', 'Y_stage', 'Z_stage', 'X_image', 
        'Y_image', 'track_focus', 'stage_tracking_enabled']
    # MOTION_COMMANDS = ['X_order', 'Y_order', 'Z_order']
    # SEND_DATA = ['liquidLens_Freq', 'track_focus' , 'image_tracking_enabled' , 'X_order', 'Y_order', 'Z_order', 'Zero_stage']
    READINGS_FROM_MCU = ['X_stage', 'Y_stage', 'Z_stage']
    INITIAL_VALUES = {'Time':0, 'X':0, 'Y':0, 'Z':0, 'X_stage':0, 'Y_stage':0,
        'Z_stage':0, 'X_image':0, 'Y_image':0, 'image_tracking_enabled':False, 'enable_image_tracking_from_hardware_button':False, 'track_focus':False, 
        'stage_tracking_enabled':False, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track'}
    PLOT_VARIABLES = {'X':'X','Y':'Y', 'Z':'Z', 'Phase':'FocusPhase'}
    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm','Phase':'radians'}
    DEFAULT_PLOTS = ['X', 'Y']
# changes 9/12/2021
# X_objStage -> X
# Y_objStage -> Y
# Z_objStage -> Z
# track_obj_stage -> stage_tracking_enabled
# track_obj_image -> image_tracking_enabled
# track_obj_image_hrdware -> enable_image_tracking_from_hardware_button


if TWO_CAMERA_PDAF:
    INTERNAL_STATE_VARIABLES.extend(['track_focus_PDAF','PDAF_shift','PDAF_error'])
    SAVE_DATA.extend(['track_focus_PDAF','PDAF_shift','PDAF_error'])
    INITIAL_VALUES.update({'track_focus_PDAF':False,'PDAF_shift':float('nan'),'PDAF_error':float('nan')})

if VOLUMETRIC_IMAGING:
    # to add variables to save
    pass

assert INTERNAL_STATE_VARIABLES == list(INITIAL_VALUES.keys()), "Variable mismatch: One or more state variables may not be initialized"

# MCU Command Set
SET_NUMBER_OF_PLANES_PER_VOLUME = 10
SET_NUMBER_OF_REQUESTED_VOLUMES = 11
SET_FREQUENCY_HZ = 12
SET_PHASE_DELAY = 13
START_TRIGGER_GENERATION = 14
STOP_TRIGGER_GENERATION = 15

# Volumetric imaging
VOLUMETRIC_IMAGING_NUMBER_OF_PLANES_PER_VOLUME_DEFAULT = 20

