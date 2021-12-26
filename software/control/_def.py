import os
import glob
import numpy as np

TRACKING_CONFIG = 'XY_Z'
# TRACKING_CONFIG = 'XZ_Y'
# TRACKING_CONFIG = 'XTheta_Y'

USE_HARDWARE_SWITCH = False # for switching on/off stage tracking

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
    DEFAULT_INIT_METHOD = "roi"

class MicrocontrollerDef:
    MSG_LENGTH = 24
    CMD_LENGTH = 8
    N_BYTES_POS = 4

class CMD_SET:
    MOVE_X = 0
    MOVE_Y = 1
    MOVE_Z = 2
    MOVE_THETA = 3
    HOME_OR_ZERO = 5
    TURN_ON_ILLUMINATION = 10
    TURN_OFF_ILLUMINATION = 11
    SET_ILLUMINATION = 12
    SET_ILLUMINATION_LED_MATRIX = 13
    ACK_JOYSTICK_BUTTON_PRESSED = 14
    ANALOG_WRITE_ONBOARD_DAC = 15
    MOVETO_X = 6
    MOVETO_Y = 7
    MOVETO_Z = 8
    SET_LIM = 9
    SET_LIM_SWITCH_POLARITY = 20
    CONFIGURE_STEPPER_DRIVER = 21
    SET_MAX_VELOCITY_ACCELERATION = 22
    SET_LEAD_SCREW_PITCH = 23

BIT_POS_JOYSTICK_BUTTON = 0
BIT_POS_SWITCH = 1


class HOME_OR_ZERO:
    HOME_NEGATIVE = 1 # motor moves along the negative direction (MCU coordinates)
    HOME_POSITIVE = 0 # motor moves along the negative direction (MCU coordinates)
    ZERO = 2

class AXIS:
    X = 0 # in plane axis 0
    Y = 1 # in plane axis 1 (actual y or actual z or actual theta - depending on the configuration)
    Z = 2 # focus axis (actual z or actual y, depending on the configuration)
    THETA = 3 # not used
    XY = 4

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

class LIMIT_CODE:
    X_POSITIVE = 0
    X_NEGATIVE = 1
    Y_POSITIVE = 2
    Y_NEGATIVE = 3
    Z_POSITIVE = 4
    Z_NEGATIVE = 5

class LIMIT_SWITCH_POLARITY:
    ACTIVE_LOW = 0
    ACTIVE_HIGH = 1
    DISABLED = 2

class ILLUMINATION_CODE:
    ILLUMINATION_SOURCE_LED_ARRAY_FULL = 0;
    ILLUMINATION_SOURCE_LED_ARRAY_LEFT_HALF = 1
    ILLUMINATION_SOURCE_LED_ARRAY_RIGHT_HALF = 2
    ILLUMINATION_SOURCE_LED_ARRAY_LEFTB_RIGHTR = 3
    ILLUMINATION_SOURCE_LED_EXTERNAL_FET = 20
    ILLUMINATION_SOURCE_405NM = 11
    ILLUMINATION_SOURCE_488NM = 12
    ILLUMINATION_SOURCE_638NM = 13
    ILLUMINATION_SOURCE_561NM = 14

LED_MATRIX_PATTERN = {'LED matrix full': ILLUMINATION_CODE.ILLUMINATION_SOURCE_LED_ARRAY_FULL,
                      'LED matrix left half': ILLUMINATION_CODE.ILLUMINATION_SOURCE_LED_ARRAY_LEFT_HALF, 
                      'LED matrix right half': ILLUMINATION_CODE.ILLUMINATION_SOURCE_LED_ARRAY_RIGHT_HALF}

# Default saving location
DEFAULT_SAVE_FOLDER = os.path.join(os.environ['HOME'], 'GravityMachine')
if(not os.path.exists(DEFAULT_SAVE_FOLDER)):
    os.makedirs(DEFAULT_SAVE_FOLDER)

WORKING_RES_DEFAULT = 0.5
TRACKERS = ['nearest-nbr', 'csrt', 'kcf', 'mil', 'tld', 'medianflow','mosse','daSiamRPN']
DEFAULT_TRACKER = 'csrt'
DEFAULT_INIT_METHOD = 'roi'
CROPPED_IMG_RATIO = 10

CAMERA_PIXEL_SIZE_UM = {'IMX290':2.9,'IMX178':2.4,'IMX226':1.85,'IMX250':3.45,'IMX252':3.45,'IMX273':3.45,'IMX264':3.45,'IMX265':3.45,'IMX571':3.76,'PYTHON300':4.8}
OBJECTIVES = {'2x':{'magnification':2, 'NA':0.10, 'tube_lens_f_mm':180}, 
                '4x':{'magnification':4, 'NA':0.13, 'tube_lens_f_mm':180}, 
                '10x':{'magnification':10, 'NA':0.25, 'tube_lens_f_mm':180}, 
                '20x (Boli)':{'magnification':20, 'NA':0.4, 'tube_lens_f_mm':180}, 
                '20x (Nikon)':{'magnification':20, 'NA':0.45, 'tube_lens_f_mm':200}, 
                '40x':{'magnification':40, 'NA':0.6, 'tube_lens_f_mm':180}}

TRACKING = 'DF1'


# MCU Command Set
SET_NUMBER_OF_PLANES_PER_VOLUME = 10
SET_NUMBER_OF_REQUESTED_VOLUMES = 11
SET_FREQUENCY_HZ = 12
SET_PHASE_DELAY = 13
START_TRIGGER_GENERATION = 14
STOP_TRIGGER_GENERATION = 15

# Volumetric imaging
VOLUMETRIC_IMAGING_NUMBER_OF_PLANES_PER_VOLUME_DEFAULT = 20

class CMD_EXECUTION_STATUS:
    COMPLETED_WITHOUT_ERRORS = 0
    IN_PROGRESS = 1
    CMD_CHECKSUM_ERROR = 2
    CMD_INVALID = 3
    CMD_EXECUTION_ERROR = 4
    ERROR_CODE_EMPTYING_THE_FLUDIIC_LINE_FAILED = 100

SLEEP_TIME_S = 0.005

LED_MATRIX_R_FACTOR = 1
LED_MATRIX_G_FACTOR = 1
LED_MATRIX_B_FACTOR = 1

##################################################
#### Default Configurations - to be overriden ####
##################################################
# note that these no longer correspond to the actual axes (starting 12/22/2021)
# note that here X corresponds to the in-plane axis 1, Y corresponds to the in-plane axis 2, Z corresponds to the focus axis

STAGE_MOVEMENT_SIGN_X = -1
STAGE_MOVEMENT_SIGN_Y = 1
STAGE_MOVEMENT_SIGN_Z = 1
STAGE_MOVEMENT_SIGN_THETA = 1

STAGE_POS_SIGN_X = STAGE_MOVEMENT_SIGN_X
STAGE_POS_SIGN_Y = STAGE_MOVEMENT_SIGN_Y
STAGE_POS_SIGN_Z = STAGE_MOVEMENT_SIGN_Z
STAGE_POS_SIGN_THETA = STAGE_MOVEMENT_SIGN_THETA

# note that these correspond to the actual axes
TRACKING_MOVEMENT_SIGN_X = 1
TRACKING_MOVEMENT_SIGN_Y = 1
TRACKING_MOVEMENT_SIGN_Z = 1

# note that these correspond to the actual axes
USE_ENCODER_X = False
USE_ENCODER_Y = False
USE_ENCODER_Z = False
USE_ENCODER_THETA = False

# note that these correspond to the actual axes
ENCODER_SIGN_X = 1
ENCODER_SIGN_Y = 1
ENCODER_SIGN_Z = 1
ENCODER_SIGN_THETA = 1

ENCODER_STEP_SIZE_X_MM = 100e-6
ENCODER_STEP_SIZE_Y_MM = 100e-6
ENCODER_STEP_SIZE_Z_MM = 100e-6
ENCODER_STEP_SIZE_THETA = 2*np.pi/(300*4)

FULLSTEPS_PER_REV_X = 200
FULLSTEPS_PER_REV_Y = 200
FULLSTEPS_PER_REV_Z = 200
FULLSTEPS_PER_REV_THETA = 200

# beginning of actuator specific configurations
# note that here X corresponds to the in-plane axis 1, Y corresponds to the in-plane axis 2, Z corresponds to the focus axis
SCREW_PITCH_X_MM = 1
SCREW_PITCH_Y_MM = 1
SCREW_PITCH_Z_MM = 0.012*25.4
GEAR_RATIO_THETA = 99+(1044.0/2057.0)

MICROSTEPPING_DEFAULT_X = 8
MICROSTEPPING_DEFAULT_Y = 8
MICROSTEPPING_DEFAULT_Z = 8
MICROSTEPPING_DEFAULT_THETA = 8 # not used, to be removed

X_MOTOR_RMS_CURRENT_mA = 490
Y_MOTOR_RMS_CURRENT_mA = 490
Z_MOTOR_RMS_CURRENT_mA = 490

X_MOTOR_I_HOLD = 0.5
Y_MOTOR_I_HOLD = 0.5
Z_MOTOR_I_HOLD = 0.5

MAX_VELOCITY_X_mm = 20
MAX_VELOCITY_Y_mm = 20
MAX_VELOCITY_Z_mm = 2

MAX_ACCELERATION_X_mm = 500
MAX_ACCELERATION_Y_mm = 500
MAX_ACCELERATION_Z_mm = 20

# end of actuator specific configurations

SCAN_STABILIZATION_TIME_MS_X = 160
SCAN_STABILIZATION_TIME_MS_Y = 160
SCAN_STABILIZATION_TIME_MS_Z = 20

# limit switch
X_HOME_SWITCH_POLARITY = LIMIT_SWITCH_POLARITY.DISABLED
Y_HOME_SWITCH_POLARITY = LIMIT_SWITCH_POLARITY.DISABLED
Z_HOME_SWITCH_POLARITY = LIMIT_SWITCH_POLARITY.DISABLED

HOMING_ENABLED_X = False
HOMING_ENABLED_Y = False
HOMING_ENABLED_Z = False
HOMING_ENABLED_THETA = False

TWO_CAMERA_PDAF = True
PDAF_FLIPUD = False
PDAF_FLIPLR = False
PDAF_SHIFT_AXIS = 'X' #'X' or 'Y'

VOLUMETRIC_IMAGING = True
USE_SEPARATE_TRIGGER_CONTROLLER = False
TRIGGERCONTROLLER_SERIAL_NUMBER = None

CAMERAS = {'DF1':{'make':'Daheng','serial':'FW0200050063','px_format':(2560,2048),'color_format':'GRAY8','fps': 60,'is_color':False,'rotate image angle':0,'flip image':'Horizental','sensor':'IMX226'},\
           'DF2':{'make':'Daheng','serial':'FW0200050068','px_format':(2560,2048),'color_format':'GRAY8','fps': 60,'is_color':False,'rotate image angle':0,'flip image':'Horizental','sensor':'IMX226'}, \
           'volumetric imaging':{'make':'Daheng','serial':'FW0200050061','px_format':(600,600),'color_format':'GRAY8','fps':30,'is_color':False,'rotate image angle':0,'flip image':'Horizental','sensor':'PYTHON300'}}
TUBE_LENS_MM = {'DF1':50,'DF2':50,'volumetric imaging':75}
DEFAULT_OBJECTIVE = '4x'

# print('-------------------')
# print(CAMERA_PIXEL_SIZE_UM[CAMERAS['DF1']['sensor']])

##########################################################
#### start of loading machine specific configurations ####
##########################################################
config_files = glob.glob('.' + '/' + 'configuration*.txt')
if config_files:
    if len(config_files) > 1:
        print('multiple machine configuration files found, the program will exit')
        exit()
    print('load machine-specific configuration')
    exec(open(config_files[0]).read())
else:
    print('machine-specifc configuration not present, the program will exit')
    exit()
##########################################################
##### end of loading machine specific configurations #####
##########################################################

FPS = {'display':{'min':1, 'max':30, 'default':15}, 
        'trigger_hardware':{'min':1, 'max':CAMERAS[TRACKING]['fps'], 
            'default':50}, 
        'trigger_software':{'min':1, 'max':120, 'default':15}, 
        'save':{'min':1, 'max':100, 'default':10}}

if TRACKING_CONFIG == 'XTheta_Y':
    INTERNAL_STATE_VARIABLES = ['Time', 'X', 'Y', 'Z', 'X_stage', 'Y_stage',
        'Theta_stage', 'X_image', 'Z_image', 'image_tracking_enabled','enable_image_tracking_from_hardware_button', 'track_focus', 'stage_tracking_enabled', 
        'Acquisition', 'homing_status',  'Zero_stage', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID', 'experimentID_and_timestamp']
    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X', 'Y', 'Z', 'Theta_stage', 'X_image', 
        'Z_image', 'track_focus', 'stage_tracking_enabled']
    # MOTION_COMMANDS = ['X_order', 'Y_order', 'Theta_order']
    # SEND_DATA = ['liquidLens_Freq', 'track_focus', 'image_tracking_enabled' , 'X_order', 'Y_order', 'Theta_order', 'Zero_stage']
    READINGS_FROM_MCU = ['FocusPhase', 'X_stage', 'Y_stage', 'Theta_stage', 'enable_image_tracking_from_hardware_button', 'stage_tracking_enabled', 'homing_status']
    INITIAL_VALUES = {'Time':0, 'X':0, 'Y':0, 'Z':0, 'X_stage':0.0, 'Y_stage':0.0,
        'Theta_stage':0.0, 'X_image':0, 'Z_image':0, 'image_tracking_enabled':False, 'enable_image_tracking_from_hardware_button':False, 'track_focus':False, 
        'stage_tracking_enabled':True, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track', 'experimentID_and_timestamp':'track_[time]'}
    PLOT_VARIABLES = {'X':'X','Y':'Y', 'Z':'Z', 'Theta':'Theta_stage'}
    PLOT_COLORS = {'X':'r','Y':'g', 'Z':'b', 'Theta':'c'}
    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm', 'Theta':'radians'}
    assert list(PLOT_VARIABLES.keys()) == list(PLOT_COLORS.keys())
    assert list(PLOT_VARIABLES.keys()) == list(PLOT_UNITS.keys())
    DEFAULT_PLOTS = ['X', 'Z']
elif TRACKING_CONFIG == 'XY_Z':
    INTERNAL_STATE_VARIABLES = ['Time', 'X', 'Y', 'Z', 'X_stage', 'Y_stage','Z_stage', 
    'X_image', 'Y_image', 'Z_image', 'image_tracking_enabled','enable_image_tracking_from_hardware_button', 'track_focus', 'stage_tracking_enabled', 
        'Acquisition', 'homing_status',  'Zero_stage', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID', 'experimentID_and_timestamp']
    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X_stage', 'Y_stage', 'Z_stage', 'X_image', 
        'Y_image', 'track_focus', 'stage_tracking_enabled']
    # MOTION_COMMANDS = ['X_order', 'Y_order', 'Z_order']
    # SEND_DATA = ['liquidLens_Freq', 'track_focus' , 'image_tracking_enabled' , 'X_order', 'Y_order', 'Z_order', 'Zero_stage']
    READINGS_FROM_MCU = ['X_stage', 'Y_stage', 'Z_stage']
    INITIAL_VALUES = {'Time':0, 'X':0, 'Y':0, 'Z':0, 'X_stage':0, 'Y_stage':0,'Z_stage':0, 
        'X_image':0, 'Y_image':0, 'Z_image':0, 'image_tracking_enabled':False, 'enable_image_tracking_from_hardware_button':False, 'track_focus':False, 
        'stage_tracking_enabled':True, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track', 'experimentID_and_timestamp':'track_[time]'}
    PLOT_VARIABLES = {'X':'X','Y':'Y', 'Z':'Z'}
    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm'}
    PLOT_COLORS = {'X':'r','Y':'g', 'Z':'b', 'Theta':'c'}
    DEFAULT_PLOTS = ['X', 'Y']
elif TRACKING_CONFIG == 'XZ_Y':
    INTERNAL_STATE_VARIABLES = ['Time', 'X', 'Y', 'Z', 'X_stage', 'Y_stage','Z_stage', 
    'X_image', 'Y_image', 'Z_image', 'image_tracking_enabled','enable_image_tracking_from_hardware_button', 'track_focus', 'stage_tracking_enabled', 
        'Acquisition', 'homing_status',  'Zero_stage', 'optical_path', 
        'imaging channels', 'Objective', 'basePath', 'experimentID', 'experimentID_and_timestamp']
    # Based on the number of imaging channels, there will also be 1 or more image names saved.
    SAVE_DATA = ['Time', 'X_stage', 'Y_stage', 'Z_stage', 'X_image', 
        'Y_image', 'track_focus', 'stage_tracking_enabled']
    # MOTION_COMMANDS = ['X_order', 'Y_order', 'Z_order']
    # SEND_DATA = ['liquidLens_Freq', 'track_focus' , 'image_tracking_enabled' , 'X_order', 'Y_order', 'Z_order', 'Zero_stage']
    READINGS_FROM_MCU = ['X_stage', 'Y_stage', 'Z_stage']
    INITIAL_VALUES = {'Time':0, 'X':0, 'Y':0, 'Z':0, 'X_stage':0, 'Y_stage':0,'Z_stage':0, 
        'X_image':0, 'Y_image':0, 'Z_image':0, 'image_tracking_enabled':False, 'enable_image_tracking_from_hardware_button':False, 'track_focus':False, 
        'stage_tracking_enabled':True, 'Acquisition':False, 'homing_status': 'not-complete', 'Zero_stage':0, 'optical_path': None, 
        'imaging channels': list(CAMERAS.keys()),  'Objective':DEFAULT_OBJECTIVE, 'basePath':'/', 'experimentID':'track', 'experimentID_and_timestamp':'track_[time]'}
    PLOT_VARIABLES = {'X':'X','Y':'Y', 'Z':'Z'}
    PLOT_UNITS = {'X':'mm','Y':'mm', 'Z':'mm'}
    PLOT_COLORS = {'X':'r','Y':'g', 'Z':'b', 'Theta':'c'}
    DEFAULT_PLOTS = ['X', 'Z'] 

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

