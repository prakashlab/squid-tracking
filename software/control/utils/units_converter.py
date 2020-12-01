# -*- coding: utf-8 -*-

import numpy as np
from control._def import *

class Units_Converter:

    def __init__(self):

        # Pixel per mm of objective
        self.pixelPermm = OBJECTIVES[DEFAULT_OBJECTIVE]['PixelPermm']

        print('Initializing Units Converter')
        print(self.pixelPermm)

        self.calib_img_width = CALIB_IMG_WIDTH
        # Pixel per mm 4x objective
        # pixelPermm = 456
        # --------------------------------------------------
        #  X Stepper (Linear Stage)
        # --------------------------------------------------

        self.StepsPerRev_X = Motors.STEPS_PER_REV_X
        self.mmPerRev_X = Motors.MM_PER_REV_X            # Pitch of the lead screw in mm

        # --------------------------------------------------
        #  Y Stepper (Linear Stage)
        # --------------------------------------------------
        self.StepsPerRev_Y = Motors.STEPS_PER_REV_Y
        self.mmPerRev_Y = Motors.STEPS_PER_REV_Y 
        # StepsPerRev_Y = 20
        # mmPerStep_Y = 0.001524;     # Pitch of the lead screw in mm

        # --------------------------------------------------
        #  Z Stepper (Linear Stage)
        # --------------------------------------------------

        self.StepsPerRev_Z = Motors.STEPS_PER_REV_Z
        self.mmPerRev_Z = Motors.STEPS_PER_REV_Z


        # --------------------------------------------------
        # Z stepper (Rotation stage) (vertical motion compensation)
        # --------------------------------------------------
        # Rcenter = 87.5 										# Radius to the center line of the fluidic chamber in mm (Wheel 16, 17): Ri=80 mm, Ro=95 mm
        self.Rcenter = Chamber.R_CENTER                                  # radius to the center-line of the fluidic chamber in mm (Wheel 18). Ri=80 mm R0= 110 mm
        self.StepsPerRev_Theta = Motors.STEPS_PER_REV_THETA_SHAFT            # No:of steps of the main motor shaft for 1 Rev of the output shaft

        # --------------------------------------------------
        # X encoder (linear)
        # --------------------------------------------------
        self.CountPermm_X = Encoders.COUNTS_PER_MM_X # 1um per count RLS miniature linear encoder
        # --------------------------------------------------
        # Y encoder (linear)
        # --------------------------------------------------
        self.CountPermm_Y = Encoders.COUNTS_PER_MM_Y # 1um per count RLS miniature linear encoder
        # --------------------------------------------------
        # Theta encoder
        # --------------------------------------------------
        self.CountsPerRev_Theta = Encoders.COUNTS_PER_REV_THETA
        # --------------------------------------------------
        # Distance in mm between the center of the Wheel and the origin of Arduino's Xpos
        # --------------------------------------------------
        self.DeltaX_Arduino_mm = 99.325 # Measured value for GM v2.0 setup (Berg)

        # --------------------------------------------------
        # Distance in mm between the front wall(adajancent to the fluid, nearest to camera)  and the origin of Arduino's Ypos
        # --------------------------------------------------
        self.DeltaY_Arduino_mm = 0 # 10x Y offset so that the chamber wall (adjacent to the fluid) closest to the 
        # self.DeltaY_Arduino_mm = -2.18 # 4x objective
    # --------------------------------------------------
    # Functions
    # --------------------------------------------------
    def set_calib_imWidth(self, imW):
        self.calib_img_width = imW

    def update_pixel_size(self, new_pixelPermm):
        
        self.pixelPermm = new_pixelPermm

        print('new pixel size: {}'.format(self.pixelPermm))

    def px_to_mm(self, Dist,resolution_width):
        return 1/self.pixelPermm/(resolution_width/self.calib_img_width)*Dist   

    def mm_to_px(self, Dist,resolution_width):
        return Dist*self.pixelPermm*resolution_width/self.calib_img_width

    