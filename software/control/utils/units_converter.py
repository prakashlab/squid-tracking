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

        self.StepsPerRev_X = Motion.STEPS_PER_REV_X
        self.mmPerRev_X = Motion.MM_PER_REV_X            # Pitch of the lead screw in mm

        # --------------------------------------------------
        #  Y Stepper (Linear Stage)
        # --------------------------------------------------
        self.StepsPerRev_Y = Motion.STEPS_PER_REV_Y
        self.mmPerRev_Y = Motion.STEPS_PER_REV_Y 
        # StepsPerRev_Y = 20
        # mmPerStep_Y = 0.001524;     # Pitch of the lead screw in mm

        # --------------------------------------------------
        #  Z Stepper (Linear Stage)
        # --------------------------------------------------

        self.StepsPerRev_Z = Motion.STEPS_PER_REV_Z


        # --------------------------------------------------
        # Z stepper (Rotation stage) (vertical motion compensation)
        # --------------------------------------------------
        # Rcenter = 87.5 										# Radius to the center line of the fluidic chamber in mm (Wheel 16, 17): Ri=80 mm, Ro=95 mm
        self.Rcenter = Chamber.R_CENTER                                  # radius to the center-line of the fluidic chamber in mm (Wheel 18). Ri=80 mm R0= 110 mm
        self.StepsPerRev_Theta = Motion.STEPS_PER_REV_THETA_SHAFT            # No:of steps of the main motor shaft for 1 Rev of the output shaft

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

    #---------------------------------------------------
    # Transforming mm to stepper motor steps.
    #---------------------------------------------------
    def mmPerRev_Z(self, Xpos_mm):                            #Xpos_mm position in the centerlign of the fluid channel's referenciel
        return 2*np.pi*(self.Rcenter+Xpos_mm)

    def Z_mm_to_step(self, Zmm, Xpos_mm):
        Zstep = (Zmm/self.mmPerRev_Z(Xpos_mm))*self.StepsPerRev_Theta
        return Zstep
    #---------------------------------------------------
    def rad_to_mm(self, ThetaWheel, Xobj):
        return ThetaWheel*(self.Rcenter + Xobj)          # 2018_09_01: by Deepak. Note major Error previously the radian value was divided by 2*pi which makes the calculation of distance incorrect. 
        #ThetaWheel is already in radians so there should be no dividing 2*pi factor. Have checked by manually rotating the wheel that this now corresponds to the actual physical distance. To completely confirm will run calibration experiments again.