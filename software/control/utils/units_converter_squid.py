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
        # --------------------------------------------------
        # X encoder (linear)
        # --------------------------------------------------
        self.CountPermm_X = Encoders.COUNTS_PER_MM_X # 1um per count RLS miniature linear encoder
        # --------------------------------------------------
        # Y encoder (linear)
        # --------------------------------------------------
        self.CountPermm_Y = Encoders.COUNTS_PER_MM_Y # 1um per count RLS miniature linear encoder

    # --------------------------------------------------
    # Functions
    # --------------------------------------------------
    def set_calib_imWidth(self, imW):
        self.calib_img_width = imW

    def update_pixel_size(self, new_pixelPermm):
        self.pixelPermm = new_pixelPermm
        print('Updated pixelPermm: {}'.format(self.pixelPermm))

    def px_to_mm(self, Dist,resolution_width):
        return Dist*(1/self.pixelPermm)*(self.calib_img_width/resolution_width)

    def mm_to_px(self, Dist,resolution_width):
        return Dist*self.pixelPermm*resolution_width/self.calib_img_width

    