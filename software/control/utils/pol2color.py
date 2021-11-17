# pip3 install git+https://github.com/elerac/polanalyser
# pip3 install numba

import cv2
import numpy as np
import polanalyser as pa

def pol2color(image):
    
	img_demosaiced = pa.demosaicing(image)

	# Calculate the Stokes vector per-pixel
	angles = np.deg2rad([0, 45, 90, 135])
	img_stokes = pa.calcStokes(img_demosaiced, angles)

	# Decompose the Stokes vector into its components
	img_S0, img_S1, img_S2 = cv2.split(img_stokes)

	# Convert the Stokes vector to Intensity, DoLP and AoLP
	img_intensity = pa.cvtStokesToIntensity(img_stokes)
	img_DoLP      = pa.cvtStokesToDoLP(img_stokes)
	img_AoLP      = pa.cvtStokesToAoLP(img_stokes)

	# https://github.com/elerac/polanalyser/wiki/How-to-Visualizing-the-AoLP-Image
	# img_AoLP_cmapped = pa.applyColorToAoLP(img_AoLP, value=img_DoLP)
	img_AoLP_cmapped = pa.applyColorToAoLP(img_AoLP, saturation=img_DoLP)
	
	return img_AoLP_cmapped