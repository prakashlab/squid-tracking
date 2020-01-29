import utils.image_processing as image_processing

import numpy as np

from DaSiamRPN.code.net import SiamRPNvot
from DaSiamRPN.code import vot 
from DaSiamRPN.code.run_SiamRPN import SiamRPN_init, SiamRPN_track
from DaSiamRPN.code.utils import get_axis_aligned_bbox, cxy_wh_2_rect



class Tracker_Image(object):
	
	def __init__(self, OpenCVTrackers, NeuralNetTrackers):
		
		self.OpenCVTrackers = OpenCVTrackers
		self.NeuralNetTrackers = NeuralNetTrackers

		# Centroid of object from the image
		self.centroid_image = None # (2,1)
		# Centroid of object along optical axis
		self.centroid_focus = None # (1,)
		self.bbox = None

		self.origLoc = np.array([0,0])

		self.tracker_type = None
		self.tracker = None

		self.isCentroidFound = False

		self.trackerActive = False



		try:
			# load net
			self.net = SiamRPNvot()
			self.net.load_state_dict(torch.load(join(realpath(dirname(__file__)), 'DaSiamRPN','code','SiamRPNOTB.model')))
			self.net.eval().cuda()
			print('Finished loading net ...')

		except:
			print('No neural net model found ...')
			print('reverting to default OpenCV tracker')
			self.tracker_type = "csrt"




	def track(self,image, thresh_image, tracker, tracker_type, start_flag = False):

		self.tracker_type = tracker_type
		self.tracker = tracker
		
		# Try to find an object
		if(start_flag or self.trackerActive == False):
			# Find centroid based on simple image thresholding

			# Threshold the image based on the set thresholding parameters
			# Get the latest thresholded image from the relevant Queue
            # thresh_image = image_processing.threshold_image(image, lower_HSV, upper_HSV)  #The threshold image as one channel

			self.isCentroidFound, self.centroid_image, self.bbox = 
				image_processing.find_centroid_basic_Rect(thresh_image)
			
			
			self.init_tracker(image, self.centroid_image, self.bbox)

			self.trackerActive = True


		else:
			# Find centroid using the tracking.
			self.bbox = self.update_tracker(image) # (x,y,w,h)

			if(self.bbox is not None)

				self.isCentroidFound = True

				self.centroid_image = self.centroid_from_bbox(self.bbox) + self.origLoc

				self.rect_pts = self.rectpts_from_bbox(self.bbox)
			else:
				print('No object found ...')
				self.isCentroidFound = False
				self.trackerActive = False




		

		return self.isCentroidFound, self.centroid_image, self.rect_pts








	def init_tracker(self, image, centroid, bbox):

		# Initialize the OpenCV based tracker
		if(self.tracker_type in OpenCVTrackers.keys()):

			self.tracker.init(image, bbox)

		# Initialize Neural Net based Tracker
		elif(self.tracker_type in NeuralNetTrackers.keys()):
			# Initialize the tracker with this centroid position
			target_pos, target_sz = np.array([centroid[0], centroid[1]]), np.array([bbox[2], bbox[3]])

			self.state = SiamRPN_init(image_data, target_pos, target_sz, self.net)

		else:
			pass


	def update_tracker(self, image):
		# Input: image
		# Output: new_bbox based on tracking

		new_bbox = None

		if(self.tracker_type in OpenCVTrackers.keys()):
			self.origLoc = np.array([0,0])
			# (x,y,w,h)
			ok, new_bbox = self.tracker.update(image_data)

				

		elif(self.tracker_type in NeuralNetTrackers.keys()):

			self.origLoc = np.array([0,0])

			self.state = SiamRPN_track(self.state, image_data)

			ok = True

			if(ok):
				# (x,y,w,h)
				new_bbox = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])

				new_bbox = [int(l) for l in new_bbox]

		else:
			# If no tracker is specified, use basic thresholding and
			# nearest neighbhour tracking. i.e Look for objects in a search region 
			# near the last detected centroid

			# Get the latest thresholded image from the queue
			# thresh_image = 

			pts,image_data = image_processing.crop(image, self.centroid_image,self.searchArea)
			
			self.origLoc = pts[0]

			isCentroidFound, centroid, new_bbox = 
				image_processing.find_centroid_basic_Rect(image_data)


		# @@@ Can add additional methods here for future tracker implementations

		return new_bbox


	def centroid_from_bbox(self, bbox):

		# Coordinates of the object centroid are taken as the center of the bounding box
		assert(len(bbox) == 4)

		cx = int(bbox[0] + bbox[2]/2)
		cy = int(bbox[1] + bbox[3]/2)

		centroid = np.array([cx, cy])

		return centroid

	def rectpts_from_bbox(self, bbox):

		pts = np.array([[bbox[0], bbox[1]],[bbox[0] + bbox[2], bbox[1] + bbox[3]]], dtype = 'int')

		return pts



class Tracker_Focus(object):
	def __init__(self):
		pass

	def track(self,image):
		pass




