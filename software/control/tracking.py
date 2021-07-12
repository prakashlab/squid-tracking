import control.utils.image_processing as image_processing
import numpy as np
from os.path import realpath, dirname, join


try:
	import torch
	from control.DaSiamRPN.code.net import SiamRPNvot
	print(1)
	from control.DaSiamRPN.code import vot
	print(2) 
	from control.DaSiamRPN.code.utils import get_axis_aligned_bbox, cxy_wh_2_rect
	print(3)
	from control.DaSiamRPN.code.run_SiamRPN import SiamRPN_init, SiamRPN_track
	print(4)
except Exception as e:
	print(e)
	# print('Warning: DaSiamRPN is not available!')
from control._def import Tracking
import cv2




class Tracker_Image(object):
	
	'''
	SLOTS: update_tracker_type, Connected to: Tracking Widget


	'''

	def __init__(self, color = False):
		
		 # Define list of trackers being used(maybe do this as a definition?)
		# OpenCV tracking suite
		# self.OPENCV_OBJECT_TRACKERS = {}
		try:
			self.OPENCV_OBJECT_TRACKERS = {
			"csrt": cv2.TrackerCSRT_create,
			"kcf": cv2.TrackerKCF_create,
			"boosting": cv2.TrackerBoosting_create,
			"mil": cv2.TrackerMIL_create,
			"tld": cv2.TrackerTLD_create,
			"medianflow": cv2.TrackerMedianFlow_create,
			"mosse": cv2.TrackerMOSSE_create
			}
		except:
			print('Warning: OpenCV-Contrib trackers unavailable!')
		# Neural Net based trackers
		self.NEURALNETTRACKERS = {"daSiamRPN":[]}

		# Image Tracker type
		self.tracker_type = Tracking.DEFAULT_TRACKER

		self.create_tracker()

		# Init method for tracker
		self.init_method = Tracking.DEFAULT_INIT_METHOD

		# Centroid of object from the image
		self.centroid_image = None # (2,1)
		# Centroid of object along optical axis
		self.centroid_focus = None # (1,)
		self.bbox = None
		self.rect_pts = None
		self.roi_bbox = None

		self.origLoc = np.array([0,0])

		

		self.isCentroidFound = False

		self.trackerActive = False

		self.searchArea = None

		self.color = color


		try:
			# load net
			self.net = SiamRPNvot()
			self.net.load_state_dict(torch.load(join(realpath(dirname(__file__)),'DaSiamRPN','code','SiamRPNOTB.model')))
			self.net.eval().cuda()
			print('Finished loading net ...')

		except Exception as e:
			print(e)
			print('No neural net model found ...')
			print('reverting to default OpenCV tracker')
			self.tracker_type = Tracking.DEFAULT_TRACKER


		# Create the tracker

		self.create_tracker()






	def track(self,image, thresh_image, start_flag = False):
		
	
		# Initialize the tracker
		if(start_flag == True or self.trackerActive == False):
			
			# Tracker initialization
			if(self.init_method=="roi"):
				self.bbox = tuple(self.roi_bbox)
				self.centroid_image = self.centroid_from_bbox(self.bbox)
				self.isCentroidFound = True
			else:
				self.isCentroidFound, self.centroid_image, self.bbox = image_processing.find_centroid_basic_Rect(thresh_image)
				self.bbox = image_processing.scale_square_bbox(self.bbox, Tracking.BBOX_SCALE_FACTOR, square = True)

			if(self.bbox is not None):
				print('Starting tracker with initial bbox: {}'.format(self.bbox))
				self.init_tracker(image, self.centroid_image, self.bbox)
				self.trackerActive = True
				self.rect_pts = self.rectpts_from_bbox(self.bbox)

		# Continue tracking an object using tracking
		else:
			# Find centroid using the tracking.

			#@@@ Debugging
			# print('Continued track: Using Tracker...')

			objectFound, self.bbox = self.update_tracker(image, thresh_image) # (x,y,w,h)

			# print('Object found?: {}'.format(objectFound))

			if(objectFound):

				self.isCentroidFound = True

				self.centroid_image = self.centroid_from_bbox(self.bbox) + self.origLoc

				self.bbox = np.array(self.bbox)
				self.bbox[0], self.bbox[1] = self.bbox[0] + self.origLoc[0], self.bbox[1] + self.origLoc[1]

				self.rect_pts = self.rectpts_from_bbox(self.bbox)
			else:
				print('No object found ...')
				self.isCentroidFound = False
				self.trackerActive = False

		return self.isCentroidFound, self.centroid_image, self.rect_pts

	def reset(self):
		print('Reset image tracker state')
		self.start_flag = True
		self.trackerActive = False
		self.isCentroidFound = False

	def create_tracker(self):

		if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
			self.tracker = self.OPENCV_OBJECT_TRACKERS[self.tracker_type]()

		elif(self.tracker_type in self.NEURALNETTRACKERS.keys()):
			print('Using {} tracker'.format(self.tracker_type))
			pass
			

	# Signal from Tracking Widget connects to this Function
	def update_tracker_type(self, tracker_type):
		self.tracker_type = tracker_type

		#@@@ Testing
		print('Image tracker set to {}'.format(self.tracker_type))
		# Update the actual tracker
		self.create_tracker()

	def update_init_method(self, method):

		self.init_method = method
		print("Tracking init method set to : {}".format(self.init_method))

	def init_tracker(self, image, centroid, bbox):

		# Initialize the OpenCV based tracker
		if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):

			print('Initializing openCV tracker')
			print(self.tracker_type)
			print(bbox)
			if(self.color == False):
				image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

			self.tracker.init(image, bbox)

		# Initialize Neural Net based Tracker
		elif(self.tracker_type in self.NEURALNETTRACKERS.keys()):
			# Initialize the tracker with this centroid position
			print('Initializing with daSiamRPN tracker')

			target_pos, target_sz = np.array([centroid[0], centroid[1]]), np.array([bbox[2], bbox[3]])

			if(self.color==False):
				image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

			self.state = SiamRPN_init(image, target_pos, target_sz, self.net)
			print('daSiamRPN tracker')


		else:
			pass


	def update_tracker(self, image, thresh_image):
		# Input: image or thresh_image
		# Output: new_bbox based on tracking

		new_bbox = None

		if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
			self.origLoc = np.array([0,0])
			# (x,y,w,h)\
			if(self.color==False):
				image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

			ok, new_bbox = self.tracker.update(image)

			# print(ok)
			# print(new_bbox)
			return ok, new_bbox

				

		elif(self.tracker_type in self.NEURALNETTRACKERS.keys()):

			self.origLoc = np.array([0,0])

			if(self.color==False):
				image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

			self.state = SiamRPN_track(self.state, image)

			ok = True

			if(ok):
				# (x,y,w,h)
				new_bbox = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])

				new_bbox = [int(l) for l in new_bbox]

				# print('Updated daSiamRPN tracker')


			return ok, new_bbox

		else:
			# If no tracker is specified, use basic thresholding and
			# nearest neighbhour tracking. i.e Look for objects in a search region 
			# near the last detected centroid

			# Get the latest thresholded image from the queue
			# thresh_image = 

			pts, thresh_image_cropped = image_processing.crop(thresh_image, self.centroid_image, self.searchArea)
			
			self.origLoc = pts[0]


			isCentroidFound, centroid, new_bbox = image_processing.find_centroid_basic_Rect(thresh_image_cropped)

			
			return isCentroidFound, new_bbox

		# @@@ Can add additional methods here for future tracker implementations

		


	def centroid_from_bbox(self, bbox):

		# Coordinates of the object centroid are taken as the center of the bounding box
		assert(len(bbox) == 4)

		cx = int(bbox[0] + bbox[2]/2)
		cy = int(bbox[1] + bbox[3]/2)

		centroid = np.array([cx, cy])

		return centroid

	def rectpts_from_bbox(self, bbox):
		if(self.bbox is not None):
			pts = np.array([[bbox[0], bbox[1]],[bbox[0] + bbox[2], bbox[1] + bbox[3]]], dtype = 'int')
		else:
			pts = None
		return pts

	def update_searchArea(self, value):

		self.searchArea = value

	def set_roi_bbox(self, bbox):
		# Updates roi bbox from ImageDisplayWindow
		self.roi_bbox = bbox
		print('Rec bbox from ImageDisplay: {}'.format(self.roi_bbox))









