import utils.image_processing as image_processing

class Tracker_Image(object):
	
	def __init__(self, OpenCVTrackers, NeuralNetTrackers):
		
		self.OpenCVTrackers = OpenCVTrackers
		self.NeuralNetTrackers = NeuralNetTrackers

		# Centroid of object from the image
		self.centroid_image = None # (2,1)
		# Centroid of object along optical axis
		self.centroid_focus = None # (1,)
		self.bbox = None

		self.tracker_type = None
		self.tracker = None

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


	def track(self,image, tracker, tracker_type, start_flag = False):

		self.tracker_type = tracker_type
		self.tracker = tracker
		# Find centroid based on simple image thresholding
		if(start_flag):
			isCentroidFound, self.centroid, self.bbox = 
				image_processing.find_centroid_basic_Rect(thresh_image)
			
			
			self.init_tracker(image, self.centroid, self.bbox)

		# Find centroid using the tracking.
		else:

			self.bbox = self.update_tracker(image)


	def find_object(self, thresh_image):

		# Threshold image:

		isCentroidFound, self.centroidLoc, self.bbox = 
				image_processing.find_centroid_basic_Rect(thresh_image)

		return isCentroidFound, 



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
			
			ok, new_bbox = self.tracker.update(image_data)

				

		elif(self.tracker_type in NeuralNetTrackers.keys()):

			self.state = SiamRPN_track(self.state, image_data)

			ok = True

			if(ok):

				new_bbox = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])

				new_bbox = [int(l) for l in new_bbox]

		# Can add additional methods here for future tracker implementations

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


class PID_Controller(object):
	def __init__(self):
		self.P = 0
		self.I = 0
		self.D = 0

	def get_actuation(self,error):
		pass

	def set_P(self,P):
		self.P = P
	def set_I(self,I):
		self.I = I
	def set_D(self,D):
		self.D = D

