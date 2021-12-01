#!/usr/bin/env python3

from sensor_msgs.msg import Image
from drone_vision.msg import Mask
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy

class image():

	def __init__(self):

		# Making a publisher 
		self.mask_pub = rospy.Publisher('/mask_info', Mask, queue_size=1)
		# Subscribing to /camera/camera/image_raw
		self.image_sub = rospy.Subscriber("/drone/camera1/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.rate = rospy.Rate(100) #Publishing at a rate of 100Hz
		self.mask_msg=Mask()  # This will contain the message structure of message type drone_vision/Mask	
	
	# Callback function of camera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.get_mask()
		except CvBridgeError as e:
			print(e)
			return

	def publish_data(self):
		self.mask_pub.publish(self.mask_msg)
		self.rate.sleep()	
	
	def classify_images(self):
		# Load a model imported from Tensorflow
		tensorflowNet = cv2.dnn.readNetFromTensorflow('../frozen_models/frozen_graph.pb', '../frozen_models/model.pbtxt')
		mean = np.array([1.0, 1.0, 1.0]) * 127.5
		scale = 1 / 127.5
		# Use the given image as input, which needs to be blob(s).
		tensorflowNet.setInput(cv2.dnn.blobFromImage(self.img.astype(np.float32), scale, size=(224, 224), mean=mean, swapRB=True, crop=False))
		# Runs a forward pass to compute the net output
		networkOutput = tensorflowNet.forward()[0]
		return networkOutput[0], networkOutput[1]

	def get_mask(self):
		self.mask_msg.mask, self.mask_msg.non_mask = self.classify_images()
		self.publish_data()

def main():
	rospy.init_node('mask_detection')
	image_obj = image()
	rospy.spin()

if __name__ == '__main__':
	try: main()
	except rospy.ROSInterruptException: pass
