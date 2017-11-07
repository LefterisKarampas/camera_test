import numpy as np
import rospy
import argparse
import imutils
import cv2
from collections import deque
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




class image_converter:

	def __init__(self):

		self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

		red_lower = np.array([136,87,111],np.uint8)
		red_upper = np.array([180,255,255],np.uint8)

		blue_lower = np.array([99,115,150],np.uint8)
		blue_upper = np.array([110,255,255],np.uint8)

		yellow_lower = np.array([22,60,200],np.uint8)
		yellow_upper = np.array([60,255,255],np.uint8)

		#finding the range of red,blue and yellow color in the image
		red = cv2.inRange(hsv,red_lower,red_upper)
		blue = cv2.inRange(hsv,blue_lower,blue_upper)
		yellow = cv2.inRange(hsv,yellow_lower,yellow_upper)

		#Morphological transformation,Dilation
		kernal = np.ones((5,5),"uint8")

		red = cv2.dilate(red,kernal)
		res = cv2.bitwise_and(image,image,mask=red)

		blue = cv2.dilate(blue,kernal)
		res1 = cv2.bitwise_and(image,image,mask=blue)

		yellow = cv2.dilate(yellow,kernal)
		res2 = cv2.bitwise_and(image,image,mask=yellow)

		#Tracking the Red Color
		(_,contours,hierarchy) = cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(0,0,255),2)
				cv2.putText(image,"Red",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))

		#Tracking the Blue Color
		contours = cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			#((x, y), radius) = cv2.minEnclosingCircle(contour)
			#M = cv2.moments(contour)
			#center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(255,0,0),2)
				cv2.putText(image,"Blue",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))



		#Tracking the Yellow Color
		(_,contours,hierarchy) = cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(0,255,0),2)
				cv2.putText(image,"Yellow",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
		cv2.imshow("Color Tracking",image)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image,"bgr8"))
		except CvBridgeError as e:
			print(e)
		#self.image_pub.publish(image)
		if cv2.waitKey(10) & 0xFF == ord('q'):
			cap.release()
			#cv2.destroyAllWindows()



if(True):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()