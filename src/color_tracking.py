import numpy as np
import rospy
import argparse
import imutils
import cv2
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

		green_lower = np.array([45, 100, 50],np.uint8)
		green_upper = np.array([75,255,255],np.uint8)

		#finding the range of red,blue and yellow color in the image
		red = cv2.inRange(hsv,red_lower,red_upper)
		blue = cv2.inRange(hsv,blue_lower,blue_upper)
		yellow = cv2.inRange(hsv,yellow_lower,yellow_upper)
		green = cv2.inRange(hsv,green_lower,green_upper)

		#Morphological transformation,Dilation
		kernal = np.ones((5,5),"uint8")

		red = cv2.dilate(red,kernal)
		res = cv2.bitwise_and(image,image,mask=red)

		blue = cv2.dilate(blue,kernal)
		res1 = cv2.bitwise_and(image,image,mask=blue)

		yellow = cv2.dilate(yellow,kernal)
		res2 = cv2.bitwise_and(image,image,mask=yellow)

		green = cv2.dilate(green,kernal)
		res3 = cv2.bitwise_and(image,image,mask=yellow)

		#Tracking the Red Color
		(_,contours,hierarchy) = cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(0,0,255),2)
				print("Red: ((%d,%d),(%d,%d))" % (x,y,x+w,y+h))
				cv2.putText(image,"Red",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))

		#Tracking the Blue Color
		(_,contours,hierarchy) = cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(255,0,0),2)
				print("Blue: ((%d,%d),(%d,%d))" % (x,y,x+w,y+h))
				cv2.putText(image,"Blue",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))

		#Tracking the Yellow Color
		(_,contours,hierarchy) = cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(0,255,0),2)
				print("Yellow: ((%d,%d),(%d,%d))" % (x,y,x+w,y+h))
				cv2.putText(image,"Yellow",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))

		(_,contours,hierarchy) = cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 300):
				x,y,w,h = cv2.boundingRect(contour)
				image = cv2.rectangle(image,(x,y),((x+w),(y+h)),(0,255,0),2)
				print("Green: ((%d,%d),(%d,%d))" % (x,y,x+w,y+h))
				cv2.putText(image,"Green",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))

		cv2.imshow("Color Tracking",image)
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