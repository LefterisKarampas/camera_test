# import the necessary packages
import argparse
import imutils
import cv2
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2",Image)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cv2.imshow("Gray",gray)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		cv2.imshow("blurred",blurred)
		thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
		cv2.imshow("thresh",thresh)
		# find contours in the thresholded image
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]

		# loop over the contours
		for c in cnts:
			# compute the center of the contour
			M = cv2.moments(c)
			if (M["m00"] == 0):
				M["m00"]=1
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])

			# draw the contour and center of the shape on the image
			cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
			#cv2.putText(image, "center", (cX - 20, cY - 20),
			#cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			# show the image
		cv2.imshow("Image", image)
		cv2.waitKey(1)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)