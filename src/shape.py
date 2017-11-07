# import the necessary packages
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import rospy
import argparse
import imutils
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

# construct the argument parse and parse the arguments

# load the image, convert it to grayscale, and blur it slightly
def draw_tf(img, corners, imgpts):
	corner = tuple(corners[0].ravel())
	img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
	img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
	img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
	return img

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

		'''gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (7, 7), 0)

		# perform edge detection, then perform a dilation + erosion to
		# close gaps in between object edges
		edged = cv2.Canny(gray, 50, 100)
		edged = cv2.dilate(edged, None, iterations=1)
		edged = cv2.erode(edged, None, iterations=1)

		# find contours in the edge map
		cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]

		# sort the contours from left-to-right and initialize the
		# 'pixels per metric' calibration variable
		(cnts, _) = contours.sort_contours(cnts)
		pixelsPerMetric = None

		# loop over the contours individually
		for c in cnts:
			# if the contour is not sufficiently large, ignore it
			if cv2.contourArea(c) < 100:
				continue

			# compute the rotated bounding box of the contour
			orig = image.copy()
			box = cv2.minAreaRect(c)
			box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
			box = np.array(box, dtype="int")

			# order the points in the contour such that they appear
			# in top-left, top-right, bottom-right, and bottom-left
			# order, then draw the outline of the rotated bounding
			# box
			box = perspective.order_points(box)
			cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

			# loop over the original points and draw them
			for (x, y) in box:
				cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

			# unpack the ordered bounding box, then compute the midpoint
			# between the top-left and top-right coordinates, followed by
			# the midpoint between bottom-left and bottom-right coordinates
			(tl, tr, br, bl) = box
			(tltrX, tltrY) = midpoint(tl, tr)
			(blbrX, blbrY) = midpoint(bl, br)

			# compute the midpoint between the top-left and top-right points,
			# followed by the midpoint between the top-righ and bottom-right
			(tlblX, tlblY) = midpoint(tl, bl)
			(trbrX, trbrY) = midpoint(tr, br)

			# draw the midpoints on the image
			cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
			cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
			cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
			cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

			# draw lines between the midpoints
			cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
				(255, 0, 255), 2)
			cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
				(255, 0, 255), 2)

			# compute the Euclidean distance between the midpoints
			dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
			dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))

			# if the pixels per metric has not been initialized, then
			# compute it as the ratio of pixels to supplied metric
			# (in this case, inches)
			if pixelsPerMetric is None:
				pixelsPerMetric = dB / 1.0

			# compute the size of the object
			dimA = dA / pixelsPerMetric
			dimB = dB / pixelsPerMetric

			# draw the object sizes on the image
			cv2.putText(orig, "{:.1f}in".format(dimA),
				(int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
				0.65, (255, 255, 255), 2)
			cv2.putText(orig, "{:.1f}in".format(dimB),
				(int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
				0.65, (255, 255, 255), 2)

			# show the output image
			cv2.imshow("Image", orig)
			cv2.waitKey(1)'''

		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		objp = np.zeros((6*9,3), np.float32)
		objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

		# Arrays to store object points and image points from all the images.
		objpoints = [] # 3d point in real world space
		imgpoints = [] # 2d points in image plane.
		gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		cv2.imshow('gray',gray)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
		axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
		# If found, add object points, image points (after refining them)
		if ret == True:
			#print(corners)
			objpoints.append(objp)

			corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
			#print(corners2)
			imgpoints.append(corners2)

			# Draw and display the corners
			img = cv2.drawChessboardCorners(image, (9,6), corners2,ret)

			ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
			corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

			# Find the rotation and translation vectors.
			b,rvecs,tvecs,_ = cv2.solvePnPRansac(objp, corners2, mtx, dist)


			# project 3D points to image plane
			if(b == True):
				imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
				image = draw_tf(image,corners2,imgpts)
		cv2.imshow('img',image)
		cv2.waitKey(1)

		#cv2.destroyAllWindows()

		try:
		  self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
		except CvBridgeError as e:
		  print(e)
		


if(True):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()