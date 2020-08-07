# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# initialize OpenCV's CSRT tracker
tracker = cv2.TrackerCSRT_create()

rospy.init_node('landing_target_tracker')
bridge = CvBridge()

tracker_initialized = False

def set_tracker_bbox(event, x, y, flags, param):
	global mouseX, mouseY, tracker_bbox, tracker_initialized, cv_image, tracker

	if event == cv2.EVENT_LBUTTONDOWN:
		
		r = 10
		tracker_bbox = (x - r, y - r, r * 2, r * 2)

		print(tracker_bbox)

		if tracker_initialized:
			del tracker
			tracker = cv2.TrackerCSRT_create()

		# Initialize tracker with first frame and bounding box
		ok = tracker.init(cv_image, tracker_bbox)

		print(ok)

		tracker_initialized = True

		mouseX, mouseY = x, y


def image_callback(frame):

	# check to see if we have reached the end of the stream
	if frame is None:
		return

	global cv_image
	cv_image = bridge.imgmsg_to_cv2(frame, desired_encoding='passthrough')

	global tracker_initialized

	if tracker_initialized:
		(ok, bbox) = tracker.update(cv_image)
		(x, y, w, h) = [int(v) for v in bbox]
		cv2.circle(cv_image, (x + w/2, y + h/2), 1, (0, 0, 255), 1)
		cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

	# show the output frame
	cv2.imshow("Frame", cv_image)
	cv2.setMouseCallback('Frame', set_tracker_bbox)
	
	key = cv2.waitKey(1) & 0xFF

	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	# if key == ord("s"):
	# 	# select the bounding box of the object we want to track (make
	# 	# sure you press ENTER or SPACE after selecting the ROI)
	# 	bbox = cv2.selectROI("Frame", cv_image, fromCenter=False,
	# 		showCrosshair=True)
	# 	ok = tracker.init(cv_image, bbox)



	# if the `q` key was pressed, break from the loop
	# elif key == ord("q"):
	# 	return

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()

# close all windows
cv2.destroyAllWindows()