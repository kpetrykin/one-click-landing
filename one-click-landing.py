#! /usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import imutils
import socket
import time
import cv2
import numpy as np
import math

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Range
from sensor_msgs.msg import CameraInfo

from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import SetMode

# Pipeline to send video frames to QGC
stream = cv2.VideoWriter("appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast tune=zerolatency ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600 sync=false", cv2.CAP_GSTREAMER, 0, 60.0, (640, 480))
if not stream.isOpened():
    print("Stream not opened")

# initialize OpenCV's CSRT tracker
tracker = cv2.TrackerCSRT_create()

rospy.init_node('one_click_landing')
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
land = rospy.ServiceProxy('land', Trigger)
set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

bridge = CvBridge()

tracker_initialized = False

# Get camera params
camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)

# Central point of the camera
central_point_x = camera_info.K[2]
central_point_y = camera_info.K[5]

# Focal length in pixels
focal_length = camera_info.K[0]

def set_tracker_bbox(event, x, y, flags, param):
	global mouseX, mouseY, tracker_bbox, tracker_initialized, cv_image, tracker,\
		focal_length, central_point_x, central_point_y

	if event == cv2.EVENT_LBUTTONDOWN:
		
		r = 15
		tracker_bbox = (x - r, y - r, r * 2, r * 2)

		if tracker_initialized:
			del tracker
			tracker = cv2.TrackerCSRT_create()

		# Initialize tracker with first frame and bounding box
		ok = tracker.init(cv_image, tracker_bbox)

		print(ok)

		tracker_initialized = True


# Setpoint calculation and navigation to it
def navigate_to_calculated_setpoint(tracking_point_x, tracking_point_y, rangefinder_data):
	real_x = ((float(tracking_point_x) - central_point_x) / focal_length) * rangefinder_data.range
	real_y = ((float(tracking_point_y) - central_point_y) / focal_length) * rangefinder_data.range

	print(math.hypot(abs(real_x), abs(real_y)))

	# Lower speed on close horizontal distances to prevent oscillations
	if math.hypot(abs(real_x), abs(real_y)) < 2.0:
		speed = 0.5
	else:
		speed = 3.0

	print("Navigate to x: ", real_x, " y: ", real_y, " z: ", rangefinder_data.range, "speed: ", speed)
	navigate(x=-real_y, y=-real_x, z=-rangefinder_data.range, yaw=float('nan'), speed=speed, frame_id='base_link', auto_arm=True)


# It is for simple timer functionality
last_time = rospy.get_time()

def image_callback(frame):

	# check to see if we have reached the end of the stream
	if frame is None:
		return

	global cv_image
	cv_image = bridge.imgmsg_to_cv2(frame, desired_encoding='passthrough')

	global tracker_initialized
	global last_time

	global tracker

	if tracker_initialized:
		(ok, bbox) = tracker.update(cv_image)

		# HOLD mode when lost target
		if not ok:
			print("Tracker reported failure")
			set_mode(custom_mode='HOLD')
			del tracker
			tracker = cv2.TrackerCSRT_create()
			tracker_initialized = false

		(x, y, w, h) = [int(v) for v in bbox]

		landing_point_x = x + w/2
		landing_point_y = y + h/2

		cv2.circle(cv_image, (landing_point_x, landing_point_y), 1, (0, 0, 255), 1)
		cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

		telemetry = get_telemetry()

		# Let's correct our navigation setpoint using new determined coordinates
		if rospy.get_time() - last_time > 1 and telemetry.armed:
			rangefinder_data = rospy.wait_for_message('rangefinder/range', Range)

			# If we are above 2 meters
			if rangefinder_data.range > 2.0:
				navigate_to_calculated_setpoint(landing_point_x, landing_point_y, rangefinder_data)
			else: # Stop tracking point, just land here
				land()
				if tracker:
					tracker_initialized = False
					del tracker
					tracker = cv2.TrackerCSRT_create()


			last_time = rospy.get_time()

	# show the output frame
	cv2.imshow("Frame", cv_image)
	cv2.setMouseCallback('Frame', set_tracker_bbox)

	# Sending frame to QGC
	stream.write(cv_image)

	key = cv2.waitKey(1) & 0xFF
	
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()

udp_video_socket.close()

# close all windows
cv2.destroyAllWindows()