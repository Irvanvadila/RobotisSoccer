#!/usr/bin/python
# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages

from __future__ import print_function
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from std_msgs.msg import Int64
import rosparam

# penambahan baru
import rospy
import std_msgs
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import Bool
# tambahan fawwaz
import roslib

roslib.load_manifest('processing_image')
import sys
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from op3_ball_detector.msg import CircleSetStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

buffer = 64
pts = deque(maxlen=64)

hsvLower = (115,42,60)
hsvUpper = (179,255,255)
class image_converter:

    def __init__(self):
        global hsvLower, hsvUpper

        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("KRSBI/processing_image/deteksi_bola/image/image_out", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("usb_cam_node/image_raw", Image, self.callback)

        self.mask_pub = rospy.Publisher("KRSBI/processing_image/deteksi_bola/image/mask_out", Image, queue_size=10)

        self.hsv = rospy.Publisher("KRSBI/processing_image/deteksi_bola/image_param/hsv", Twist, queue_size=10)
        self.hsv_sub = rospy.Subscriber("KRSBI/processing_image/deteksi_bola/image_param/hsv", Twist, self.hsvcallback)

        self.state = rospy.Publisher("KRSBI/processing_image/deteksi_bola/image_processing/state", String, queue_size=10)

        self.rate = rospy.Rate(30)

        if rospy.has_param("/bola_h_low"):

            hsvLower = (rospy.get_param("/bola_h_low"), rospy.get_param("/bola_s_low"), rospy.get_param("/bola_v_low"))
            hsvUpper = ( rospy.get_param("/bola_h_up"), rospy.get_param("/bola_s_up"), rospy.get_param("/bola_v_up"))

        else:
            hsvLower = (115, 42, 60)
            hsvUpper = (179, 255, 255)

    def state_bola(self, status):
        if status == 1:
            self.state.publish("OK")
        elif status == 0:
            self.state.publish("NOTDETECT")

    def hsvcallback(self, data):
        rosparam.set_param('/bola_h_low', str(data.linear.x))
        rosparam.set_param('/bola_s_low', str(data.linear.y))
        rosparam.set_param('/bola_v_low', str(data.linear.z))
        rosparam.set_param('/bola_h_up', str(data.angular.x))
        rosparam.set_param('/bola_s_up', str(data.angular.y))
        rosparam.set_param('/bola_v_up', str(data.angular.z))

    def callback(self, data):
        global hsvLower
        global hsvUpper

        rosparam.set_param('/usb_cam_node/brightness', '8')
        rosparam.set_param('/usb_cam_node/exposure', '3')

        hsvMsg = Twist()

        if rospy.has_param("/bola_h_low"):


            hsvMsg.linear.x = rospy.get_param("/bola_h_low")
            hsvMsg.linear.y = rospy.get_param("/bola_s_low")
            hsvMsg.linear.z = rospy.get_param("/bola_v_low")
            hsvMsg.angular.x = rospy.get_param("/bola_h_up")
            hsvMsg.angular.y = rospy.get_param("/bola_s_up")
            hsvMsg.angular.z = rospy.get_param("/bola_v_up")

        else:
            hsvMsg.linear.x = 115
            hsvMsg.linear.y = 42
            hsvMsg.linear.z = 60
            hsvMsg.angular.x = 179
            hsvMsg.angular.y = 255
            hsvMsg.angular.z = 255

        self.hsv.publish(hsvMsg)

        # tesMsg = rospy.get_param("/tes")
        hsvLower = (hsvMsg.linear.x, hsvMsg.linear.y, hsvMsg.linear.z)
        hsvUpper = (hsvMsg.angular.x, hsvMsg.angular.y, hsvMsg.angular.z)

        try:
            # encoding ke blue green red 8 bit
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = imutils.resize(frame, width=640, height=360)
        lebar = frame.shape[1]
        tinggi = frame.shape[0]

        # def preprocessInputFrame(self,frame):
        #	imutils.resize(frame, width=500, height=500)

        # mulai image processing
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = morphology(hsv)
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]),
                      int(M["m01"] / M["m00"]))
            # self.pub.publish(point_x)

            # only proceed if the radius meets a minimum size
            if radius > 3:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                print(x,y, center)
                x = (x / lebar) * 2 - 1
                y = (y / tinggi) * 2 - 1

                self.state_bola(1)
                circleMsg = CircleSetStamped()
                circlePoint = Point()
                circlePoint.x = x
                circlePoint.y = y
                circlePoint.z = radius
                circleMsg.circles = [circlePoint]

                pubCircle.publish(circleMsg)
                self.rate.sleep()
        else:
            self.state_bola(0)

        # update the points queue
        pts.appendleft(center)
        cv2.imshow("Image window", frame)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_rgb, "bgr8"))

        except CvBridgeError as e:
            print(e)


def morphology(hsv_image):
    mask = cv2.inRange(hsv_image, hsvLower, hsvUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask


# ball_detector()
if __name__ == "__main__":

    try:
        rospy.init_node('processing_image_bola', anonymous=False)
        image_converter()
        pubCircle = rospy.Publisher("KRSBI/processing_image/deteksi_bola/coordinate/bola", CircleSetStamped,
                                    queue_size=1)

        time.sleep(2.0)
        rospy.spin()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass