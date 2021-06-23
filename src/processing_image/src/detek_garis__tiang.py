#!/usr/bin/python

from __future__ import print_function
from collections import deque

import numpy as np
import cv2
import imutils
import math
import time
import sys
# from GUI_KRSBI import *

# penambahan baru
import rosparam
import rospy
import roslib

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Int64

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from op3_ball_detector.msg import CircleSetStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


def nothing():
    pass


roslib.load_manifest('processing_image')

hsvLow = (0, 0, 106)
hsvMax = (179, 45, 255)

# 0, 0 , 229
#   179 39 255
#
treshold_val = 0
contours = []


class DeteksiGaris:

    def __init__(self):
        global hsvMax, hsvLow, treshold_val

        self.image_pub = rospy.Publisher("KRSBI/image/garis_garis/image_out", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("usb_cam_node/image_raw", Image, self.callback)

        self.rate = rospy.Rate(30)

        self.hsv = rospy.Publisher("KRSBI/image/deteksi_garis/image_param/hsv", Twist, queue_size=10)
        self.hsv_sub = rospy.Subscriber("KRSBI/image/deteksi_garis/image_param/hsv", Twist, self.hsvcallback)
        # self.image_sub = rospy.Subscriber("usb_cam_node_node/image_raw", Image, self.callback)

        self.state = rospy.Publisher("KRSBI/image/garis_garis/image/state", String, queue_size=10)
        if rospy.has_param("/garis_h_low"):

            hsvLower = (rospy.get_param("/garis_h_low"), rospy.get_param("/garis_s_low"), rospy.get_param("/garis_v_low"))
            hsvUpper = ( rospy.get_param("/garis_h_up"), rospy.get_param("/garis_s_up"), rospy.get_param("/garis_v_up"))

        else:

            hsvLow = (0, 0, 106)
            hsvMax = (179, 45, 255)

        # rosparam.set_param('/usb_cam_node_node/brightness', '10')
        # rosparam.set_param('/usb_cam_node_node/exposure', '10')
        self.i = 0
        # self.ParamUpdate()
        self.bridge = CvBridge()

    def state_garis(self, status):
        if status == 1:
            self.state.publish("OK")
        elif status == 0:
            self.state.publish("NOTDETECT")

    def hsvcallback(self, data):
        rosparam.set_param('/garis_h_low', str(data.linear.x))
        rosparam.set_param('/garis_s_low', str(data.linear.y))
        rosparam.set_param('/garis_v_low', str(data.linear.z))
        rosparam.set_param('/garis_h_up', str(data.angular.x))
        rosparam.set_param('/garis_s_up', str(data.angular.y))
        rosparam.set_param('/garis_v_up', str(data.angular.z))

    def callback(self, data):
        global hsvLow, hsvMax, treshold_val

        rosparam.set_param('/usb_cam_node/brightness', '8')
        rosparam.set_param('/usb_cam_node/exposure', '3')

        # tresholding_val = Int8()

        hsvMsg = Twist()
        if rospy.has_param("/garis_h_low"):

            hsvMsg.linear.x = rospy.get_param("/garis_h_low")
            hsvMsg.linear.y = rospy.get_param("/garis_s_low")
            hsvMsg.linear.z = rospy.get_param("/garis_v_low")
            hsvMsg.angular.x = rospy.get_param("/garis_h_up")
            hsvMsg.angular.y = rospy.get_param("/garis_s_up")
            hsvMsg.angular.z = rospy.get_param("/garis_v_up")

        else:
            hsvMsg.linear.x = 0
            hsvMsg.linear.y = 0
            hsvMsg.linear.z = 106
            hsvMsg.angular.x = 179
            hsvMsg.angular.y = 45
            hsvMsg.angular.z = 255

        hsvLow = np.array([hsvMsg.linear.x, hsvMsg.linear.y, hsvMsg.linear.z])
        hsvMax = np.array([hsvMsg.angular.x, hsvMsg.angular.y, hsvMsg.angular.z])

        try:
            # encoding ke blue green red 8 bit
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        # Resize Frame
        # lebar = 640
        # tinggi = 360
        frame = imutils.resize(frame, width=640)
        img = frame.copy()

        img = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsvLow, hsvMax)
        highTresh, mask= cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        lowTresh = 0.5 * highTresh
        # thres = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 2)

        # imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        # Specify size on horizontal axis
        cols = mask.shape[1]
        horizontal_size = cols // 30
        # Create structure element for extracting horizontal lines through morphology operations
        horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_size, 1))
        # Apply morphology operations
        horizontal = cv2.erode(mask, horizontalStructure)
        horizontal = cv2.dilate(mask, horizontalStructure)
        # Show extracted horizontal lines
        canny = cv2.Canny(horizontal, lowTresh, highTresh)        #
        linesP = cv2.HoughLinesP(canny, 1, np.pi / 180, threshold=100, lines=66, minLineLength=1, maxLineGap=10)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]

                width = l[2] - l[0]
                heigth = l[3] - l[1]
                cek ='0'
                if width >= 10 and heigth <= 1:
                    # print(width)
                    cek = '1'
                    cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 1)
                    cv2.circle(img, (l[0], l[1]), 3, (0, 0, 255), 5)
                    cv2.circle(img, (l[2], l[3]), 3, (0, 0, 255), 5)
                if cek == '0':
                    if width >= 10 and heigth >= 2:
                        cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 1)
                        cv2.circle(img, (l[0], l[1]), 3, (0, 0, 255), 5)
                        cv2.circle(img, (l[2], l[3]), 3, (0, 0, 255), 5)
                # if width >= 1 and heigth <= 1:
                #     # print(width)
                #
                #     cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 1)
                #     cv2.circle(img, (l[0],l[1]), 3, (0, 0, 255), 5)
                #     cv2.circle(img, (l[2],l[3]), 3, (0, 0, 255), 5)
                #
                # if width >= 1 and heigth >= 5 :
                #     cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 1)
                #     cv2.circle(img, (l[0], l[1]), 3, (0, 255, 0), 5)
                #     cv2.circle(img, (l[2], l[3]), 3, (0, 255, 0), 5)
        # #
        # # cv2.imshow('Shape image', frame)
        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", canny)
        # cv2.imshow("Masking", thres)

        cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", img)
        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # self.mask_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))

        except CvBridgeError as e:
            print(e)

        self.rate.sleep()


# ball_detector()


rospy.init_node('processing_image_garis', anonymous=True)

if __name__ == '__main__':
    try:

        objgaris = DeteksiGaris()
        rospy.spin()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass


