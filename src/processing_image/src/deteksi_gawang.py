#!/usr/bin/python

from __future__ import print_function
from collections import deque

import numpy as np
import cv2
import imutils
import time
import sys
# from GUI_KRSBI import *

# penambahan baru
import rosparam
import rospy
import roslib

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32
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

hsvLow = (94, 57, 63)
hsvMax = (133, 255,255)
treshold_val = 0
contours = []

h_low = rospy.get_param("/tiang_h_low")
s_low = rospy.get_param("/tiang_s_low")
v_low = rospy.get_param("/tiang_v_low")
h_up = rospy.get_param("/tiang_h_up")
s_up = rospy.get_param("/tiang_s_up")
v_up = rospy.get_param("/tiang_v_up")
brigtnessCam = rospy.get_param("/usb_cam_node/brightness")
exposureCam = rospy.get_param("/usb_cam_node/exposure")

class DeteksiTiang:

    def __init__(self):

        self.image_pub = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/image/image_out", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("usb_cam_node/image_raw", Image, self.callback)

        self.mask_pub = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/image/mask_out", Image, queue_size=10)

        self.tiang_x_y = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/x", Point, queue_size=10)

        self.pub_w = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/w", Float64, queue_size=10)
        self.pub_h = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/h", Float64, queue_size=10)

        self.pub_w_lacth = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/w_lacth", Float64, queue_size=10)
        self.pub_h_lacth = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/h_lacth", Float64, queue_size=10)

        self.aspectRatio = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/aspectRatio", String, queue_size=10)
        self.hsv = rospy.Publisher("KRSBI/processing_image/deteksi_tiang/koordinat/image_param/hsv", Twist, queue_size=10)
        self.hsv_sub = rospy.Subscriber("KRSBI/processing_image/deteksi_tiang/koordinat/image_param/hsv", Twist, self.hsvcallback)
        # self.image_sub = rospy.Subscriber("usb_cam_node/image_raw", Image, self.callback)

        self.state = rospy.Publisher("KRSBI/processing_image/deteksi_bola/image_processing/state", String, queue_size=10)

        # rosparam.set_param('/usb_cam_node/brightness', '10')
        # rosparam.set_param('/usb_cam_node/exposure', '10')

        # self.ParamUpdate()
        self.bridge = CvBridge()

    # def ParamUpdate(self):
    #
    #     rosparam.set_param('/h_low', self.h_low)
    #     rosparam.set_param('/s_low', self.s_low)
    #     rosparam.set_param('/v_low', self.v_low)
    #     rosparam.set_param('/h_up',  self.h_up)
    #     rosparam.set_param('/s_up',  self.s_up)
    #     rosparam.set_param('/v_up',  self.v_up)
    #
    #
    def hsvcallback(self, data):
        rosparam.set_param('/h_low', str(data.linear.x))
        rosparam.set_param('/s_low', str(data.linear.y))
        rosparam.set_param('/v_low', str(data.linear.z))
        rosparam.set_param('/h_up', str(data.angular.x))
        rosparam.set_param('/s_up', str(data.angular.y))
        rosparam.set_param('/v_up', str(data.angular.z))

    def callback(self, data):
        global hsvLow, hsvMax

        rosparam.set_param('/usb_cam/brightness', '8')
        rosparam.set_param('/usb_cam/exposure', '3')

        hsvMsg = Twist()
        hsvMsg.linear.x = rospy.get_param("/tiang_h_low")
        hsvMsg.linear.y = rospy.get_param("/tiang_s_low")
        hsvMsg.linear.z = rospy.get_param("/tiang_v_low")
        hsvMsg.angular.x = rospy.get_param("/tiang_h_up")
        hsvMsg.angular.y = rospy.get_param("/tiang_s_up")
        hsvMsg.angular.z = rospy.get_param("/tiang_v_up")

        self.hsv.publish(hsvMsg)

        hsvLow = np.array([hsvMsg.linear.x, hsvMsg.linear.y, hsvMsg.linear.z])
        hsvMax = np.array([hsvMsg.angular.x, hsvMsg.angular.y, hsvMsg.angular.z])

        try:
            # encoding ke blue green red 8 bit
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Resize Frame
        frame = imutils.resize(frame, width=640, height=360)
        lebar = frame.shape[1]
        tinggi = frame.shape[0]

        output_blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        hsv = cv2.cvtColor(output_blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsvLow, hsvMax)
        kernel = np.ones((4, 4), np.uint8)
        erosion = cv2.erode(mask, kernel)
        dilation = cv2.dilate(mask, kernel)
        #
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        output = cv2.bitwise_and(frame, frame, mask=mask)

        image_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        # ret, treshold = cv2.threshold(image_gray, 127, 255, 0)
        #
        _, thresh = cv2.threshold(image_gray, treshold_val, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        _, contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # #
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            # cv2.drawContours(frame,[approx],0,(255,255,255),5)

            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            x, y, w, h = cv2.boundingRect(approx)
            self.pub_w.publish(float(w))

            self.pub_h.publish(float(h))

            if w > 10 and h > 58:

                if len(approx) >= 0:
                    tiang_point = Point()
                    aspectRatio = float(w) / h
                    pos_x = (cX / lebar) * 2 - 1
                    pos_y = (cY / tinggi) * 2 - 1

                    tiang_point.x = pos_x
                    tiang_point.y = pos_y

                    self.tiang_x_y.publish(tiang_point)


                    # w_tengah = w*50/100
                    # print(w,"  " ,w_tengah)
                    # print("AspectRatio " + str(aspectRatio))
                    self.aspectRatio.publish(str(aspectRatio))
                    # time.sleep(2.0)

                    # if aspectRatio >= 0.95 and aspectRatio <= 1.05:
                    if aspectRatio >= 0.01:

                        # cv2.putText(frame, "Square", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0))
                        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                        # cv2.rectangle(frame, (x, y), (21, 11), (0, 0, 255), 2)
                        # cv2.
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
                        # print("SIZE TIANG: WIdth {} -  Heigth {}".format(w- x, h-y))
                        # print("X: {} - Y: {}".format(pos_x, pos_y))


        cv2.imshow('Shape image', frame)
        cv2.imshow('Image Mask', mask)
        cv2.imshow("out", output)
        # # cv2.imshow('Shape image', img_clone)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # self.mask_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))

        except CvBridgeError as e:
            print(e)


# ball_detector()
rospy.init_node('processing_image', anonymous=True)



if __name__ == '__main__':
    try:
        objTiang = DeteksiTiang()
        # pubCircle = rospy.Publisher("/ball_detector_node/circle_set", CircleSetStamped, queue_size=1)
        # time.sleep(2.0)
        rospy.spin()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass


