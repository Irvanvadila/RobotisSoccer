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

hsvLow = (0, 115, 92)
hsvMax = (55, 181, 172)
treshold_val = 0
contours = []


class Deteksibase:

    def __init__(self):
        global  hsvLow, hsvMax
        self.image_pub = rospy.Publisher("KRSBI/image/deteksi_base/image_out", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("usb_cam_node/image_raw", Image, self.callback)

        self.mask_pub = rospy.Publisher("KRSBI/image/deteksi_base/mask_out", Image, queue_size=10)

        self.base_x_y = rospy.Publisher("KRSBI/image/deteksi_base/koordinat/x_y", Point, queue_size=10)

        self.pub_w = rospy.Publisher("KRSBI/image/deteksi_base/koordinat/w", Float64, queue_size=10)
        self.pub_h = rospy.Publisher("KRSBI/image/deteksi_base/koordinat/h", Float64, queue_size=10)

        self.pub_w_lacth = rospy.Publisher("KRSBI/image/deteksi_base/koordinat/w_lacth", Float64, queue_size=10)
        self.pub_h_lacth = rospy.Publisher("KRSBI/image/deteksi_base/koordinat/h_lacth", Float64, queue_size=10)

        self.rate = rospy.Rate(30)

        self.aspectRatio = rospy.Publisher("KRSBI/image/deteksi_base/koordinat/aspectRatio", String, queue_size=10)
        self.hsv = rospy.Publisher("KRSBI/image/deteksi_base/image_param/hsv", Twist, queue_size=10)
        self.hsv_sub = rospy.Subscriber("KRSBI/image/deteksi_base/image_param/hsv", Twist, self.hsvcallback)
        # self.image_sub = rospy.Subscriber("usb_cam_node_node/image_raw", Image, self.callback)

        self.state = rospy.Publisher("KRSBI/image/deteksi_base/image/state", String, queue_size=10)

        if rospy.has_param("/base_h_low"):

            hsvLow = (rospy.get_param("/base_h_low"), rospy.get_param("/base_s_low"), rospy.get_param("/base_v_low"))
            hsvMax = (rospy.get_param("/base_h_up"), rospy.get_param("/base_s_up"), rospy.get_param("/base_v_up"))

        else:
            hsvLow = (0, 115, 92)
            hsvMax = (55, 181, 172)
        # rosparam.set_param('/usb_cam_node_node/brightness', '10')
        # rosparam.set_param('/usb_cam_node_node/exposure', '10')
        self.i = 0
        # self.ParamUpdate()
        self.bridge = CvBridge()

    def state_base(self, status):
        if status == 1:
            self.state.publish("OK")
        elif status == 0:
            self.state.publish("NOTDETECT")

    def hsvcallback(self, data):
        rosparam.set_param('/base_h_low', str(data.linear.x))
        rosparam.set_param('/base_s_low', str(data.linear.y))
        rosparam.set_param('/base_v_low', str(data.linear.z))
        rosparam.set_param('/base_h_up', str(data.angular.x))
        rosparam.set_param('/base_s_up', str(data.angular.y))
        rosparam.set_param('/base_v_up', str(data.angular.z))

    def callback(self, data):
        global hsvLow, hsvMax

        # rosparam.set_param('/usb_cam_node/brightness', '10')
        # rosparam.set_param('/usb_cam_node/exposure', '9')

        hsvMsg = Twist()
        if rospy.has_param("/base_h_low"):

            hsvMsg.linear.x = rospy.get_param("/base_h_low")
            hsvMsg.linear.y = rospy.get_param("/base_s_low")
            hsvMsg.linear.z = rospy.get_param("/base_v_low")
            hsvMsg.angular.x = rospy.get_param("/base_h_up")
            hsvMsg.angular.y = rospy.get_param("/base_s_up")
            hsvMsg.angular.z = rospy.get_param("/base_v_up")

        else:
            hsvMsg.linear.x = 0
            hsvMsg.linear.y = 115
            hsvMsg.linear.z = 92
            hsvMsg.angular.x = 55
            hsvMsg.angular.y = 181
            hsvMsg.angular.z = 172

        self.hsv.publish(hsvMsg)

        hsvLow = np.array([hsvMsg.linear.x, hsvMsg.linear.y, hsvMsg.linear.z])
        hsvMax = np.array([hsvMsg.angular.x, hsvMsg.angular.y, hsvMsg.angular.z])

        try:
            # encoding ke blue green red 8 bit
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Resize Frame
        lebar = 640
        tinggi = 360
        frame = imutils.resize(frame, width=640)


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
        self.state_base(0)
        # #
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            cv2.drawContours(frame,[approx],0,(0,255, 0),5)

            M = cv2.moments(contour)
            # cX = int(M["m10"] / M["m00"])
            # cY = int(M["m01"] / M["m00"])

            center = (int(M["m10"] / M["m00"]),
                      int(M["m01"] / M["m00"]))

            x, y, w, h = cv2.boundingRect(approx)


            # print(cX,cY)


            if w > 5 and h > 0.1:

                if len(approx) >= 0:

                    base_point = Point()

                    # pos_x = (x / lebar) * 2 - 1
                    # pos_y = (y / tinggi) * 2 - 1
                    pos_x = ((float(M["m10"] / M["m00"]) / lebar) * 2 - 1) * 1
                    pos_y = ((float(M["m01"] / M["m00"]) / tinggi) * 2 - 1) * -1

                    # print(lebar, tinggi)
                    # print("\nX: {} - Y: {}".format(pos_x, pos_y))

                    aspectRatio = float(w) / h

                    base_point.x = pos_x
                    base_point.y = pos_y
                    base_point.z = self.i

                    self.base_x_y.publish(base_point)

                    # w_tengah = w*50/100
                    # print(w,"  " ,w_tengah)
                    # print("AspectRatio " + str(aspectRatio))
                    self.aspectRatio.publish(str(aspectRatio))
                    # time.sleep(2.0)

                    # if aspectRatio >= 0.95 and aspectRatio <= 1.05:
                    if aspectRatio >= 0.01:
                        self.state_base(1)
                        # cv2.putText(frame, "Square", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0))
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        # cv2.rectangle(frame, (x, y), (21, 11), (0, 0, 255), 2)
                        # cv2.
                        self.pub_w.publish(float(w))

                        self.pub_h.publish(float(h))

                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
                        # print("SIZE base: WIdth {} -  Heigth {}".format(w- x, h-y))
                        # print("X: {} - Y: {}".format(pos_x, pos_y))
                    else:
                        self.state_base(0)
                else: self.state_base(0)

            else:
                self.state_base(0)

        cv2.imshow('Deteksi BASE', frame)
        # cv2.imshow('Image Mask', mask)
        # cv2.imshow("out", output)
        # # cv2.imshow('Shape image', img_clone)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))

        except CvBridgeError as e:
            print(e)

        self.rate.sleep()

# ball_detector()


rospy.init_node('processing_image_base', anonymous=True)

if __name__ == '__main__':
    try:

        objbase = Deteksibase()
        # pubCircle = rospy.Publisher("/ball_detector_node/circle_set", CircleSetStamped, queue_size=1)
        # time.sleep(2.0)
        rospy.spin()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass


