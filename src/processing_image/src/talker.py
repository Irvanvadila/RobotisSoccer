#!/usr/bin/python
import rospy
# import numpy as np
# import imutils
# import sys
#
from std_msgs.msg import String
# from std_msgs.msg import Float64
# from std_msgs.msg import Int32
# from std_msgs.msg import Bool
#
# import roslib
# roslib.load_manifest('processing_image')
#
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# # from op3_ball_detector.msg import CircleSetStamped
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Twist
#
# # from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world"
        # rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass