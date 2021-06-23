import rospy
import math
from op3_ball_detector.msg import CircleSetStamped
from std_msgs.msg import String
from op3_walking_module_msgs.msg import WalkingParam
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from robotis_controller_msgs.msg import SyncWriteItem
from geometry_msgs.msg import Vector3

ballState = ""
ballPos = CircleSetStamped()
poleState = ""
polePos = Point()
poleWidth = Float64()
systemState = "STANDBY"
taskSequence = "NONE"
imuData = Imu()
orientData = Vector3()

def ballState_Callback(data):
    global ballState
    ballState = data.data

def ballCoord_Callback(data):
    global ballPos
    ballPos = data

def poleState_Callback(data):
    global poleState
    poleState = data.data

def poleCoord_Callback(data):
    global polePos
    polePos = data

def poleWidth_Callback(data):
    global poleWidth
    poleWidth = data.data

def opencrButton_Callback(data):
    global systemState
    if data.data == "mode":
        systemState = 'PRE_STOP'
    elif data.data == "start":
        systemState = 'PRE_RUN'
    elif data.data == 'user':
        rospy.loginfo('KILLING')
        LED_RGB(0, 0, 0)
        LED_Status(1, 0, 0)
        if systemState != 'STOP':
            Motion_InitPose()
        systemState = 'KILL'
        LED_Status(0, 0, 0)
        exit()

zInt = 0
def opencrImu_Callback(data):
    global imuData
    global orientData
    global zInt
    imuData = data
    # orientData = quaternion_to_euler(imuData)
    # print(orientData.z)
    zInt += imuData.angular_velocity.z

pub_ManCommand = rospy.Publisher("/KRSBI/Manuvering/Command", String, queue_size = 1)
pub_ManMotionMode = rospy.Publisher("/KRSBI/Manuvering/MotionMode", String, queue_size = 1)
pub_ManWalkingParams = rospy.Publisher("/KRSBI/Manuvering/WalkingParams", WalkingParam, queue_size = 1)
pub_ManActionNum = rospy.Publisher("/KRSBI/Manuvering/ActionNum", Int32, queue_size = 1)
pub_ManHeadPan = rospy.Publisher("/KRSBI/Manuvering/HeadPan", Float64, queue_size=1)
pub_ManHeadTilt = rospy.Publisher("/KRSBI/Manuvering/HeadTilt", Float64, queue_size=1)
pub_ManHeadScan = rospy.Publisher("/KRSBI/Manuvering/HeadScan", Bool, queue_size=1)
pub_SyncWrite = rospy.Publisher("/robotis/sync_write_item", SyncWriteItem, queue_size=1)

def Init():
    rospy.init_node("TaskControl", anonymous = False)
    rospy.Subscriber("/KRSBI/processing_image/deteksi_bola/state", String, ballState_Callback)
    rospy.Subscriber("/KRSBI/processing_image/deteksi_bola/coordinate/bola", CircleSetStamped, ballCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/image/state", String, poleState_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/koordinat/x_y", Point, poleCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/koordinat/w", Float64, poleWidth_Callback)
    rospy.Subscriber("/robotis/open_cr/button", String, opencrButton_Callback)
    rospy.Subscriber("/robotis/open_cr/imu", Imu, opencrImu_Callback)
    time.sleep(1)

def MotionMode(mode):
    data = String()
    data.data = mode
    pub_ManMotionMode.publish(data)
    time.sleep(0.1)
def Motion_InitWalking():
    MotionMode('walk')
    time.sleep(3.5)
def Motion_InitAction():
    MotionMode('action')
    time.sleep(0.5)
def Motion_InitHead():
    MotionMode('head')
    time.sleep(0.1)

def MotionCommand(cmd):
    data = String()
    data.data = cmd
    pub_ManCommand.publish(data)
    time.sleep(0.1)
def Motion_InitPose():
    MotionCommand('reset')
    time.sleep(4)
def Motion_Start():
    MotionCommand('start')
def Motion_Stop():
    MotionCommand('stop')
    time.sleep(0.5)

def Motion_WalkingParams(x, y, o, t):
    walkingParams_data = WalkingParam()
    walkingParams_data.x_move_amplitude = x
    walkingParams_data.y_move_amplitude = y
    walkingParams_data.angle_move_amplitude = o
    walkingParams_data.period_time = t
    pub_ManWalkingParams.publish(walkingParams_data)
    time.sleep(0.1)

def Motion_ActionNum(num):
    pub_ManActionNum.publish(num)
    time.sleep(3)

def Motion_HeadScan(enable):
    pub_ManHeadScan.publish(enable)
    time.sleep(0.1)
def Motion_HeadControl(pan, tilt):
    pub_ManHeadPan.publish(pan)
    pub_ManHeadTilt.publish(tilt)
    time.sleep(0.1)

def LED_Status(r, g, b):
    data = SyncWriteItem()
    data.joint_name = ['open-cr']
    data.item_name = 'LED'
    data.value = [r + g*2 + b*4]
    pub_SyncWrite.publish(data)
    time.sleep(0.01)

def LED_RGB(r, g, b):
    data = SyncWriteItem()
    data.joint_name = ['open-cr']
    data.item_name = 'LED_RGB'
    data.value = [(r) | (g << 5) | (b << 10)]
    pub_SyncWrite.publish(data)
    time.sleep(0.01)

def Ticks():
    return int(time.time()*1000)

action_stand = 1
action_keeper = 60
action_standInit = 80
action_sit = 15
action_defenseRight = 61
action_defenseLeft = 62
action_getUpFront = 122
action_getUpBack = 123

taskTimer = 0

if __name__ == '__main__':
    try:
        poleSetPoint = -0.4
        stepSpeed_x = 0.01
        headPan = 0.00
        headTilt = 0.00 #0.00
        Init()
        LED_Status(1, 0, 0)
        LED_RGB(0, 0, 0)
        rospy.loginfo('Node Initialized')

        yawOffset = 0
        yawErrorIntegrator = 0
        yawErrorLast = 0

        while True:
            yawError = (zInt - yawOffset) * -1
            #Kp = 5.0
            Kp = 5.0
            Ki = 0.05
            Kd = 0.0
            if systemState == 'PRE_RUN':
                rospy.loginfo('Starting')
                LED_Status(0, 1, 0)
                Motion_InitWalking()
                Motion_WalkingParams(0.00, 0, 0, 0.45)
                Motion_Start()
                yawOffset = zInt
                systemState = 'RUN'
                taskSequence = 'FORWARD'
                rospy.loginfo('RUN')
            elif systemState == 'RUN':
                u = (yawError * Kp) + (yawErrorIntegrator * Ki) + (yawErrorLast-yawError)*Kd
                yawErrorIntegrator += yawError
                yawErrorLast = yawError
                Motion_WalkingParams(0.02, 0, u, 0.45)
                print(u)
                # Orientation Checker
                if abs(imuData.linear_acceleration.x) > 8.0:
                    systemState = 'RECOVERY'
                    LED_RGB(31, 0, 0)
            elif systemState == 'PRE_STOP':
                LED_RGB(0, 0, 0)
                LED_Status(1, 0, 0)
                rospy.loginfo('Reset Position')
                Motion_InitPose()
                systemState = 'STOP'
                rospy.loginfo('STOP')
            elif systemState == 'RECOVERY':
                rospy.loginfo('Recovery')
                Motion_Stop()
                Motion_InitAction()
                if imuData.linear_acceleration.x > 0:
                    Motion_ActionNum(action_getUpBack)
                elif imuData.linear_acceleration.x < 0:
                    Motion_ActionNum(action_getUpFront)
                Motion_InitWalking()
                Motion_WalkingParams(0, 0, 0, 0.45)
                Motion_Start()
                systemState = 'RUN'
            elif systemState == 'STOP':()
            elif systemState == 'KILL':
                LED_Status(0, 0, 0)
                exit()
    except rospy.ROSInterruptException():
        pass