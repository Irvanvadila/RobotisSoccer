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
poleWidth = 0.0
poleHeight = 0.0
baseState = ""
basePos = Point()
baseWidth = 0.0
baseHeight = 0.0
systemState = "PRE_STANDBY"
taskSequence = "NONE"
imuData = Imu()
orientData = Vector3()

# Wafak

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

def poleHeight_Callback(data):
    global poleHeight
    poleHeight = data.data

def baseState_Callback(data):
    global baseState
    baseState = data.data

def baseCoord_Callback(data):
    global basePos
    basePos = data

def baseWidth_Callback(data):
    global baseWidth
    baseWidth = data.data

def baseHeight_Callback(data):
    global baseHeight
    baseHeight = data.data

def opencrButton_Callback(data):
    global systemState
    if data.data == "mode":
        if not systemState == 'IDLE':
            systemState = 'PRE_STOP'
            BuzzerTone(1, 250)
    elif data.data == "start":
        if systemState == 'STOP':
            systemState = 'PRE_RUN'
            BuzzerTone(1, 100)
    elif data.data == 'user':
        if systemState == 'STOP':
            systemState = 'PRE_IDLE'
        elif systemState == 'IDLE':
            systemState = 'PRE_STANDBY'

gyroYawIntegrator = 0
def opencrImu_Callback(data):
    global imuData
    global orientData
    global gyroYawIntegrator
    imuData = data
    # orientData = quaternion_to_euler(imuData)
    # print(orientData.z)
    gyroYawIntegrator += imuData.angular_velocity.z

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
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/koordinat/h", Float64, poleHeight_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/image/state", String, baseState_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/x_y", Point, baseCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/w", Float64, baseWidth_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/h", Float64, baseHeight_Callback)
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
    time.sleep(0.3)

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
    time.sleep(0.1)
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

def Buzzer(freq):
    data = SyncWriteItem()
    data.joint_name = ['open-cr']
    data.item_name = 'buzzer'
    data.value = [freq]
    pub_SyncWrite.publish(data)
    time.sleep(0.01)

buzzerCount = 0
buzzerPeriod = 0
buzzerState = 0
def BuzzerTone(count, period):
    global buzzerCount
    global buzzerPeriod
    buzzerCount = count
    buzzerPeriod = period

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
buzzerTimer = 0

if __name__ == '__main__':
    try:
        Init()
        Motion_InitHead()
        Motion_HeadControl(1.0, 1.5)
    except rospy.ROSInterruptException():
        pass
