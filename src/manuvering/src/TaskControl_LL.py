#!/usr/bin/python
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
        Kp = 8.0
        Ki = 0.01
        Kd = 0.0
        KfPoleSize = 20
        KfPolePos = 8
        KfBaseSize = 10
        KfBasePos = 10

        baseSetPoint = 0
        poleSetPoint = -0.4
        stepSpeed_x = 0.040
        stepPeriod = 0.55
        headPan = 0
        headTilt = 0.07

        yawOffset = 0
        yawErrorIntegrator = 0
        yawErrorLast = 0
        polePosFiltered = 0
        poleSizeFiltered = 0
        basePosFiltered = 0
        baseSizeFiltered = 0

        Init()
        rospy.loginfo('Node Initialized')
        rospy.loginfo('Checking Gyroscope...')
        yawOffset = gyroYawIntegrator
        time.sleep(2)
        if abs(gyroYawIntegrator - yawOffset) > 1:
            rospy.loginfo('GYROSOCOPE NOT STABLE!')
            exit()
        else:
            rospy.loginfo('Gyroscope OK')

        while not rospy.is_shutdown():
            yawError = (gyroYawIntegrator - yawOffset) * -1
            if systemState == 'PRE_RUN':
                rospy.loginfo('Starting')
                LED_Status(0, 1, 0)
                Motion_WalkingParams(0.00, 0, 0, 0.45)
                Motion_Start()
                yawOffset = gyroYawIntegrator
                yawErrorLast = 0
                yawErrorIntegrator = 0
                polePosFiltered = 0
                poleSizeFiltered = 0
                headTilt = 0.07
                Motion_HeadControl(0.0, headTilt)
                systemState = 'RUN'
                taskSequence = 'FORWARD'
                rospy.loginfo('RUN')
            elif systemState == 'RUN':
                u = (yawError * Kp) + (yawErrorIntegrator * Ki) + (yawErrorLast - yawError) * Kd
                yawErrorIntegrator += yawError
                yawErrorLast = yawError
                if taskSequence == 'FORWARD':
                    poleSize = poleHeight * poleWidth
                    poleSizeFiltered = (poleSizeFiltered * KfPoleSize) + poleSize
                    poleSizeFiltered /= KfPoleSize + 1
                    polePosFiltered = (polePosFiltered * KfPolePos) + polePos.x
                    polePosFiltered /= KfPolePos + 1
                    if poleState == "OK":
                        xError = (polePosFiltered * -1 - poleSetPoint) / 30
                        xSpeed = stepSpeed_x / (1 + abs(xError))
                        xSpeed = stepSpeed_x / (1 + abs(yawError / 5))
                        Motion_WalkingParams(stepSpeed_x, xError, u, stepPeriod)
                        LED_RGB(0, 31, 0)
                    else:
                        LED_RGB(0, 0, 0)
                    if poleSizeFiltered > 10000:
                        LED_RGB(0, 0, 31)
                        BuzzerTone(1, 500)
                        taskTimer = Ticks()
                        taskSequence = 'ROTATE'
                elif taskSequence == 'ROTATE':
                    if Ticks() - taskTimer < 3000:
                        Motion_WalkingParams(stepSpeed_x, 0.000, u, stepPeriod)
                    elif Ticks() - taskTimer < 9000:
                        Motion_WalkingParams(0.035, 0.00, -500, 0.50)
                    else:
                        yawOffset += -420
                        basePos.x = 0
                        headTilt = -0.15
                        Motion_HeadControl(0.0, headTilt)
                        basePosFiltered = 0
                        baseSizeFiltered = 0
                        BuzzerTone(1, 500)
                        taskSequence = 'RETURN'
                elif taskSequence == 'RETURN':
                    baseSize = baseHeight * baseWidth
                    baseSizeFiltered = (baseSizeFiltered * KfBaseSize) + baseSize
                    baseSizeFiltered /= KfBaseSize + 1
                    basePosFiltered = (basePosFiltered * KfBasePos) + basePos.x
                    basePosFiltered /= KfBasePos + 1
                    if baseState == "OK":
                        xError = (basePosFiltered * -1 - baseSetPoint) / 10
                        xSpeed = stepSpeed_x / (1 + abs(xError))
                        xSpeed = stepSpeed_x / (1 + abs(yawError / 5))
                        Motion_WalkingParams(stepSpeed_x, xError, u, stepPeriod)
                        LED_RGB(0, 31, 0)
                    else:
                        Motion_WalkingParams(stepSpeed_x, xError, u, stepPeriod)
                        LED_RGB(0, 0, 0)
                    if baseSizeFiltered > 30000:
                        LED_RGB(0, 0, 31)
                        taskTimer = Ticks()
                        BuzzerTone(1, 500)
                        taskSequence = 'RETURN_FWD'
                elif taskSequence == 'RETURN_FWD':
                    if Ticks() - taskTimer < 4000:
                        Motion_WalkingParams(stepSpeed_x, 0.000, u, stepPeriod)
                    else:
                        BuzzerTone(3, 100)
                        systemState = 'PRE_STOP'

                # Orientation Checker
                if abs(imuData.linear_acceleration.x) > 8.0:
                    systemState = 'RECOVERY'
                    LED_RGB(31, 0, 0)
            elif systemState == 'PRE_STOP':
                LED_RGB(0, 0, 0)
                LED_Status(1, 0, 0)
                rospy.loginfo('Reset Position')
                Motion_Stop()
                #Motion_InitPose()
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
                Motion_InitHead()
                Motion_HeadControl(0.0, headTilt)
                systemState = 'RUN'
            elif systemState == 'PRE_IDLE':
                Motion_InitPose()
                Buzzer(3000)
                time.sleep(0.2)
                Buzzer(2000)
                time.sleep(0.2)
                Buzzer(0)
                systemState = 'IDLE'
            elif systemState == 'IDLE':
                if Ticks() - taskTimer < 500:
                    LED_Status(1, 0, 0)
                    LED_RGB(0, 0, 0)
                elif Ticks() - taskTimer < 1000:
                    LED_Status(0, 0, 0)
                    LED_RGB(31, 0, 0)
                else:
                    taskTimer = Ticks()
            elif systemState == 'PRE_STANDBY':
                LED_RGB(0,0,0)
                LED_Status(0,0,1)
                Motion_InitWalking()
                Motion_InitHead()
                Buzzer(2000)
                Motion_HeadControl(0.0, headTilt)
                Buzzer(3000)
                time.sleep(0.1)
                LED_Status(1, 0, 0)
                LED_RGB(0, 0, 0)
                Buzzer(0)
                systemState = 'STANDBY'
            elif systemState == 'STANDBY':
                systemState = 'STOP'
            elif systemState == 'STOP':()

            if buzzerCount > 0:
                if Ticks() - buzzerTimer >= buzzerPeriod:
                    buzzerTimer = Ticks()
                    if not buzzerState:
                        Buzzer(3000)
                        buzzerState = 1
                    else:
                        Buzzer(0)
                        buzzerState = 0
                        buzzerCount -= 1
            else:
                buzzerTimer = Ticks()

    except rospy.ROSInterruptException():
        pass
