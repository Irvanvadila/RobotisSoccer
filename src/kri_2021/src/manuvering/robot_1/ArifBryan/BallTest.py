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
import matplotlib.pyplot as plt
import numpy as np

ballState = ""
ballPos = Point()
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


def Quat2Euler(imu):
    w = imu.orientation.w
    x = imu.orientation.x
    y = imu.orientation.y
    z = imu.orientation.z

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    pitch = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2 > +1.0, +1.0, t2)
    # t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2 < -1.0, -1.0, t2)
    # t2 = -1.0 if t2 < -1.0 else t2
    roll = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = np.degrees(np.arctan2(t3, t4))

    return yaw, pitch, roll

def opencrImu_Callback(data):
    global imuData
    global orientData
    imuData = data
    orientData = Quat2Euler(imuData)

yawPosition = 0
def yawPosition_Callback(data):
    global yawPosition
    yawPosition = data.data

pub_ManCommand = rospy.Publisher("/KRSBI/Manuvering/Command", String, queue_size=1)
pub_ManMotionMode = rospy.Publisher("/KRSBI/Manuvering/MotionMode", String, queue_size=1)
pub_ManWalkingParams = rospy.Publisher("/KRSBI/Manuvering/WalkingParams", WalkingParam, queue_size=1)
pub_ManActionNum = rospy.Publisher("/KRSBI/Manuvering/ActionNum", Int32, queue_size=1)
pub_ManHeadPan = rospy.Publisher("/KRSBI/Manuvering/HeadPan", Float64, queue_size=1)
pub_ManHeadTilt = rospy.Publisher("/KRSBI/Manuvering/HeadTilt", Float64, queue_size=1)
pub_ManHeadScan = rospy.Publisher("/KRSBI/Manuvering/HeadScan", Bool, queue_size=1)
pub_SyncWrite = rospy.Publisher("/robotis/sync_write_item", SyncWriteItem, queue_size=1)


def Init():
    rospy.init_node("TaskControl", anonymous=False)
    rospy.Subscriber("/KRSBI/deteksi_bola/state", String, ballState_Callback)
    rospy.Subscriber("/KRSBI/deteksi_bola/koordinat/bola", Point, ballCoord_Callback)
    rospy.Subscriber("/KRSBI/deteksi_tiang/image/state", String, poleState_Callback)
    rospy.Subscriber("/KRSBI/deteksi_tiang/koordinat/x_y", Point, poleCoord_Callback)
    rospy.Subscriber("/KRSBI/deteksi_tiang/koordinat/w", Float64, poleWidth_Callback)
    rospy.Subscriber("/KRSBI/deteksi_tiang/koordinat/h", Float64, poleHeight_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/image/state", String, baseState_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/x_y", Point, baseCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/w", Float64, baseWidth_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/h", Float64, baseHeight_Callback)
    rospy.Subscriber("/robotis/open_cr/button", String, opencrButton_Callback)
    rospy.Subscriber("/robotis/open_cr/imu", Imu, opencrImu_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/Sensor/Gyro/Yaw", Float64, yawPosition_Callback)
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
    time.sleep(0.01)


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
    data.value = [r + g * 2 + b * 4]
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
    return int(time.time() * 1000)


class PID:
    def __init__(self):
        self.u = 0.0
        self.sp = 0.0
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.errorIntegrator = 0.0
        self.errorLast = 0.0

    def setPoint(self, sp):
        self.sp = sp

    def setPID(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calc(self, av):
        e = self.sp - av
        self.u = (e * self.kp) + (self.errorIntegrator * self.ki) + (e - self.errorLast) * self.kd
        self.errorIntegrator += e
        self.errorLast = e
        return (self.u)

    def reset(self):
        self.errorIntegrator = 0.0
        self.errorLast = 0.0
        self.u = 0.0


class ExpFilter:
    def __init__(self):
        self.out = 0.0
        self.kf = 0.0

    def setKf(self, kf):
        self.kf = kf

    def calc(self, val):
        self.out = ((self.out * self.kf) + val) / (self.kf + 1)
        return (self.out)

    def reset(self, set):
        self.out = set

def limit(val, min, max):
    if val < min:
        val = min
    elif val > max:
        val = max
    return val

yawFilter = ExpFilter()
polePosFilter = ExpFilter()
poleSizeFilter = ExpFilter()
basePosFilter = ExpFilter()
baseSizeFilter = ExpFilter()
ballPosxFilter = ExpFilter()
ballPosyFilter = ExpFilter()
ballSizeFilter = ExpFilter()

yawPID = PID()
polePosPID = PID()
basePosPID = PID()
ballPosxPID = PID()
ballPosyPID = PID()

headTilt = 0

def Motion_Kick(num):
    global headTilt
    Motion_Stop()
    Motion_InitAction()
    Motion_ActionNum(num)
    Motion_InitWalking()
    Motion_WalkingParams(0, 0, 0, 0.45)
    Motion_InitHead()
    Motion_HeadControl(0.0, headTilt)

action_stand = 1
action_keeper = 60
action_standInit = 80
action_sit = 15
action_defenseRight = 61
action_defenseLeft = 62
action_getUpFront = 122
action_getUpBack = 123
action_kickL1 = 90
action_kickR1 = 91
action_kickL2 = 92
action_kickR2 = 93
action_kickL3 = 94
action_kickR3 = 107

taskTimer = 0
buzzerTimer = 0

error_t = []
errorFiltered_t = []
u_t = []
sp_t = []
t_t = []
t = 0

if __name__ == '__main__':
    try:
        # Yaw Correction
        yawFilter.setKf(15)
        yawPID.setPoint(0.0)
        yawPID.setPID(15.0, 0.01, 10.0)
        # Pole Tracking
        polePosSetpoint = 0.03
        poleDistStop = 10000
        poleDistComp = 10
        polePosFilter.setKf(20)
        poleSizeFilter.setKf(25)
        polePosPID.setPoint(0)
        # default 10.0/100, 0.01/100, 0.0/100
        polePosPID.setPID(10.0 / 100, 0.00 / 100, 0.0 / 100)  # KP #KI #KD NORMALISASI = PER 100
        # Base Tracking
        basePosFilter.setKf(100)
        baseSizeFilter.setKf(10)
        basePosPID.setPoint(0.0)
        basePosPID.setPID(0.00, 0.0, 0.0)
        errComp = 80
        # Ball Tracking
        ballPosxFilter.setKf(50)
        ballPosyFilter.setKf(30)
        ballSizeFilter.setKf(30)

        ballPosxPID.setPID(0.05, 0.0, 1.0)
        ballPosyPID.setPID(0.01, 0.001, 1.0)

        ballPosxPID.setPoint(0.0)
        ballPosyPID.setPoint(0.5)

        stepSpeed_x = 0.020
        stepPeriod = 0.60
        headPan = 0
        headTiltForward = -0.7
        headTiltReturn = 0.0
        headTilt = headTiltForward

        yawOffset = 0

        Init()
        rospy.loginfo('Bismillah Initialized')
        rospy.loginfo('Checking Gyroscope...')
        yawOffset = yawPosition
        # time.sleep(2)
        if abs(yawPosition - yawOffset) > 1:
            rospy.loginfo('GYROSOCOPE NOT STABLE!')
            # exit()
        else:
            rospy.loginfo('Gyroscope OK')

        while not rospy.is_shutdown():
            yawError = yawPosition - yawOffset
            yawFilter.calc(yawError)
            ballPosxFilter.calc(ballPos.x)
            ballPosyFilter.calc(ballPos.y)
            ballSizeFilter.calc(ballPos.z)

            # error_t.append(orientData[2])
            # t_t.append(t)
            # t += 1

            if systemState == 'PRE_RUN':
                rospy.loginfo('Starting')
                LED_Status(0, 1, 0)
                Motion_WalkingParams(0.0, 0, 0.0, stepPeriod)
                Motion_Start()
                yawOffset = yawPosition
                yawError = yawPosition - yawOffset
                Motion_HeadControl(0.0, headTilt)
                yawFilter.reset(yawError)
                yawPID.reset()
                poleSizeFilter.reset(0)
                polePosFilter.reset(0)
                polePosPID.reset()
                basePosFilter.reset(0)
                baseSizeFilter.reset(0)
                basePosPID.reset()
                systemState = 'RUN'
                taskSequence = 'FORWARD'
                rospy.loginfo('RUN')
                error_t = []
                errorFiltered_t = []
                u_t = []
                sp_t = []
                t_t = []
                t = 0
                taskTimer = Ticks()
                while Ticks() - taskTimer < 200: ()
                taskTimer = Ticks()
            elif systemState == 'RUN':
                yawPID.calc(yawFilter.out)
                if taskSequence == 'FORWARD':
                    poleSizeFilter.calc(poleHeight * poleWidth)
                    polePosPID.setPoint(math.sqrt(poleSizeFilter.out) / poleDistComp * polePosSetpoint)
                    polePosFilter.calc(polePos.x)
                    polePosPID.calc(polePosFilter.out)

                    ballPosxPID.calc(ballPosxFilter.out)
                    ballPosyPID.calc(ballPosyFilter.out)


                    if poleState == "OK":
                        # rospy.loginfo(polePosPID.u)
                        # rospy.loginfo(polePosFilter.out)
                        # xSpeed = stepSpeed_x / (1 + abs(xError / errComp))
                        # xSpeed = xSpeed / (1 + abs(yawError / errComp / 100))
                        # Motion_WalkingParams(xSpeed, uPole, uYaw, stepPeriod)
                        LED_RGB(0, 31, 0)
                    else:
                        LED_RGB(0, 0, 0)
                    if poleSizeFilter.out > poleDistStop:
                        LED_RGB(0, 0, 31)
                        BuzzerTone(1, 500)
                        taskTimer = Ticks()
                        # taskSequence = 'ROTATE'
                        # systemState = 'PRE_STOP'
                    if abs(ballPosyPID.sp - ballPosyFilter.out) < 0.01 and abs(ballPosxPID.sp - ballPosxFilter.out) < 0.02:
                        BuzzerTone(1, 500)
                        taskTimer = Ticks()
                        Motion_Kick(action_kickR3)
                        # taskSequence = 'ROTATE'
                        #systemState = 'PRE_STOP'
                    sp_t.append(polePosPID.sp)
                    error_t.append(polePosFilter.out)
                    u_t.append(polePosPID.u)
                    t_t.append(t)
                    t += 1
                    xSpeed = stepSpeed_x
                    Motion_WalkingParams(xSpeed, limit(ballPosxPID.u, -0.025, 0.025), yawPID.u, stepPeriod)
                    # if Ticks() - taskTimer >= 7000:
                    #     systemState = 'PRE_STOP'
                    #     #taskSequence = 'ROTATE'



                elif taskSequence == 'ROTATE':
                    if Ticks() - taskTimer < 3300:
                        Motion_WalkingParams(stepSpeed_x, 0.000, yawPID.u, stepPeriod)
                    elif Ticks() - taskTimer < 7000:
                        # Rotation
                        Motion_WalkingParams(0.040, 0.00, -950, 0.62)  # default 0.040, 0.00, -950, 0.62
                    else:
                        yawOffset += -420  # default -420
                        basePos.x = 0
                        headTilt = headTiltReturn
                        Motion_HeadControl(0.0, headTilt)
                        basePosFiltered = 0
                        baseSizeFiltered = 0
                        BuzzerTone(1, 500)
                        # taskSequence = 'RETURN'
                        systemState = 'PRE_STOP'
                elif taskSequence == 'RETURN':
                    baseSizeFilter.calc(baseHeight * baseWidth)
                    if baseState == "OK":
                        basePosFilter.calc(basePos.x)
                        basePosPID.calc(basePosFilter.out)
                        # xSpeed = stepSpeed_x / (1 + abs(xError / errComp))
                        # xSpeed = xSpeed / (1 + abs(yawError / errComp / 100))
                        xSpeed = stepSpeed_x
                        Motion_WalkingParams(xSpeed, basePosPID.u, yawPID.u, stepPeriod)
                        LED_RGB(0, 31, 0)
                    else:
                        Motion_WalkingParams(stepSpeed_x, basePosPID.u, yawPID.u, stepPeriod)
                        LED_RGB(0, 0, 0)
                    if baseSizeFilter.out > 30000:
                        LED_RGB(0, 0, 31)
                        taskTimer = Ticks()
                        BuzzerTone(1, 500)
                        taskSequence = 'RETURN_FWD'
                elif taskSequence == 'RETURN_FWD':
                    if Ticks() - taskTimer < 4000:
                        Motion_WalkingParams(stepSpeed_x, 0.000, yawPID.u, stepPeriod)
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
                # Motion_InitPose()
                plt.close()
                plt.plot(t_t, error_t, color='red')
                # plt.plot(t_t, errorFiltered_t, color='orange')
                plt.plot(t_t, u_t, color='blue')
                plt.plot(t_t, sp_t, color='black')
                plt.show(block=False)
                systemState = 'STOP'
                rospy.loginfo('STOP')
            elif systemState == 'RECOVERY':
                rospy.loginfo('Recovery')
                Motion_Stop()
                # Motion_InitAction()
                # if imuData.linear_acceleration.x > 0:
                #     Motion_ActionNum(action_getUpBack)
                # elif imuData.linear_acceleration.x < 0:
                #     Motion_ActionNum(action_getUpFront)
                # Motion_InitWalking()
                # Motion_WalkingParams(0, 0, 0, 0.45)
                # Motion_Start()
                # Motion_InitHead()
                # Motion_HeadControl(0.0, headTilt)
                # systemState = 'RUN'
                systemState = 'PRE_STOP'
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
                LED_RGB(0, 0, 0)
                LED_Status(0, 0, 1)
                Motion_InitWalking()
                Motion_InitHead()
                Buzzer(2000)
                headTilt = headTiltForward
                Motion_HeadControl(0.0, headTilt)
                Buzzer(3000)
                time.sleep(0.1)
                LED_Status(1, 0, 0)
                LED_RGB(0, 0, 0)
                Buzzer(0)
                systemState = 'STANDBY'
            elif systemState == 'STANDBY':
                systemState = 'STOP'
            elif systemState == 'STOP':
                ()

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