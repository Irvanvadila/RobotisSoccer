#!/usr/bin/python
import time
import rospy
import math
from op3_walking_module_msgs.msg import WalkingParam
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu


# Macros
TASKSTA_STARTUP = "startup"
TASKSTA_ERROR = "error"
TASKSTA_ESTOP = "estop"
TASKSTA_STOP = "stop"
TASKSTA_STANDBY = "standby"
TASKSTA_RUN = "run"

MTNMODE_NONE = "none"
MTNMODE_WALK = "walk"
MTNMODE_ACTION = "action"
MTNMODE_HEAD = "head"

# Variables
motionMode = MTNMODE_NONE
taskStatus = TASKSTA_STARTUP

# Task Handlers
def eStop_Handler():
    taskStatus = TASKSTA_ESTOP
    WalkingCommand("stop")
    InitPose()

# Callbacks
def eStop_Callback(data):
    eStop_Handler()

def MotionMode_Callback(data):
    global taskStatus
    global motionMode
    if data.data == MTNMODE_WALK:
        if taskStatus == TASKSTA_STANDBY or taskStatus == TASKSTA_STOP:
            EnableCtrlModule("walking_module")
            time.sleep(2)
            rospy.loginfo("Walking Module Enabled")
            motionMode = MTNMODE_WALK
            taskStatus = TASKSTA_STOP
    elif data.data == MTNMODE_ACTION:
        if taskStatus == TASKSTA_STANDBY or taskStatus == TASKSTA_STOP:
            EnableCtrlModule("action_module")
            time.sleep(0.1)
            rospy.loginfo("Action Module Enabled")
            motionMode = MTNMODE_ACTION
            taskStatus = TASKSTA_STOP
    elif data.data == MTNMODE_HEAD:
        EnableCtrlModule("head_control_module")
        time.sleep(0.1)
        rospy.loginfo("Head Control Module Enabled")
    pub_TaskStatus.publish(taskStatus)

def Command_Callback(data):
    global taskStatus
    global motionMode
    if data.data == "start":
        if taskStatus == TASKSTA_STOP:
            if motionMode == MTNMODE_WALK:
                WalkingCommand("start")
                rospy.loginfo("Walking Started")
            taskStatus = TASKSTA_RUN
    elif data.data == "stop":
        if motionMode == MTNMODE_WALK:
            WalkingCommand("stop")
            rospy.loginfo("Walking Stopped")
            taskStatus = TASKSTA_STOP
    elif data.data == "reset":
        if taskStatus == TASKSTA_STOP:
            InitPose()
            rospy.loginfo("Reset to Init Pose")
        elif taskStatus == TASKSTA_RUN:
            WalkingCommand("stop")
            rospy.loginfo("Walking Stopped")
            InitPose()
            rospy.loginfo("Reset to Init Pose")
        taskStatus = TASKSTA_STANDBY
    pub_TaskStatus.publish(taskStatus)

def WalkingParams_Callback(data):
    data.angle_move_amplitude = data.angle_move_amplitude * math.pi / 180
    Walking(data.x_move_amplitude, data.y_move_amplitude,
            data.angle_move_amplitude, data.period_time)

def ActionNum_Callback(data):
    pub_PageNum.publish(data)
    time.sleep(0.1)
    rospy.loginfo("Action Start")

headJointData = JointState()
headJointData.name = ['head_pan', 'head_tilt']
headJointData.position = [0, 0]
headPubUpdate = False
def HeadPan_Callback(data):
    global headJointData
    global  headPubUpdate
    headPubUpdate = True
    if data.data > 1.5:
        headJointData.position = [1.5]
    elif data.data < -1.5:
        headJointData.position = [-1.5]
    else:
        headJointData.position[0] = data.data

def HeadTilt_Callback(data):
    global headJointData
    global  headPubUpdate
    headPubUpdate = True
    if data.data > 1.0:
        headJointData.position = [1.0]
    elif data.data < -1.2:
        headJointData.position = [-1.2]
    else:
        headJointData.position[1] = data.data

def HeadScan_Callback(data):
    if data.data:
        pub_HeadScan.publish('scan')
        rospy.loginfo("Head Scan Enabled")
    else:
        pub_HeadScan.publish('stop')
        rospy.loginfo("Head Scan Disabled")
    time.sleep(0.1)

gyroYawIntegrator = 0
def opencrImu_Callback(data):
    global gyroYawIntegrator
    imuData = data
    gyroYawIntegrator += imuData.angular_velocity.z

# Publishers
pub_WalkingParams = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=1)
pub_WalkingCommand = rospy.Publisher("/robotis/walking/command", String, queue_size=1)
pub_EnableCtrlModule = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=1)
pub_InitPose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=1)
pub_PageNum = rospy.Publisher("/robotis/action/page_num", Int32, queue_size=1)
pub_HeadJoint = rospy.Publisher("robotis/head_control/set_joint_states", JointState, queue_size=1)
pub_HeadScan = rospy.Publisher("robotis/head_control/scan_command", String, queue_size=1)
pub_TaskStatus = rospy.Publisher("/KRSBI/Manuvering/Status", String, queue_size=1)
pub_YawPos = rospy.Publisher("/KRSBI/Manuvering/Sensor/Gyro/Yaw", Float64, queue_size = 1)

def Init():
    rospy.init_node('Manuvering_MotionDriver', anonymous=False)
    time.sleep(1)
    rospy.Rate(50)

    rospy.Subscriber("/KRSBI/Manuvering/eStop", Bool, eStop_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/MotionMode", String, MotionMode_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/Command", String, Command_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/WalkingParams", WalkingParam, WalkingParams_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/ActionNum", Int32, ActionNum_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/HeadPan", Float64, HeadPan_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/HeadTilt", Float64, HeadTilt_Callback)
    rospy.Subscriber("/KRSBI/Manuvering/HeadScan", Bool, HeadScan_Callback)
    rospy.Subscriber("/robotis/open_cr/imu", Imu, opencrImu_Callback)

def EnableCtrlModule(Module):
    pub_EnableCtrlModule.publish(Module)
    time.sleep(0.1)

def InitPose():
    pub_InitPose.publish('ini_pose')
    time.sleep(4)

WalkingParam_msg = WalkingParam()
walkingPubUpdate = False
def Walking(move_x, move_y, move_yaw, period):
    global WalkingParam_msg
    global walkingPubUpdate
    WalkingParam_msg.init_x_offset = -0.02
    WalkingParam_msg.init_y_offset = 0.014
    WalkingParam_msg.init_z_offset = 0.032
    WalkingParam_msg.init_roll_offset = -0.0015
    WalkingParam_msg.init_pitch_offset = 0.0
    WalkingParam_msg.init_yaw_offset = 0.0
    WalkingParam_msg.hip_pitch_offset = 0.12
    WalkingParam_msg.period_time = period
    WalkingParam_msg.dsp_ratio = 0.2
    WalkingParam_msg.step_fb_ratio = 0.3
    WalkingParam_msg.x_move_amplitude = move_x
    WalkingParam_msg.y_move_amplitude = move_y
    WalkingParam_msg.z_move_amplitude = 0.09
    WalkingParam_msg.angle_move_amplitude = (move_yaw+-0.5) * math.pi/180
    WalkingParam_msg.move_aim_on = False
    WalkingParam_msg.balance_enable = True
    WalkingParam_msg.balance_hip_roll_gain = 0.30
    WalkingParam_msg.balance_knee_gain = 0.65
    WalkingParam_msg.balance_ankle_roll_gain = 1.0
    WalkingParam_msg.balance_ankle_pitch_gain = 0.0
    WalkingParam_msg.y_swap_amplitude = 0.020
    WalkingParam_msg.z_swap_amplitude = 0.010
    WalkingParam_msg.arm_swing_gain = 0.25
    WalkingParam_msg.pelvis_offset = 0.009
    WalkingParam_msg.p_gain = 15
    WalkingParam_msg.i_gain = 1
    WalkingParam_msg.d_gain = 5
    walkingPubUpdate = True

def WalkingCommand(data):
    pub_WalkingCommand.publish(data)
    time.sleep(0.1)

def Ticks():
    return int(time.time()*1000)

taskTimer = 0

if __name__ == '__main__':
    try:
        Init()
        pub_TaskStatus.publish(taskStatus)
        rospy.loginfo('ROSnode Init')
        taskStatus = TASKSTA_STANDBY
        pub_TaskStatus.publish(taskStatus)

        while not rospy.is_shutdown():
                if walkingPubUpdate:
                    walkingPubUpdate = False
                    pub_WalkingParams.publish(WalkingParam_msg)
                    rospy.loginfo("Walking Parameters Set")
                    time.sleep(0.1)
                if headPubUpdate:
                    headPubUpdate = False
                    pub_HeadJoint.publish(headJointData)
                    rospy.loginfo("Head Control")
                    time.sleep(0.1)
                if Ticks() - taskTimer >= 100:
                    taskTimer = Ticks()
                    pub_YawPos.publish(gyroYawIntegrator)


    except rospy.ROSInterruptException():
        pass

