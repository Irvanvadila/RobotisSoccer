import time
import rospy
import math
from op3_walking_module_msgs.msg import WalkingParam
from std_msgs.msg import String


pub_WalkingParams = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=1)
pub_WalkingCommand = rospy.Publisher("/robotis/walking/command", String, queue_size=1)
pub_EnableCtrlModule = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=1)
pub_InitPose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=1)


def Init():
    rospy.init_node('Test', anonymous=False)
    time.sleep(1)
    rospy.Rate(50)

def EnableCtrlModule(Module):
    pub_EnableCtrlModule.publish(Module)
    time.sleep(0.1)

def InitPose():
    pub_InitPose.publish('ini_pose')
    time.sleep(4)

def Walking(move_x, move_y, move_yaw, period):
    WalkingParam_msg = WalkingParam()
    WalkingParam_msg.init_x_offset = -0.01
    WalkingParam_msg.init_y_offset = 0.015
    WalkingParam_msg.init_z_offset = 0.035
    WalkingParam_msg.init_roll_offset = 0.0
    WalkingParam_msg.init_pitch_offset = 0.0
    WalkingParam_msg.init_yaw_offset = 0.0
    WalkingParam_msg.hip_pitch_offset = 0.12
    WalkingParam_msg.period_time = period
    WalkingParam_msg.dsp_ratio = 0.2
    WalkingParam_msg.step_fb_ratio = 0.3
    WalkingParam_msg.x_move_amplitude = move_x
    WalkingParam_msg.y_move_amplitude = move_y
    WalkingParam_msg.z_move_amplitude = 0.06
    WalkingParam_msg.angle_move_amplitude = move_yaw * math.pi/180
    WalkingParam_msg.move_aim_on = False
    WalkingParam_msg.balance_enable = True
    WalkingParam_msg.balance_hip_roll_gain = 0.35
    WalkingParam_msg.balance_knee_gain = 0.5
    WalkingParam_msg.balance_ankle_roll_gain = 0.7
    WalkingParam_msg.balance_ankle_pitch_gain = 0.9
    WalkingParam_msg.y_swap_amplitude = 0.030
    WalkingParam_msg.z_swap_amplitude = 0.008
    WalkingParam_msg.arm_swing_gain = 0.20
    WalkingParam_msg.pelvis_offset = 0.009
    WalkingParam_msg.p_gain = 0
    WalkingParam_msg.i_gain = 0
    WalkingParam_msg.d_gain = 0
    pub_WalkingParams.publish(WalkingParam_msg)
    time.sleep(0.1)

def WalkingCommand(data):
    pub_WalkingCommand.publish(data)
    time.sleep(0.1)


if __name__ == '__main__':
    try:
        Init()
        rospy.loginfo('ROSnode Init')
        EnableCtrlModule("walking_module")
        rospy.loginfo('Walking Module Enabled')
        time.sleep(2)
        rospy.loginfo('Move Forward')
        Walking(0.040, 0.0, -1.0, 0.55)
        WalkingCommand("start")
        time.sleep(3)
        rospy.loginfo('Turn Right')
        Walking(0.03, 0.0, -15.0, 0.50)
        #time.sleep(14.5)
        time.sleep(5)
        rospy.loginfo('Move Forward')
        Walking(0.040, 0.0, -1.0, 0.55)
        time.sleep(3)
        rospy.loginfo('Stop')
        WalkingCommand("stop")
        rospy.loginfo('Init Pose')
        InitPose()

    except rospy.ROSInterruptException():
        pass

