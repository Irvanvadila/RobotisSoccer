import scipy.signal
import pyaudio
import wave
import sys
import numpy as np
import rospy
import math
import warnings
import time
import matplotlib.pyplot as plt
from scipy.io import wavfile as wav
from scipy.fftpack import fft
from std_msgs.msg import Int8
# from pydub import AudioSegment
# from pydub.playback import play
from op3_ball_detector.msg import CircleSetStamped
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem

# L = 1
# S = 0

listen = 0
listen2 = 0
talking = ""
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

def listen1_Callback(data):
    global listen
    listen = data.data

def listen2_Callback(data):
    global listen2
    listen2 = data.data

def talking1_Callback(data):
    global talking
    talking = data.data

def talking2_Callback(data):
    global talking2
    talking2 = data.data

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

gyroYawIntegrator = 0
def opencrImu_Callback(data):
    global imuData
    global orientData
    global gyroYawIntegrator
    imuData = data
    # orientData = quaternion_to_euler(imuData)
    # print(orientData.z)
    gyroYawIntegrator += imuData.angular_velocity.z

pub_ManWalkingParams = rospy.Publisher("/KRSBI/Manuvering/WalkingParams", WalkingParam, queue_size=1)
pub_WalkingCommand = rospy.Publisher("/robotis/walking/command", String, queue_size=1)
pub_EnableCtrlModule = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=1)
pub_InitPose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=1)
pub_ManActionNum = rospy.Publisher("/KRSBI/Manuvering/ActionNum", Int32, queue_size = 1)
pub_ManHeadPan = rospy.Publisher("/KRSBI/Manuvering/HeadPan", Float64, queue_size=1)
pub_ManHeadTilt = rospy.Publisher("/KRSBI/Manuvering/HeadTilt", Float64, queue_size=1)
pub_ManHeadScan = rospy.Publisher("/KRSBI/Manuvering/HeadScan", Bool, queue_size=1)
pub_SyncWrite = rospy.Publisher("/robotis/sync_write_item", SyncWriteItem, queue_size=1)
pub_ManCommand = rospy.Publisher("/KRSBI/Manuvering/Command", String, queue_size = 1)
pub_ManMotionMode = rospy.Publisher("/KRSBI/Manuvering/MotionMode", String, queue_size = 1)

def Init():
    rospy.init_node('LKR', anonymous=False)
    rospy.Subscriber("/KRSBI/deteksi_bola/state", String, ballState_Callback)
    rospy.Subscriber("/KRSBI/deteksi_bola/koordinat/bola", Vector3, ballCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/image/state", String, poleState_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/koordinat/x_y", Point, poleCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/koordinat/w", Float64, poleWidth_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_tiang/koordinat/h", Float64, poleHeight_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/image/state", String, baseState_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/x_y", Point, baseCoord_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/w", Float64, baseWidth_Callback)
    rospy.Subscriber("/KRSBI/image/deteksi_base/koordinat/h", Float64, baseHeight_Callback)
    rospy.Subscriber("/KRSBI/Signal_Processing/Listening/shot", Int8, listen1_Callback)
    rospy.Subscriber("/KRSBI/Signal_Processing/Listening/pass", Int8, listen2_Callback)
    rospy.Subscriber("/KRSBI/Signal_Processing/Talking/Shot", String, talking1_Callback)
    rospy.Subscriber("/KRSBI/Signal_Processing/Talking/Pass", String, talking2_Callback)
    rospy.Subscriber("/robotis/open_cr/button", String, opencrButton_Callback)
    rospy.Subscriber("/robotis/open_cr/imu", Imu, opencrImu_Callback)
    time.sleep(1)
   # rospy.Rate(50)

def EnableCtrlModule(Module):
    pub_EnableCtrlModule.publish(Module)
    time.sleep(0.1)

def InitPose():
    pub_InitPose.publish('ini_pose')
    time.sleep(4)

# def Walking(move_x, move_y, move_yaw, period):
#     WalkingParam_msg = WalkingParam()
#     WalkingParam_msg.init_x_offset = -0.01
#     WalkingParam_msg.init_y_offset = 0.015
#     WalkingParam_msg.init_z_offset = 0.035
#     WalkingParam_msg.init_roll_offset = 0.0
#     WalkingParam_msg.init_pitch_offset = 0.0
#     WalkingParam_msg.init_yaw_offset = 0.0
#     WalkingParam_msg.hip_pitch_offset = 0.12
#     WalkingParam_msg.period_time = period
#     WalkingParam_msg.dsp_ratio = 0.2
#     WalkingParam_msg.step_fb_ratio = 0.3
#     WalkingParam_msg.x_move_amplitude = move_x
#     WalkingParam_msg.y_move_amplitude = move_y
#     WalkingParam_msg.z_move_amplitude = 0.06
#     WalkingParam_msg.angle_move_amplitude = move_yaw * math.pi/180
#     WalkingParam_msg.move_aim_on = False
#     WalkingParam_msg.balance_enable = True
#     WalkingParam_msg.balance_hip_roll_gain = 0.35
#     WalkingParam_msg.balance_knee_gain = 0.5
#     WalkingParam_msg.balance_ankle_roll_gain = 0.7
#     WalkingParam_msg.balance_ankle_pitch_gain = 0.9
#     WalkingParam_msg.y_swap_amplitude = 0.030
#     WalkingParam_msg.z_swap_amplitude = 0.008
#     WalkingParam_msg.arm_swing_gain = 0.20
#     WalkingParam_msg.pelvis_offset = 0.009
#     WalkingParam_msg.p_gain = 0
#     WalkingParam_msg.i_gain = 0
#     WalkingParam_msg.d_gain = 0
#     pub_WalkingParams.publish(WalkingParam_msg)
#     time.sleep(0.1)

def Motion_WalkingParams(x, y, o, t):
    walkingParams_data = WalkingParam()
    walkingParams_data.x_move_amplitude = x
    walkingParams_data.y_move_amplitude = y
    walkingParams_data.angle_move_amplitude = o
    walkingParams_data.period_time = t
    pub_ManWalkingParams.publish(walkingParams_data)
    time.sleep(0.1)

def WalkingCommand(data):
    pub_WalkingCommand.publish(data)
    time.sleep(0.1)

def MotionMode(mode):
    data = String()
    data.data = mode
    pub_ManMotionMode.publish(data)
    time.sleep(0.1)
def Motion_InitWalking():
    MotionMode('walk')
    time.sleep(2.0)
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

def Motion_HeadScan(enable):
    pub_ManHeadScan.publish(enable)
    time.sleep(0.1)

def Motion_HeadControl(pan, tilt):
    pub_ManHeadPan.publish(pan)
    time.sleep(0.1)
    pub_ManHeadTilt.publish(tilt)
    time.sleep(0.1)

def Motion_ActionNum(num):
    pub_ManActionNum.publish(num)
    time.sleep(3)

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

#talker
def sine_wave(frequency, length, rate):
    length = int(length * rate)
    factor = float(frequency) * (math.pi * 2) / rate
    return np.sin(np.arange(length) * factor)

def sine_sine_wave(f1, f2, length, rate):
    s1=sine_wave(f1,length,rate)
    s2=sine_wave(f2,length,rate)
    ss=s1+s2
    sa=np.divide(ss, 2.0)
    return sa

def play_tone(stream, frequency=440, length=0.20, rate=44100):
    frames = []
    frames.append(sine_wave(frequency, length, rate))
    chunk = np.concatenate(frames) * 0.25
    stream.write(chunk.astype(np.float32).tostring())

def play_dtmf_tone(stream, digits, length=0.20, rate=44100):
        dtmf_freqs = {'1': (1209, 697), '2': (1336, 697), '3': (1477, 697), 'A': (1633, 697),
                      '4': (1209, 770), '5': (1336, 770), '6': (1477, 770), 'B': (1633, 770),
                      '7': (1209, 852), '8': (1336, 852), '9': (1477, 852), 'C': (1633, 852),
                      '*': (1209, 941), '0': (1336, 941), '#': (1477, 941), 'D': (1633, 941)}
        dtmf_digits = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#', 'A', 'B', 'C', 'D']
        if type(digits) is not type(''):
            digits = str(digits)[0]
        digits = ''.join([dd for dd in digits if dd in dtmf_digits])
        for digit in digits:
            digit = digit.upper()
            frames = []
            frames.append(sine_sine_wave(dtmf_freqs[digit][0], dtmf_freqs[digit][1], length, rate))
            chunk = np.concatenate(frames) * 0.25
            stream.write(chunk.astype(np.float32).tostring())
            time.sleep(0.2)

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

#listener
def select_pjg(array, number):
    offset = 2
    for i in range(number - offset, number + offset):
        if i in array:
            return True
    return False

DTMF_TABLE = {
    '1': [1209, 697],
    '0': [1336, 941],
#	'2': [1336, 697],
#	'3': [1477, 697],
    #'3': [1477, 697],
    #'A': [1633, 697],
    # '1': [1209, 697],

    # 'A': [1633, 697],
}

signal = ''

def record():
    #ROS Ini
    global L
    global signal
   # rospy.init_node("Tone", anonymous=True)
   # pub = rospy.Publisher("dtmfTone", Int8, queue_size=1)
action_stand = 1
action_keeper = 60
action_standInit = 80
action_sit = 15
action_defenseRight = 61
action_defenseLeft = 62
action_rightKick = 11     #121
action_leftKick = 10
action_sideKickRight = 12
action_getUpFront = 122
action_getUpBack = 123

taskTimer = 0
buzzerTimer = 0
if __name__ == '__main__':
    try:

        Kp = 5.0
        Ki = 0.05
        Kd = 2.0
        KfPoleSize = 30
        KfPolePos = 15
        KfBaseSize = 10
        KfBasePos = 100
        KfBallSize = 10
        KfBallPos = 10
        kfYawError = 10
        errComp = 80

        # talking = "1"
        # talking2 = "1"

        ballSetPoint = 0
        baseSetPoint = 0
        poleSetPoint = -0.4
        stepSpeed_x = 0.040
        stepPeriod = 0.60
        headPan = 0
        initHeadTilt = -0.470
        headTilt = initHeadTilt

        yawOffset = 0
        yawErrorIntegrator = 0
        yawErrorLast = 0
        polePosFiltered = 0
        poleSizeFiltered = 0
        basePosFiltered = 0
        baseSizeFiltered = 0
        ballPosFiltered = 0
        ballSizeFiltered = 0

        #komunikasi

        FORMAT = pyaudio.paInt16 # format of sampling 16 bit int
        CHANNELS = 1 # number of channels it means number of sample in every sampling
        RATE = 10000 # number of sample in 1 second sampling #10000
        CHUNK = 1024 # length of every chunk #1024
        RECORD_SECONDS = 0.5 # time of recording in seconds #0.5
        WAVE_OUTPUT_FILENAME = "dataStream.wav" # file name

        audio = pyaudio.PyAudio()

        Init()
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32,
                        channels=1, rate=44100, output=1)

        EnableCtrlModule("walking_module")
        rospy.loginfo('Walking Module Enabled')
        time.sleep(1)
        rospy.loginfo('Checking Gyroscope...')
        yawOffset = gyroYawIntegrator
        if abs(gyroYawIntegrator - yawOffset) > 1:
            rospy.loginfo('GYROSOCOPE NOT STABLE!')
        else:
            rospy.loginfo('Gyroscope OK')
        time.sleep(1)
        while not rospy.is_shutdown():
            yawError = (gyroYawIntegrator - yawOffset) * -1
            if systemState == 'PRE_RUN':
                rospy.loginfo('Starting')
                LED_Status(0, 1, 0)
                # Motion_WalkingParams(0.00, 0, 0.0, stepPeriod)
                # Motion_Start()
                yawErrorFiltered = 0
                yawOffset = gyroYawIntegrator
                yawErrorLast = 0
                yawErrorIntegrator = 0
                polePosFiltered = 0
                poleSizeFiltered = 0
                headTilt = initHeadTilt
                Motion_HeadControl(0.0, headTilt)
                systemState = 'RUN'
                taskSequence = 'TENDANG'
                # taskSequence = 'TRACKING'
                rospy.loginfo('Recording')
                taskTimer = Ticks()

            elif systemState == 'RUN':
                # u = (yawErrorFiltered * Kp) + (yawErrorIntegrator * Ki) + (yawErrorFiltered - yawErrorLast) * Kd
                # yawErrorIntegrator += yawErrorFiltered
                # yawErrorLast = yawErrorFiltered

                u = (yawError * Kp) + (yawErrorIntegrator * Ki) + (yawErrorLast - yawError) * Kd
                yawErrorIntegrator += yawError
                yawErrorLast = yawError
                # start Recording


                if taskSequence == 'TENDANG':
                    stream = audio.open(format=FORMAT, channels=CHANNELS,
                                    rate=RATE, input=True,
                                    frames_per_buffer=CHUNK)
                    frames = []

                    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                        data = stream.read(CHUNK)
                        frames.append(data)

                    # stop Recording
                    stream.stop_stream()
                    stream.close()

                    # storing voice
                    waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
                    waveFile.setnchannels(CHANNELS)
                    waveFile.setsampwidth(audio.get_sample_size(FORMAT))
                    waveFile.setframerate(RATE)
                    waveFile.writeframes(b''.join(frames))
                    waveFile.close()

                    # reading voice
                    rate, data = wav.read('dataStream.wav')
                    # data is voice signal. its type is list(or numpy array)
                    spf = wave.open('dataStream.wav', 'r')

                    signal = spf.readframes(-1)
                    signal = np.fromstring(signal, "Int16")

                    if spf.getnchannels() == 2:
                        print("mono")
                        sys.exit(0)


                    # Calculate fourier trasform of data
                    data_fft = np.fft.fft(data, RATE)

                    # Convert fourier transform complex number to integer numbers
                    for i in range(len(data_fft)):
                       data_fft[i] = int(np.absolute(data_fft[i]))

                    pfs = wave.open("dataStream.wav", "r")
                    #extract raw audio from wav file
                    signal = pfs.readframes(-1)
                    signal = np.fromstring(signal, "Int16")

                    # if stereo
                    if pfs.getnchannels() == 2:
                        sys.exit(0)


                    #s Calculate lower bound for filtering fourier trasform numbers
                    LowerBound = 10 * np.average(data_fft) #20

                    wav_loc = "dataStream.wav"
                    rate, data = wav.read(wav_loc)
                    global data
                    data = data / 32768

                    def fftnoise(f):
                        f = np.array(f, dtype="complex")
                        Np = (len(f) - 1) // 2
                        phases = np.random.rand(Np) * 2 * np.pi
                        phases = np.cos(phases) + 1j * np.sin(phases)
                        f[1 : Np + 1] *= phases
                        f[-1 : -1 - Np : -1] = np.conj(f[1 : Np + 1])
                        return np.fft.ifft(f).real

                    def band_limited_noise(min_freq, max_freq, samples=1024, samplerate=1):
                        freqs = np.abs(np.fft.fftfreq(samples, 1 / samplerate))
                        f = np.zeros(samples)
                        f[np.logical_and(freqs >= min_freq, freqs <= max_freq)] = 1
                        return fftnoise(f)

                    # Filter fourier transform data (only select frequencies that X(jw) is greater than LowerBound)
                    filter_freq = []
                    for i in range(len(data_fft)):
                        if (data_fft[i] > LowerBound):
                            filter_freq.append(i)

                    for char, freqCocok in DTMF_TABLE.items():
                        if (select_pjg(filter_freq, freqCocok[0]) and
                            select_pjg(filter_freq, freqCocok[1])):
                            #print(char)
                            if (char == '0'):
                            ##### if listen == 1 :
                                LED_Status(0, 0, 1)
                                rospy.loginfo("Tendang")
                                Motion_Stop()
                                Motion_InitAction()
                                Motion_ActionNum(action_leftKick)
                                time.sleep(0.2)
                                rospy.loginfo('SELESAI TENDANG')
                                Motion_InitWalking()
                                Motion_WalkingParams(0.00, 0, 0, 0.65)
                                Motion_Start()
                                time.sleep(0.5)
                                taskTimer = Ticks()
                                LED_Status(1, 0, 0)
                                taskSequence = 'NGOMONG'

                elif taskSequence == 'NGOMONG':
                    #lari ambek teriak
                    if Ticks() - taskTimer > 1500 :
                        rospy.loginfo('TALKING')
                        ####talking = '1'
                        p = pyaudio.PyAudio()
                        stream = p.open(format=pyaudio.paFloat32,
                                        channels=1, rate=44100, output=1)
                        if len(sys.argv) != 2:
                            digits = "0"
                            rospy.loginfo("Ngomong")
                        else:
                            digits = sys.argv[1]

                        play_dtmf_tone(stream, digits, length=2.0)
                        stream.close()
                        p.terminate()
                        taskTimer = Ticks()
                        taskSequence = 'LARI'
                    else :
                        rospy.loginfo("Lari")
                        Motion_WalkingParams(stepSpeed_x, 0.00, u, stepPeriod)

                elif taskSequence == 'LARI' :
                    if Ticks() - taskTimer > 7000:
                        yawOffset += 75
                        BuzzerTone(3, 100)
                        taskSequence = 'TRACKING'
                        rospy.loginfo('TRACKING')
                    elif Ticks() - taskTimer > 6000:
                        rospy.loginfo('PUTER')
                        Motion_WalkingParams(0.035, 0.00, 500, 0.68)
                    elif Ticks() - taskTimer > 6350:
                        Motion_WalkingParams(stepSpeed_x, 0, 0, stepPeriod)
                    else :
                        rospy.loginfo("Lari")
                        Motion_WalkingParams(stepSpeed_x, 0.00, 0.0, stepPeriod)

                elif taskSequence == 'TRACKING' :
                    ballSizeFiltered = ballPos.z
                    ballPosFiltered = ballPos.x
                    # ballSizeFiltered = (ballSizeFiltered * KfBallSize) + ballPos.z
                    # ballSizeFiltered /= KfBallSize + 1
                    # ballPosFiltered = (ballPosFiltered * KfBallPos) + ballPos.x
                    # ballPosFiltered /= KfBallPos + 1
                    #
                    xError = (ballPosFiltered * -1 - ballSetPoint)
                    # rospy.loginfo(xError)
                    xSpeed = stepSpeed_x / (1 + abs(xError / 5))
                    xSpeed = xSpeed / (1 + abs(yawError / 5))
                    xError /= 5
                    if xError > 0.018: xError = 0.018
                    elif xError < -0.018: xError = -0.018
                    if ballState == "OK":
                        Motion_WalkingParams(xSpeed, xError, u, stepPeriod)
                        rospy.loginfo('bola jauh')
                        head = (-1 * (ballSizeFiltered / 100)) + -0.25
                        if head > 0:
                            head = -0.4
                        elif head < -1.2:
                            head = -1.2
                        # elif head < -0.8:
                        #     head = head + -0.1
                        rospy.loginfo(ballSizeFiltered)
                        Motion_HeadControl(0, head)
                        LED_RGB(0, 31, 0)
                        if ballSizeFiltered > 67:
                            if xError < -0.01 :
                                Motion_Stop()
                                rospy.loginfo('tendang kanan')
                                LED_RGB(0, 0, 31)
                                Motion_InitAction()
                                Motion_ActionNum(action_rightKick)
                                Motion_InitWalking()
                                Buzzer(3000)
                                time.sleep(0.2)
                                Buzzer(2000)
                                time.sleep(0.2)
                                Buzzer(0)
                                # Motion_InitPose()
                                systemState = 'PRE_STOP'
                            else :
                                Motion_Stop()
                                rospy.loginfo('tendang kiri')
                                LED_RGB(0, 0, 31)
                                Motion_InitAction()
                                Motion_ActionNum(action_leftKick)
                                Motion_InitWalking()
                                Buzzer(3000)
                                time.sleep(0.2)
                                Buzzer(2000)
                                time.sleep(0.2)
                                Buzzer(0)
                                # Motion_InitPose()
                                systemState = 'PRE_STOP'

                    else:
                        Motion_WalkingParams(-0.010, xError, u, 0.65)
                        LED_RGB(0, 0, 0)
                        rospy.loginfo('GADA BOLA')

                if abs(imuData.linear_acceleration.x) > 8.0:
                    systemState = 'RECOVERY'
                    LED_RGB(31, 0, 0)

            elif systemState == 'PRE_STOP':
                LED_RGB(0, 0, 0)
                LED_Status(1, 0, 0)
                rospy.loginfo('Reset Position')
                Motion_Stop()
                #Motion_InitPose()
                # plt.close()
                # plt.plot(t_t, error_t, color='red')
                # plt.plot(t_t, errorFiltered_t, color='orange')
                # plt.plot(t_t, u_t, color='blue')
                # plt.plot(t_t, sp_t, color='black')
                # plt.show(block=False)
                systemState = 'STOP'
                rospy.loginfo('STOP')

            elif systemState == 'RECOVERY':
                rospy.loginfo('Recovery')
                Motion_Stop()
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

        audio.terminate()
        record()
        # plt.figure(1)
        # plt.title("DTMF DATA STREAM")
        # plt.plot(signal)
        # plt.figure(2)
        # plt.title("Bandpass Filter")
        # plt.plot(data)
        # plt.show()
    except rospy.ROSInterruptException:
        pass