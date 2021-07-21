import rospy
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

gyroYawIntegrator = 0.0
def opencrImu_Callback(data):
    global imuData
    global gyroYawIntegrator
    imuData = data
    zInt += imuData.angular_velocity.z
    #pubData = Vector3()
    #pubData.z = zInt
    #pub_ImuData.publish(pubData)
    print(zInt)

pub_ImuData = rospy.Publisher("/KRSBI/Manuvering/Imu", Vector3, queue_size=10)

def Init():
    rospy.init_node("ImuHandler", anonymous = False)
    rospy.Subscriber("/robotis/open_cr/imu", Imu, opencrImu_Callback)
    time.sleep(1)

if __name__ == '__main__':
    try:
        Init()
        rospy.spin()
    except rospy.ROSInterruptException():
        pass