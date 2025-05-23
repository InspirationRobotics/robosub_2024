import time
import threading
from serial import Serial
import rospy
import sensor_msgs.msg
from auv.utils import deviceHelper
from tf.transformations import euler_from_quaternion

rospy.init_node("vectornav_api_node")

class VN100:
    def __init__(self,port:str = deviceHelper.dataFromConfig("vectornav")):
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port,baudrate=self.__bps, timeout=1)
        self.vectornav_subscriber = rospy.Subscriber("/vectornav/IMU", sensor_msgs.msg.Imu, self.get_orientation)
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()
            time.sleep(0.25)
    
    def get_orientation(self, msg):
        """Parses quaternion orientation to Euler angles (roll, pitch, yaw)"""
        quat_orient = msg.orientation
        orientation_list = [quat_orient.x, quat_orient.y, quat_orient.z, quat_orient.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print(f"Roll: {roll}\nPitch:{pitch}\nYaw:{yaw}")


    
    
if __name__ == "__main__":
    vectornav_sensor = VN100()    


