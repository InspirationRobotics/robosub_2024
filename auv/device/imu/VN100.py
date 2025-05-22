import time
import threading
from serial import Serial
import rospy
import sensor_msgs.msg
from auv.utils import deviceHelper

rospy.init_node("vectornav_api_node")

class VN100:
    def __init__(self,port:str = deviceHelper.dataFromConfig("vectornav_port")):
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port,baudrate=self.__bps, timeout=1)
        self.vectornav_subscriber = rospy.Subscriber("/vectornav/IMU", sensor_msgs.msg.Imu, self.get_orientation)
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()
    
    def get_orientation(self, msg):
        print(msg.orientation)
    
    
if __name__ == "__main__":
    vectornav_sensor = VN100()    


