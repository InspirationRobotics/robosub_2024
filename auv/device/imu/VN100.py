import time
import threading
from serial import Serial
import rospy
from sensor_msg.msg import Imu

class VN100:
    def __init__(self,port:str = "dev/ttyUSB0"):
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port,baudrate=self.__bps, timeout=1)
    
    
    


