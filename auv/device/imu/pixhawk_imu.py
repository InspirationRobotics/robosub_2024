import time
import threading
from serial import Serial
import rospy
import sensor_msgs.msg
import math
from auv.utils import deviceHelper
from transforms3d.euler import quat2euler

rospy.init_node("pixhawk_api_node")

class PixHawk:
    def __init__(self):
        """Initializes a connection to the /vectornav/IMU ros node. There's
        unused infrastructure for a serial connection, seen in vn100_serial.py.
        I tore down the serial infrastructure because having multiple connections
        to the IMU might cause issues with properly getting the data."""
        self.vectornav_subscriber = rospy.Subscriber("/auv/devices/imu", sensor_msgs.msg.Imu, self.get_orientation)
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()
    
    def get_orientation(self, msg):
        """Parses quaternion orientation to Euler angles (roll, pitch, yaw)"""
        # Get quaternion orientation
        quat_orient = msg.orientation
        orientation_list = [quat_orient.x, quat_orient.y, quat_orient.z, quat_orient.w]
        # Parse to euler angles, convert to degrees
        (roll, pitch, yaw) = quat2euler(orientation_list)
        for angle in (roll, pitch, yaw):
            angle = math.degrees(angle) % 360
        print(f"Roll: {self.roll}\nPitch:{self.pitch}\nYaw:{self.yaw}")



    
    
if __name__ == "__main__":
    sensor = PixHawk()
    init_time = time.time()
    while True:
        # Print angles every second
        if time.time() - init_time > 1 and hasattr(sensor, "roll"):
            init_time = time.time()
            print(f"Roll: {sensor.roll}\nPitch:{sensor.pitch}\nYaw:{sensor.yaw}")



