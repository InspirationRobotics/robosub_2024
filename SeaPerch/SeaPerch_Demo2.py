import rospy
import time
from auv.motion import robot_control

class Movement:
    def __init__(self):
        self.robot_control = robot_control.RobotControl()

    def sleep(self):
        """Allows AUV to dissipate momentum then reset
        the time used to move in a particular direction"""
        rospy.sleep(2)
        self.first_time = time.time()

    def circumnavigate(self):
        """Circumnavigates the buoy based on the gate mission choice. 
        Aims to make a square around the buoy"""
        print("Starting circumnavigation")
        self.first_time = time.time()
        # yaw_time = 2.2 # Tune this value -- the amount of time it takes at power 1 or -1 to go 90 degrees
        # forward_time = 10 # Tune this value -- the amount of time it takes to go forward at power 1
        # lateral_time = 8 # Tune this value -- the amount of time it takes to go lateral at power 1
        forward = 2
        yaw = 1
        for i in range(4):
            while time.time() - self.first_time < 2:
                self.robot_control.movement(forward = forward)
            self.sleep()
            while time.time() - self.first_time < 0.6:
                self.robot_control.movement(yaw = yaw)
            self.sleep()
    