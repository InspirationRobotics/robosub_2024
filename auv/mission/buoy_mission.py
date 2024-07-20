"""
Mission class for the buoy.
"""

import json
import rospy
import time

from std_msgs.msg import String
from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For running the motors on the sub
from ..utils import arm, disarm

class BuoyMission:
    
    def __init__(self, target="Red", **config):
        """
        Initialize the mission class; here should be all of the things needed in the run function. 

        Args:
            config: Mission-specific parameters to run the mission.
        """
        self.cv_files = ["buoy_cv"] # CV file to run
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler
        self.next_data = {}  # Dictionary to store the newest data from the CV handler; this data will be merged with self.data.
        self.received = False
        self.target = target

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        self.positioned = False

        # Initialize the CV handlers; dummys are used to input a video file instead of the camera stream as data for the CV script to run on
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("buoy_cv", target)
        print("[INFO] Buoy Mission Init")

    def callback(self, msg):
        """
        Calls back the cv_handler output -- you can have multiple callbacks for multiple CV handlers. Converts the output into JSON format.

        Args:
            msg: cv_handler output -- this will be a dictionary of motion commands and potentially the visualized frame as well as servo commands (like the torpedo launcher)
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the topic name
        data = json.loads(msg.data) # Convert the data to JSON
        self.next_data[file_name] = data 
        self.received = True

    def run(self):
        """
        Here should be all the code required to approach the buoy.
        This could be a loop, a finite state machine, etc.
        """

        while not rospy.is_shutdown():
            if not self.received:
                continue

            # Merge self.next_data, which contains the updated CV handler output, with self.data, which contains the previous CV handler output.
            # self.next_data will be wiped so that it can be updated with the new CV handler output.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key]) # Merge the data
                else:
                    self.data[key] = self.next_data[key] # Update the keys if necessary
            self.received = False
            self.next_data = {}

            # Do something with the data.
            lateral = self.data["buoy_cv"].get("lateral", None)
            forward = self.data["buoy_cv"].get("forward", None)
            yaw = self.data["buoy_cv"].get("yaw", None)
            end = self.data["buoy_cv"].get("end", None)

            if end:
                print("[INFO] AUV has aligned with the buoy. Beginning circumnavigation.")
                self.positioned = True
                break
            else:
                self.robot_control.movement(lateral = lateral, forward = forward, yaw = yaw)
                # print(forward, lateral, yaw) 
            
        print("[INFO] Buoy mission run")
        
        if self.positioned == True:
            self.circumnavigate()
    
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
        if self.target == "Red":
            movement_list = [-1.5, 1.5, 2] # lateral, forward, yaw
        elif self.target == "Blue":
            movement_list = [1.5, 1.5, -2] # lateral, forward, yaw
        # First move laterally, then move around the buoy
        self.sleep()
        while time.time() - self.first_time < 0.75:
            self.robot_control.movement(lateral = movement_list[0])
        self.sleep()
        for i in range(4):
            while time.time() - self.first_time < 3.0:
                self.robot_control.movement(forward = movement_list[1])
            self.sleep()
            # More than 1.0 to counteract slight left skew of forward movement
            while time.time() - self.first_time < 1.2:
                self.robot_control.movement(yaw = movement_list[2])
            self.sleep()
        while time.time() - self.first_time < 0.75:
            self.robot_control.movement(lateral = -movement_list[0])
        self.sleep()
        while time.time() - self.first_time < 2.4:
            self.robot_control.movement(yaw = -movement_list[2])
        self.sleep()

    def cleanup(self):
        """
        Here should be all the code required after the run function.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # Idle the robot
        self.robot_control.movement(lateral = 0, forward = 0, yaw = 0)
        print("[INFO] Buoy mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.buoy_mission"
    # You can also import it in a mission file outside of the package
    import time
    from auv.utils import deviceHelper

    rospy.init_node("buoy_mission", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = BuoyMission(**config)

    # Run the mission
    arm.arm()
    mission.circumnavigate()
    mission.cleanup()
    disarm.disarm()
