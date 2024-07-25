"""
Mission file for the Gate class. Essentially takes the output from gate_cv and sends it over to robot_control to move the AUV.
"""

import json

import rospy
import time
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For running the motors on the sub
from .. utils import arm, disarm
from . import style_mission

class GateMission:
    cv_files = ["gate_cv"] # CV file to run

    def __init__(self, target="Red", **config):
        """
        Initialize the mission class; here should be all of the things needed in the run function. 

        Args:
            config: Mission-specific parameters to run the mission.
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler
        self.next_data = {}  # Dictionary to store the newest data from the CV handler; this data will be merged with self.data.
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV handlers; dummys are used to input a video file instead of the camera stream as data for the CV script to run on
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("gate_cv", target)
        print("[INFO] Gate Mission Init")

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

        print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """
        print("[INFO] Beginning Gate Run Function")
        while not rospy.is_shutdown():
            # print("[INFO] ROSPY run")
            curr_time = time.time()
            time.sleep(0.1)
            if not self.received:
                continue

            # Merge self.next_data, which contains the updated CV handler output, with self.data, which contains the previous CV handler output.
            # self.next_data will be wiped so that it can be updated with the new CV handler output.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key]) # Merge the data
                    print(key)
                else:
                    self.data[key] = self.next_data[key] # Update the keys if necessary
            self.received = False
            self.next_data = {}

            # Do something with the data.
            lateral = self.data["gate_cv"].get("lateral", None)
            forward = self.data["gate_cv"].get("forward", None)
            yaw = self.data["gate_cv"].get("yaw", None)
            end = self.data["gate_cv"].get("end", None)

            if end:
                print("Ending")
                self.robot_control.movement(lateral = 0, forward = 0, yaw = 0)
                break
            else:
                self.robot_control.movement(lateral = lateral, forward = forward, yaw = yaw)
                # print(forward, lateral, yaw)
            print(f"[DEBUG]: Time elapsed is {time.time() - curr_time}")

        print("[INFO] gate mission run")
    
    def style_movement(self):
        # run style - compass heading functions will ensure
        # she is at her initial heading
        style = style_mission.StyleMission()
        style.run()
        style.cleanup()

        # Go forward for 5 secs
        initial_time = time.time()
        while time.time() - initial_time < 5:
            self.robot_control.movement(forward=2)



    def cleanup(self):
        """
        Here should be all the code required after the run function.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # Idle the robot
        self.robot_control.movement(lateral = 0, forward = 0, yaw = 0)
        print("[INFO] gate mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.gate_mission"
    # You can also import it in a mission file outside of the package
    import time
    from auv.utils import deviceHelper

    rospy.init_node("gate_mission", anonymous=True)

    config = deviceHelper.variables
    # config.update(
    #     {
    #         # # this dummy video file will be used instead of the camera if uncommented
    #         # "cv_dummy": ["/somepath/thisisavideo.mp4"],
    #     }
    # )

    # Create a mission object with arguments
    mission = GateMission(**config)

    arm.arm()

    # Run the mission
    mission.run()
    mission.cleanup()
    disarm.disarm()
