"""
Class to run the prequalification maneuver. 
"""

import time
import json

# Find the two sides of the gate
# Calculate the midpoint
# Yaw until aligned
# Strafe until aligned
# Move through the gate
# Keep moving until we see the marker, and it takes up a fair amount of the screen.
# Yaw in one direction, then move forward and slowly yaw in the other direction to circle the marker.
# After circling, keep moving forward slowly and adjust yaw then lateral to make exit the gate at the midpoint.

# NOTE: We can use yaw logic as follows:
    # First, yaw until both sides of the gate are in the frame
    # Then, make adjust laterally until the midpoint of the frame is aligned with the midpoint of the gate
    # Yaw again so that the sides of the gate are equal distanced from the side of the frame
    # Move laterally to fully align.

import rospy

from auv.motion import robot_control
from auv.device import cv_handler
from auv.utils import disarm

class PreQualificationMission:
    """
    Class to run the prequalification maneuver.

    Attributes:
        config: Configuration of the devices on the sub.
        data: Data from the CV handler.
        nextdata: Data that contains the most updated/newest data from the CV handler.
        received: Flag indicating whether output from the CV handler has been received.
        RobotControl: Instance of the RobotControl class, which controls the movement of the sub.
        CVHandler: Instance of the CVHandler class, which controls the running of individual, mission-specific, CV scripts.
    """

    cv_files: ["PreQualification_cv"]

    def __init__(self, **config):
        """
        Initialize the PreQualificationMission class.

        Args:
            config: Dictionary containing the configuration of the sub's devices.
        """

        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        self.RobotControl = robot_control.RobotControl()
        self.CVHandler = cv_handler.CVHandler(**self.config)

        for file_name in cv_files:
            self.CVHandler.start_cv(file_name, self.callback)

        print("[INFO] PreQualification Maneuver Initialization.")

    def callback(self, msg):
        """
        Callback for the CV handler's output. Stores the output in a dictionary.

        Args:
            msg: Message data from the CV handler.
        """

        file_name = msg.connection_header["topic"].split("/")[-1]
        data = json.loads(msg)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        """
        Code to run the PreQualification Mission.
        """

        while not rospy.is_shutdown():
            if not self.received:
                print("No output received from the CV Handler")

            self.RobotControl.set_depth(4.0)

            for key in self.next_data.keys:
                if key in self.data.keys:
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]

            lateral = self.data["PreQualification_cv"].get("lateral")
            forward = self.data["PreQualification_cv"].get("forward")
            yaw = self.data["PreQualification_cv"].get("yaw")
            end = self.data["PreQualification_cv"].get("end")

            if end:
                print("Ending...")
                self.RobotControl.movement(lateral = 0, forward = 0, yaw = 0)
                break
            else:
                self.RobotControl.movement(lateral = lateral, forward = forward, yaw = yaw)

        print("PreQualification Mission running...")

    def cleanup(self):
        """
        Exit the mission gracefully. Stops the CV script running through the CV handler, and idle the robot.
        """
        for file_name in cv_files:
            self.CVHandler.stop_cv(file_name)
        
        self.RobotControl.movement(lateral = 0, forward = 0, yaw = 0)
        print("[INFO] PreQualification Mission terminated.")

    if __name__ == "main":
        # The code inside this if statement will be run if you call the mission directly.
        # It is here for testing purposes.
        # You can call this file using "python -m mission.PreQualification_Mission.py" if you are outside the directory, or 
        # "python -m PreQualification_Mission.py" if you are inside the directory.

        import time
        from auv.utils import deviceHelper

        rospy.init_node("PreQualification Mission", anonymous = True)
        config = deviceHelper.variables

        mission = PreQualificationMission(**config)

        mission.run()
        mission.cleanup()