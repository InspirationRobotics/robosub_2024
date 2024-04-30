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
        self.nextdata = {}
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
        self.nextdata[file_name] = data
        self.received = True