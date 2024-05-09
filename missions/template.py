"""
Template class to run an individual mission. 
"""

import json

import rospy

from auv.motion import robot_control
from auv.device import cv_handler
from auv.utils import disarm

class templateMission:
    """
    Class to run the mission.

    Attributes:
        config: Configuration of the devices on the sub.
        data: Data from the CV handler.
        nextdata: Data that contains the most updated/newest data from the CV handler.
        received: Flag indicating whether output from the CV handler has been received.
        RobotControl: Instance of the RobotControl class, which controls the movement of the sub.
        CVHandler: Instance of the CVHandler class, which controls the running of individual, mission-specific, CV scripts.
    """

    cv_files: [] # Adjust based on the mission specific CV file.

    def __init__(self, **config):
        """
        Initialize the mission class.

        Args:
            config: Dictionary containing the configuration of the sub's devices.
        """

        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        self.RobotControl = robot_control.RobotControl()
        self.CVHandler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.CVHandler.start_cv(file_name, self.callback)

        print("[INFO] Template Mission Initialization.")

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
        Code to run the mission.
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

            lateral = self.data["template_cv"].get("lateral")
            forward = self.data["template_cv"].get("forward")
            yaw = self.data["template_cv"].get("yaw")
            end = self.data["template_cv"].get("end")

            if end:
                print("Ending...")
                self.RobotControl.movement(lateral = 0, forward = 0, yaw = 0)
                break
            else:
                self.RobotControl.movement(lateral = lateral, forward = forward, yaw = yaw)

        print("Template mission running...")

    def cleanup(self):
        """
        Exit the mission gracefully. Stops the CV script running through the CV handler, and idle the robot.
        """
        for file_name in self.cv_files:
            self.CVHandler.stop_cv(file_name)
        
        self.RobotControl.movement(lateral = 0, forward = 0, yaw = 0)
        print("[INFO] Template Mission terminated.")

    if __name__ == "main":
        # The code inside this if statement will be run if you call the mission directly.
        # It is here for testing purposes.
        # You can call this file using "python -m mission.template.py" if you are outside the directory, or 
        # "python -m template.py" if you are inside the directory.

        import time
        from auv.utils import deviceHelper

        rospy.init_node("Template Mission", anonymous = True)
        config = deviceHelper.variables

        mission = templateMission(**config)

        mission.run()
        mission.cleanup()