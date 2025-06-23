"""
Finds the red pole, navigates to an offset position, and performs a slalom maneuver between alternating poles while staying below the horizontal plane.
"""

import json
import rospy
import time

from ..device import cv_handler  # For running mission-specific CV scripts
from ..motion import robot_control  # For running the motors on the sub
from ..utils import arm, disarm

class PoleSlalomMission:

    def __init__(self, target="Red", **config):
        """
        Initialize the mission class.

        Args:
            config: Mission-specific parameters to run the mission.
        """
        self.cv_files = ["poles_cv"]
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False
        self.target = target

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        self.passed_poles = 0
        self.required_passes = 6  # Adjust based on point rules or mission goals

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("poles_cv", target)
        print("[INFO] Pole Slalom Mission Init")

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        print("[INFO] Starting Red Pole Slalom run...")

        while not rospy.is_shutdown():
            time.sleep(0.01)
            if not self.received:
                continue

            for key in self.next_data.keys():
                if key in self.data:
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            # Get movement commands
            lateral = self.data["poles_cv"].get("lateral", 0)
            forward = self.data["poles_cv"].get("forward", 0)
            yaw = self.data["poles_cv"].get("yaw", 0)
            vertical = self.data["poles_cv"].get("vertical", 0)

            # This version of the pole CV doesn't set an `end` flag; you can optionally add that based on pass count
            self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=vertical)

            # Optional: Count pole passes based on x-movement or time (requires slalom logic in CV to publish it)
            # You can increase self.passed_poles here if CV adds a flag

            # Example (dummy):
            # if self.data["poles_cv"].get("passed_pole", False):
            #     self.passed_poles += 1

            # End after passing required number of poles
            if self.passed_poles >= self.required_passes:
                print("[INFO] Completed slalom pass requirement.")
                break

        print("[INFO] Red Pole Slalom mission complete")

    def cleanup(self):
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)
        self.robot_control.movement(lateral=0, forward=0, yaw=0)
        print("[INFO] Pole Slalom Mission Terminated")


if __name__ == "__main__":
    import time
    from auv.utils import deviceHelper
    from auv.motion import robot_control

    rospy.init_node("pole_slalom_mission", anonymous=True)

    config = deviceHelper.variables
    config.update({
        # "cv_dummy": ["/path/to/test_video.mp4"],
    })

    mission = PoleSlalomMission(**config)
    rc = robot_control.RobotControl()

    arm.arm()
    rc.set_depth(0.5)
    time.sleep(5)
    mission.run()
    mission.cleanup()
    disarm.disarm()