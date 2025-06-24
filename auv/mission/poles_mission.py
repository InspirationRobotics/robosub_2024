"""
Mission file for red pole slalom
"""

import json
import rospy
from std_msgs.msg import String

from ..device import cv_handler  # For running mission-specific CV scripts
from ..motion import robot_control  # For running the motors on the sub
from ..utils import disarm


class PoleSlalomMission:
    cv_files = ["pole_cv"]  # Name of your red pole CV script file (no .py extension)

    def __init__(self, target=None, **config):
        """
        Initialize the mission class; configure everything needed in the run function.
        """
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("pole_cv", target)
        print("[INFO] Pole Slalom Mission Init")

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True
        print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Run the pole slalom mission loop.
        """
        print("[INFO] Pole Slalom mission running")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.received:
                rate.sleep()
                continue

            for key in self.next_data:
                if key in self.data:
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]

            self.received = False
            self.next_data = {}

            cv_data = self.data["pole_cv"]
            lateral = cv_data.get("lateral", 0)
            forward = cv_data.get("forward", 0)
            yaw = cv_data.get("yaw", 0)
            end = cv_data.get("end", False)

            print(f"[MOTION] Fwd: {forward}, Lat: {lateral}, Yaw: {yaw}")

            if end:
                print("[INFO] Pole slalom mission complete.")
                self.robot_control.movement(lateral=0, forward=0, yaw=0)
                break
            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)

            rate.sleep()

        print("[INFO] Pole Slalom mission run complete")

    def cleanup(self):
        """
        Clean up after the mission.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        self.robot_control.movement(lateral=0, forward=0, yaw=0)
        print("[INFO] Pole Slalom mission terminated")


if __name__ == "__main__":
    import time
    from auv.utils import deviceHelper

    rospy.init_node("pole_slalom_mission", anonymous=True)

    config = deviceHelper.variables
    config.update({
        # "cv_dummy": ["/somepath/test.mp4"],  # Uncomment for offline video testing
        "strafe_direction": "right",           # or "left"
        "red_size_threshold": 12000,
        "forward_duration": 30
    })

    mission = PoleSlalomMission(**config)
    mission.run()
    mission.cleanup()