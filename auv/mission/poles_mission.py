"""
Mission file to navigate a slalom course of white and red vertical PVC pipes.
White pipes are on the left and right, red is in the center.
The AUV must enter from one side of the red pipe and stay on that side while slaloming.
"""

import json
import rospy
from std_msgs.msg import String

from ..device import cv_handler
from ..motion import robot_control
from ..utils import disarm

class PipeSlalomMission:
    cv_files = ["poles_cv"]

    def __init__(self, target=None, **config):
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("pipe_slalom_cv", target)
        print("[INFO] Pipe Slalom Mission Initialized")

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

        print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        side_locked = False
        chosen_side = None  # "left" or "right"

        while not rospy.is_shutdown():
            if not self.received:
                continue

            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]

            self.received = False
            self.next_data = {}

            cv_data = self.data["poles_cv"]

            forward = cv_data.get("forward", 0)
            lateral = cv_data.get("lateral", 0)
            yaw = cv_data.get("yaw", 0)
            end = cv_data.get("end", False)
            side = cv_data.get("side", None)  # Optional: which side of red pipe AUV is on

            if not side_locked and side in ["left", "right"]:
                chosen_side = side
                side_locked = True
                print(f"[INFO] Locked on to {chosen_side} side of red pipe")

            # Enforce slalom path rules: stay on same side if locked
            if side_locked and side != chosen_side:
                print(f"[WARN] Switching sides! Penalized path.")
                # Optionally: adjust path or halt
                lateral = 0  # force correction or stop
                forward = 0

            if end:
                print("[INFO] Mission Complete")
                self.robot_control.movement(forward=0, lateral=0, yaw=0)
                break
            else:
                self.robot_control.movement(forward=forward, lateral=lateral, yaw=yaw)
                print(f"[DEBUG] Movement -> Fwd: {forward}, Lat: {lateral}, Yaw: {yaw}")

        print("[INFO] Pipe Slalom Mission Run Complete")

    def cleanup(self):
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        self.robot_control.movement(forward=0, lateral=0, yaw=0)
        print("[INFO] Pipe Slalom Mission Terminated")


if __name__ == "__main__":
    import time
    from auv.utils import deviceHelper

    rospy.init_node("pipe_slalom_mission", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # Optional: replace with a video for offline testing
            # "cv_dummy": ["/somepath/slalom_test_video.mp4"],
        }
    )

    mission = PipeSlalomMission(**config)
    mission.run()
    mission.cleanup()