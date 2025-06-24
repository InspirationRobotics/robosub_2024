import json
import rospy
import time

from std_msgs.msg import String
from ..device import cv_handler
from ..motion import robot_control
from ..utils import arm, disarm

class PoleSlalomMission:

    def __init__(self, side="left", **config):
        """
        Initialize the slalom pole mission class.
        Args:
            side: Which side of the red pipe to pass (left or right)
            config: Mission-specific parameters
        """
        self.cv_files = ["pole_cv"]
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False
        self.side = side
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)
        self.cv_handler.set_target("pole_cv", self.side)
        print(f"[INFO] Pole Slalom Mission initialized for {self.side} side")

    def callback(self, msg):
        """
        Process CV handler output, convert to usable data.
        """
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        """
        Run the navigation through the red/white poles.
        The AUV adjusts based on detected center.
        """
        rospy.loginfo("[INFO] Starting Pole Slalom Mission")
        aligned_counter = 0

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

            motion_data = self.data.get("pole_cv", {})
            forward = motion_data.get("forward", 0)
            lateral = motion_data.get("lateral", 0)
            yaw = motion_data.get("yaw", 0)
            vertical = motion_data.get("vertical", 0)
            end = motion_data.get("end", False)

            # Basic success condition: aligned for several frames
            if abs(yaw) < 0.05:
                aligned_counter += 1
            else:
                aligned_counter = 0

            self.robot_control.movement(forward=forward, lateral=lateral, yaw=yaw, vertical=vertical)
            rospy.loginfo(f"[MOVEMENT] F: {forward}, L: {lateral}, Y: {yaw}, V: {vertical}")

            if aligned_counter > 10:
                rospy.loginfo("[INFO] AUV aligned through the gate. Mission accomplished.")
                break

        self.robot_control.movement(0, 0, 0, 0)

    def cleanup(self):
        """
        Cleanup actions after mission finishes.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)
        self.robot_control.movement(0, 0, 0)
        print("[INFO] Pole Slalom Mission cleanup complete")

if __name__ == "__main__":
    from auv.utils import deviceHelper
    from auv.motion import robot_control

    rospy.init_node("pole_slalom_mission", anonymous=True)

    config = deviceHelper.variables
    config.update({
        # "cv_dummy": ["/Users/avikaprasad/Downloads/poles_test_2.mp4"],  # Uncomment for testing with dummy input
    })

    mission = PoleSlalomMission(side="left", **config)
    rc = robot_control.RobotControl()

    arm.arm()
    rc.set_depth(0.5)
    time.sleep(5)

    try:
        mission.run()
    finally:
        mission.cleanup()
        disarm.disarm()