import json
import rospy
from std_msgs.msg import String

from ..device import cv_handler  # For running mission-specific CV scripts
from ..motion import robot_control  # For running the motors on the sub
from ..utils import disarm, arm

class PolesMission:    

    def __init__(self, side="left", **config):
        """
        Initialize the mission class for the poles.
        Args:
            side: "left" or "right" (from the gate mission)
            config: Mission-specific parameters to run the mission.
        """
        self.cv_files = ["poles_cv_huy"]  # List of CV files to run for this mission
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        # Add the side (left/right) into config so your poles_cv_huy.py uses it
        self.config['side'] = side

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Start CV handler for each CV file
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)
        self.cv_handler.set_target("poles_cv_huy", side)
        print(f"[INFO] Poles Mission Init (side: {side})")

    def callback(self, msg):
        """
        Called by cv_handler when new data arrives.
        """
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True
        # print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Main mission loop.
        """
        while not rospy.is_shutdown():
            if not self.received:
                rospy.sleep(0.01)
                continue

            # Merge CV output
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            # Extract movement commands from CV
            cv_key = "poles_cv_huy"
            forward = self.data[cv_key].get("forward", 0)
            lateral = self.data[cv_key].get("lateral", 0)
            yaw = self.data[cv_key].get("yaw", 0)
            vertical = self.data[cv_key].get("vertical", 0)
            end = self.data[cv_key].get("end", False)

            if end:
                print("[INFO] Poles mission finished (CV returned end=True)")
                self.robot_control.movement(lateral=0, forward=0, yaw=0, vertical=0)
                break
            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=vertical)
                print(f"[CMD] F:{forward} L:{lateral} Y:{yaw} V:{vertical}")

        print("[INFO] Poles mission run complete")

    def cleanup(self):
        """
        Cleanup after the run.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)
        self.robot_control.movement(lateral=0, forward=0, yaw=0, vertical=0)
        print("[INFO] Poles mission terminated")


if __name__ == "__main__":
    import time
    from auv.utils import deviceHelper
    from auv.motion import robot_control

    rospy.init_node("poles_mission", anonymous=True)

    config = deviceHelper.variables
    config.update({
        # You can set dummy video for testing here:
        # "cv_dummy": ["C:/Users/HOME/Documents/GitHub/CV_data/poles_test_1.mp4"],
    })

    # Here, set side based on the result of the gate mission
    # Example: side="left" or side="right"
    mission = PolesMission(side="left", **config)
    rc = robot_control.RobotControl()

    # Run the mission
    arm.arm()
    rc.set_depth(0.7)
    time.sleep(5)   
    mission.run()
    mission.cleanup()
    disarm.disarm()  # Disarm the robot after mission completion
