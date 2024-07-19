"""
To create a sequential order of missions for Graey to follow.

Graey will complete the Rough Seas, Enter the Pacific, Path, and Hydrothermal Vent missions.
"""

import rospy
import time

from auv.mission import buoy_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

class PreQualMission:
    def __init__(self) -> None:
        self.marker_mission = buoy_mission.BuoyMission()
        self.rc = robot_control.RobotControl()
        self.first_time = time.time()

    def run(self):
        movement_list = [-2, 2, 1] # lateral, forward, yaw
        self.first_time = time.time()
        # move forward for 15 secs
        while time.time() - self.first_time < 8:
            self.rc.movement(forward = movement_list[1])
        self.marker_mission.sleep()
        while time.time() - self.first_time < 8:
            self.rc.movement(forward = -movement_list[1])
        self.marker_mission.sleep()
        




if __name__ == "__main__":
    rospy.init_node("prequal_mission", anonymous = True)
    mission = PreQualMission()
    arm.arm()
    time.sleep(5)
    mission.run()
    disarm.disarm()