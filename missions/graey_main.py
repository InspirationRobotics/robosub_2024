"""
To create a sequential order of missions for Graey to follow.

Graey will complete the Rough Seas, Enter the Pacific, Path, and Hydrothermal Vent missions.
"""

import rospy
import time

from auv.mission import buoy_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

marker_mission = buoy_mission.BuoyMission()
rc = robot_control.RobotControl()

rospy.init_node("prequal_mission", anonymous = True)

movement_list = [-2, 2, 1] # lateral, forward, yaw
first_time = time.time()


arm.arm()

# move forward for 15 secs

def sleep():
    first_time = time.time()
    rospy.sleep(2)


while time.time() - first_time < 8:
    rc.movement(forward = movement_list[1])
sleep()
while time.time() - first_time < 8:
    rc.movement(forward = -movement_list[1])
sleep()

disarm.disarm()