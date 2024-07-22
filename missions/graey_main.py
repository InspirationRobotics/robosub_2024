"""
To create a sequential order of missions for Graey to follow.

Graey will complete the Rough Seas, Enter the Pacific, Path, and Hydrothermal Vent missions.
"""

import rospy
import time

from auv.mission import buoy_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("prequal_mission", anonymous = True)

marker_mission = buoy_mission.BuoyMission()
rc = robot_control.RobotControl()

movement_list = [-2, 1.5, 2] # lateral, forward, yaw
first_time = time.time()


arm.arm()

time.sleep(5)
first_time = time.time()

# while time.time() - first_time < 2.4:
#     rc.movement(yaw = movement_list[2])


while time.time() - first_time < 19:
    rc.movement(forward = movement_list[1])

time.sleep(1)

marker_mission.circumnavigate()
# marker_mission.cleanup()

# Move forward for 8 secs

first_time = time.time()

while time.time() - first_time < 10:
    rc.movement(forward = movement_list[1])

time.sleep(1)

disarm.disarm()
