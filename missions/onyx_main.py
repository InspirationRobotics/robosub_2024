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

movement_list = [-2, 4, 1] # lateral, forward, yaw



arm.arm()
time.sleep(5)

first_time = time.time()

# move forward for 8 secs


while time.time() - first_time < 18:
    rc.movement(forward = movement_list[1])

time.sleep(1)


# Run buoy mission

marker_mission.run()
marker_mission.cleanup()

# Move forward for 8 secs

first_time = time.time()

while time.time() - first_time < 18:
    rc.movement(forward = movement_list[1])

time.sleep(1)

disarm.disarm()
