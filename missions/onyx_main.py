"""
To create a sequential order of missions for Graey to follow.

Graey will complete the Rough Seas, Enter the Pacific, Path, and Hydrothermal Vent missions.
"""

import rospy
import time

from auv.mission import gate_mission, buoy_mission, style_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Onyx", anonymous = True)

rc = robot_control.RobotControl()

target = "Red"
gate_heading = rc.get_heading()

arm.arm()

time.sleep(10)

# Rotate towards the heading of the gate, move 2 meters forward
rc.set_heading(gate_heading)
rc.forward_dvl(2)

# Run the gate mission
gate = gate_mission.GateMission(target)
gate.run()
gate.cleanup()

print("[INFO] Gate mission terminate")

# Run the buoy mission
buoy = buoy_mission.BuoyMission(target)
buoy.run()
buoy.cleanup()

print("[INFO] Buoy mission terminate")


print("[INFO] Mission run terminate")

time.sleep(1.0)

disarm.disarm()
