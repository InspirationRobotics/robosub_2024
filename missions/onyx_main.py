"""
To create a sequential order of missions for Onyx to follow.

Onyx will complete the Coin Flip, Gate, and Buoy missions.
"""

import rospy
import time

from auv.mission import gate_mission, buoy_mission, style_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Onyx", anonymous = True)

rc = robot_control.RobotControl()

target = "Red"
gate_heading = 230

arm.arm()

time.sleep(5)

# Rotate towards the heading of the gate, move 2 meters forward
rc.set_heading(gate_heading)

# Run the gate mission
gate = gate_mission.GateMission(target)
gate.run()
gate.cleanup()
rc.set_heading(gate_heading)

# Run the buoy mission
buoy = buoy_mission.BuoyMission(target)
buoy.run()
buoy.cleanup()

print("[INFO] Mission run terminate")

time.sleep(1.0)

disarm.disarm()
