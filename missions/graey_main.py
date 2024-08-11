"""
To create a sequential order of missions for Onyx to follow. This is the official mission script.

Onyx will complete the coin flip, go through the "Red" side of the gate, circumnavigate clockwise around the buoy, and then surface in the octagon.
"""

import rospy
import time

from auv.mission import style_mission, buoy_mission, octagon_approach_mission, coms
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper
from auv.device.modems import modems_api

rospy.init_node("Graey", anonymous = True)

rc = robot_control.RobotControl(enable_dvl=False)

target = "Red"
gate_heading = 220

time.sleep(60)

arm.arm()

rc.set_depth(0.7)

time.sleep(5)

# Rotate towards the heading of the gate.
# rc.set_heading(gate_heading)

curr_time = time.time()

while time.time() - curr_time < 21:
    rc.movement(forward=2)


# Run the style mission
# style = style_mission.StyleMission()
# style.run()
# style.cleanup()
# rc.set_heading(gate_heading - 70)

# # curr_time = time.time()
# # while time.time() - curr_time < 5:
# #     rc.movement(forward=2)

# # Run the buoy mission
# buoy = buoy_mission.BuoyMission(target)
# buoy.run()
# buoy.cleanup()
# rc.set_heading(gate_heading + 35)

# # Get to the octagon, our model is short range only
# rc.set_depth(0.38)

# curr_time = time.time()

# while time.time() - curr_time < 17:
#     rc.movement(forward=2.5)

# # Octagon mission
# octagon = octagon_approach_mission.OctagonApproachMission()
# octagon.run()
# octagon.cleanup()

# time.sleep(1.0)

# Coms mission, experimental


try:
    modem = modems_api.Modem()
    modem.send_msg("graey handshake") # Send a modem message
    fail_modem = False
except:
    fail_modem = True
    print("Failed to start modem, starting directly")


print("[INFO] Mission run terminate")

