"""
Runs the marker droppper mission
"""

import os
import time

from auv.mission import bins
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration of the sub's devices
config = deviceHelper.variables

# Arms the sub (sets mode to autonomous)
arm.arm()
time.sleep(5)

# Create the mission object
bins1 = bins.BinMission(**config)

# Run the mission
bins1.run()

# =Terminate the mission
bins1.cleanup()

# End
print("[INFO] Mission ended")
