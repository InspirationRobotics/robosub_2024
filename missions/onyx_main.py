"""
Script to run the planned missions for Onyx sequentially.
"""

import time
import signal

from . import PreQualification_Mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems import modems_api

import rospy

config = deviceHelper.variables
RobotControl = robot_control.RobotControl()

missionsNode = rospy.init_node("Missions", anonymous = True)

def onExit(signum, frame):
    """
    Function for cleanly exiting the script.

    Args:
        signum (int): Signal number.
        frame (frame): Current stack frame.
    """
    try:
        print("Exiting...")
        missionsNode.stop()
        time.sleep(3)
        rospy.signal_shutdown("Rospy Exited.")

        while rospy.not_shutdown():
            pass

        print("\n\nCleanly Exited.")
        exit(1)

    except:
        pass

# When Ctrl + C is pressed, exit by calling onExit().
signal.SIGINT(signal.SIGINT, onExit) 

arm.arm()

prequalification = PreQualification_Mission.PreQualificationMission()
time.sleep(1)

prequalification.run()
prequalification.cleanup()

disarm.disarm()

