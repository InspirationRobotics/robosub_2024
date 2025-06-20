import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("PrequalRunHeadingControl", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)

rc.set_depth(0.8)
rc.set_heading(target = 0,  heading_sensor="vectornav_imu") # measure the heading during test

rc.movementWithHeadingControl(desired_heading = 0, time = 20.0, movement_type = "forward", power = 2)
rc.movementWithHeadingControl(desired_heading = 0, time = 8.0, movement_type = "lateral", power = -1)
rc.movementWithHeadingControl(desired_heading = 0, time = 18.0, movement_type = "forward", power = -2)

time.sleep(1.0)

disarm.disarm()
