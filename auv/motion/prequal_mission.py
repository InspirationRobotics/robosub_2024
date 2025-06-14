import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("PrequalRun", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)

rc.set_depth(0.5)
rc.set_heading(target = 0,  heading_sensor="vectornav_imu") # measure the heading during test

# unit test each rotation and forward first, comment each out
first_time = time.time()
while time.time() - first_time < 6:
    rc.movement(forward = 3)

time.sleep(1.0)
first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(lateral = -3)
    
time.sleep(1.0)
first_time = time.time()
while time.time() - first_time < 6:
    rc.movement(forward = -3)

time.sleep(1.0)

disarm.disarm()
