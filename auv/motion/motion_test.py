import rospy
import time
from auv.motion import robot_control2
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control2.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)

rc.set_depth(0.05)
rc.set_mode("MANUAL")
#first_time = time.time()


#first_time = time.time()
#while time.time() - first_time < 3:
#    rc.movement(forward = 2)

#first_time = time.time()
#while time.time() - first_time < 3:
#    rc.movement(forward = -2)

#rc.set_relative_depth(0.1)

#time.sleep(5)

#rc.set_relative_depth(-0.1)

time.sleep(5)
rc.button_press(256)

first_time = time.time()
while time.time() - first_time < 10:
    rc.movement(lateral = 3)

first_time = time.time()
while time.time() - first_time < 10:
    rc.movement(lateral = -3)

time.sleep(1.0)

disarm.disarm()
