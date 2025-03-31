import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()


# rc.get_callback_compass()

arm.arm()

time.sleep(5)

while True:
    user_input = input("Give an integer heading, press q to quit: ")
    if user_input == "q":
        disarm.disarm()
    else:
        rc.set_heading(int(user_input))
