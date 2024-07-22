import rospy
from auv.motion import robot_control


rospy.init_node("Heading Test", anonymous=True)
rc = robot_control.RobotControl()


rc.get_callback_compass()

rc.set_heading(90)

