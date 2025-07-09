import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm
if __name__ == "__main__":
    rospy.init_node("poshold_test", anonymous=True)  # avoid hiearchy issue
    rc = robot_control.RobotControl(debug=True)

    rc.set_control_mode("pid")
    arm.arm()
    time.sleep(3.0)
    print("[INFO}This is the start")
    rc.set_absolute_z(0)
    rospy.loginfo("set absolute depth to 0")

    time.sleep(10)
    rc.set_absolute_x(15)
    rospy.loginfo("set absolute x to 15")

    time.sleep(10)

    rc.set_control_mode('direct')
    time.sleep(10)
    print("[INFO] Reached the end of the program")

    disarm.disarm()
