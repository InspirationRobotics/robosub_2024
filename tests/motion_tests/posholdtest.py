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

    time.sleep(3)
    rc.set_absolute_x(0.2)
    rospy.loginfo("set absolute x to 0.2")

    time.sleep(3)

    rc.set_control_mode('direct')
    time.sleep(3)

    rc.movement(yaw=3)
    rospy.loginfo("yaw at pwm=1600")
    time.sleep(3)
    print("[INFO] Reached the end of the program")

    disarm.disarm()
