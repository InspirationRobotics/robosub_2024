import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm
if __name__ == "__main__":
    rospy.init_node("poshold_test", anonymous=True)  # avoid hiearchy issue
    rc = robot_control.RobotControl(debug=True)

    rospy.loginfo(f"current state: {rc.orientation}")

    rc.set_control_mode("pid")
    arm.arm()
    time.sleep(3.0)
    print("[INFO}This is the start")
    rc.set_absolute_z(3)
    rospy.loginfo("set absolute depth to 0")

    time.sleep(3)
    rc.set_absolute_x(5.5)
    rc.set_absolute_y(4.5)
    rospy.loginfo("set absolute x to 5.5, y to 4.5")

    rc.set_absolute_heading(0)
    rospy.loginfo("set heading to 0")
    time.sleep(3)

    rospy.loginfo(f"current state: {rc.orientation}")
    rc.set_control_mode('direct')
    time.sleep(3)

    rc.movement(yaw=3)
    rospy.loginfo("yaw at pwm=3")
    time.sleep(3)

    rc.set_control_mode('pid')
    rospy.loginfo("Set mode back to pid")
    time.sleep(3)

    rc.set_absolute_heading(153)
    rc.set_absolute_x(2)
    rc.set_absolute_y(3)
    rc.set_absolute_z(0.8)
    rospy.loginfo("set absolute x to 2, y to 3, z to 0.8, heading to 153")
    time.sleep(3)

    rospy.loginfo(f"current state: {rc.orientation}")
    rospy.loginfo("Reached the end of the program")
    disarm.disarm()
