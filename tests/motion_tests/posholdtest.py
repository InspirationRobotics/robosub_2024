import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm
if __name__ == "__main__":
    try:
        rospy.init_node("poshold_test", anonymous=True)  # avoid hiearchy issue
        rc = robot_control.RobotControl()

        # rospy.loginfo(f"current state: {rc.orientation}")

        rc.set_control_mode("depth_hold")
        arm.arm()
        time.sleep(3.0)
        print("[INFO}This is the start")
        # rc.set_absolute_z(0.5)
        rc.set_absolute_z(0.5)
        rospy.loginfo("starting...")
        time.sleep(10)

        rc.set_absolute_yaw(90)
        time.sleep(10)
        rc.reset()
        rospy.loginfo("Reached the end of the program")
        rc.exit()
        disarm.disarm()
    except KeyboardInterrupt:
        disarm.disarm()
        
