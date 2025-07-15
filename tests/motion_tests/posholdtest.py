import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm
if __name__ == "__main__":
    try:
        rospy.init_node("poshold_test", anonymous=True)  # avoid hiearchy issue
        rc = robot_control.RobotControl()

        # rospy.loginfo(f"current state: {rc.orientation}")

        rc.set_control_mode("p_control")
        arm.arm()
        time.sleep(3.0)
        print("[INFO}This is the start")
        # rc.set_absolute_z(0.5)
        rc.set_absolute_z(0.5)
        rospy.loginfo("starting...")
        time.sleep(5)

        rc.set_absolute_yaw(180)
        time.sleep(10)

        rc.reset()

        # rospy.loginfo(f"current state: {rc.orientation}")
        # rc.set_control_mode('direct')
        # time.sleep(1)

        # rc.movement(yaw=1)
        # rospy.loginfo("yaw at pwm=1")
        # time.sleep(10)

        # rc.set_control_mode('pid')
        # rospy.loginfo("Set mode back to pid")
        # time.sleep(1)

        rc.set_absolute_z(2)
        # rospy.loginfo("set absolute depth to 2")
        # time.sleep(20)

        # rc.set_absolute_yaw(180)
        # rc.set_absolute_x(1.7)
        # rc.set_absolute_y(2)
        # rc.set_absolute_z(0.8)
        # rospy.loginfo("set absolute x to 2, y to 3, z to 0.8, heading to 153")
        # time.sleep(10)

        # rospy.loginfo(f"current state: {rc.orientation}")
        rospy.loginfo("Reached the end of the program")
        disarm.disarm()
    except KeyboardInterrupt:
        disarm.disarm()
        
