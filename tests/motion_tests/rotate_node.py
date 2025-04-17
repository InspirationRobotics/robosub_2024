import rospy
from geometry_msgs.msg import TwistStamped

class AUV():
    def __init__(self):
        rospy.init_node("yaw_node", anonymous=True)
        self.pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel ",TwistStamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        while not rospy.is_shutdown():
            vel = TwistStamped()
            vel.header.stamp = rospy.Time.now()
            vel.header.frame_id = "base_link"
            vel.twist.linear.x = 0.5  # move forward
            vel.twist.linear.y = 0.0
            vel.twist.linear.z = 0.0
            vel.twist.angular.x = 0.0
            vel.twist.angular.y = 0.0
            vel.twist.angular.z = 0.0
            self.pub.publish(vel)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        auv = AUV()
        auv.run()
    except rospy.ROSInterruptException:
        
