import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
import time

def positionCallback(prev, curr, dt):
    return ((prev + curr) / 2) * dt

class Localize:
    def __init__(self):
        rospy.init_node('localize_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # define subscribers, publishers
        self.sub_imu = rospy.Subscriber('/auv/devices/imu', Imu, self.imuCallback)
        self.pub_vel = rospy.Publisher('/auv/devices/twist', TwistStamped, queue_size=10)
        self.pub_pos = rospy.Publisher('/auv/devices/pose', PoseStamped, queue_size=10)

        self.imu_last_time = None

        self.acc_x = None
        self.acc_y = None
        self.acc_z = None

        self.imu_vel_x: float = 0.0
        self.imu_vel_y: float = 0.0
        self.imu_vel_z: float = 0.0

        self.imu_pos_x: float = 0.0
        self.imu_pos_y: float = 0.0
        self.imu_pos_z: float = 0.0

    def imuCallback(self, msg):
        current_time = msg.header.stamp.to_sec()
        acc_x_offset = 0.9
        acc_y_offset = 0.35
        acc_z_offset = 9.98

        if self.imu_last_time is None:
            self.imu_last_time = current_time
            self.acc_x = msg.linear_acceleration.x + acc_x_offset
            self.acc_y = msg.linear_acceleration.y + acc_y_offset
            self.acc_z = msg.linear_acceleration.z + acc_z_offset
            return

        dt = current_time - self.imu_last_time

        # X-axis
        prev_vel = self.imu_vel_x
        self.imu_vel_x += (self.acc_x + msg.linear_acceleration.x + acc_x_offset) * dt / 2
        self.imu_pos_x += positionCallback(prev_vel, self.imu_vel_x, dt)

        # Y-axis
        prev_vel = self.imu_vel_y
        self.imu_vel_y += (self.acc_y + msg.linear_acceleration.y + acc_y_offset) * dt / 2
        self.imu_pos_y += positionCallback(prev_vel, self.imu_vel_y, dt)

        # Z-axis
        prev_vel = self.imu_vel_z
        self.imu_vel_z += (self.acc_z + msg.linear_acceleration.z + acc_z_offset) * dt / 2
        self.imu_pos_z += positionCallback(prev_vel, self.imu_vel_z, dt)

        # Update last acceleration and time
        self.acc_x = msg.linear_acceleration.x + acc_x_offset
        self.acc_y = msg.linear_acceleration.y + acc_y_offset
        self.acc_z = msg.linear_acceleration.z + acc_z_offset
        self.imu_last_time = current_time

    def run(self):
        while not rospy.is_shutdown():
            # update position data to pose
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.imu_pos_x
            pose.pose.position.y = self.imu_pos_y
            pose.pose.position.z = self.imu_pos_z

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            self.pub_pos.publish(pose)

            # Also publish velocity if you want
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.header.frame_id = "map"
            twist.twist.linear.x = self.imu_vel_x
            twist.twist.linear.y = self.imu_vel_y
            twist.twist.linear.z = self.imu_vel_z
            self.pub_vel.publish(twist)

            self.rate.sleep()

if __name__ == "__main__":
    localize = Localize()
    try:
        rospy.loginfo("Localization node running")
        localize.run()
    except rospy.ROSInterruptException:
        pass
