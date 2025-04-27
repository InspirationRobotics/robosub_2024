import rospy
import std_msgs
import geometry_msgs
import time

def positionCallback(prev, curr, dt):
    return ((prev + curr) / 2) * dt
class Localize:
    def __init__(self):
        rospy.init_node('localize', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # define subscribers, publishers
        #self.sub_heading    = rospy.Subscriber('/auv/devices/compass', std_msgs.msg.Float32, self.heading_callback)
        self.sub_imu        = rospy.Subscriber('/auv/devices/imu',std_msgs.msg.Imu,self.imuCallback)
        self.pub_vel        = rospy.Publisher('/auv/devices/twist',geometry_msgs.TwistStamped, queue_size=10) 
        self.pub_pos        = rospy.Publisher('/auv/devices/pose',geometry_msgs.PoseStamped, queue_size=10)



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

        if self.imu_last_time is None:
            self.imu_last_time = current_time
            self.acc_x = msg.linear_acceleration.x
            self.acc_y = msg.linear_acceleration.y
            self.acc_z = msg.linear_acceleration.z
            return

        dt = current_time - self.imu_last_time

        # X-axis
        prev_vel = self.imu_vel_x
        self.imu_vel_x += (self.acc_x + msg.linear_acceleration.x) * dt / 2
        self.imu_pos_x += positionCallback(prev_vel, self.imu_vel_x, dt)

        # Y-axis
        prev_vel = self.imu_vel_y
        self.imu_vel_y += (self.acc_y + msg.linear_acceleration.y) * dt / 2
        self.imu_pos_y += positionCallback(prev_vel, self.imu_vel_y, dt)

        # Z-axis
        prev_vel = self.imu_vel_z
        self.imu_vel_z += (self.acc_z + msg.linear_acceleration.z) * dt / 2
        self.imu_pos_z += positionCallback(prev_vel, self.imu_vel_z, dt)

        # Update last acceleration and time
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z
        self.imu_last_time = current_time



    def run(self):
        while not rospy.is_shutdown():
            # update position data to pose
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.stamp = rospy.Time.now()  # In ROS 1
            pose.header.frame_id = "map" 
            pose.position.x = self.imu_pos_x
            pose.position.y = self.imu_pos_y
            pose.position.z = self.imu_pos_z
            # TODO add orientation in pose
            self.pub_pos.publish(pose)

            #twist = geometry_msgs.msg.TwistStamped()

            self.rate.sleep()


if __name__ == "__main__":
    localize = Localize()
    try:
        localize.run()
    except rospy.ROSInterruptException:
        pass