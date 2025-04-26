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
        self.sub_heading    = rospy.Subscriber('/auv/devices/compass', std_msgs.msg.Float32, self.heading_callback)
        self.sub_imu        = rospy.Subscriber('/auv/devices/imu',std_msgs.msg.Imu)
        
        self.imu_last_time = None

        self.acc_x = None
        self.acc_y = None
        self.acc_z = None

        self.vel_x:float = None
        self.vel_y:float = None
        self.vel_z:float = None

        self.pos_x:float = None
        self.pos_y:float = None
        self.pos_z:float = None
        
    
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
        prev_vel = self.vel_x
        self.vel_x += (self.acc_x + msg.linear_acceleration.x) * dt / 2
        self.pos_x += positionCallback(prev_vel, self.vel_x, dt)

        # Y-axis
        prev_vel = self.vel_y
        self.vel_y += (self.acc_y + msg.linear_acceleration.y) * dt / 2
        self.pos_y += positionCallback(prev_vel, self.vel_y, dt)

        # Z-axis
        prev_vel = self.vel_z
        self.vel_z += (self.acc_z + msg.linear_acceleration.z) * dt / 2
        self.pos_z += positionCallback(prev_vel, self.vel_z, dt)

        # Update last acceleration and time
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z
        self.imu_last_time = current_time


    def run(self):
        while not rospy.is_shutdown():
            # update position data to pose
            
            last_time = time.time()  # update last_time
            self.rate.sleep()