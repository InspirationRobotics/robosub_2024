import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PoseStamped
from threading import Lock
#!/usr/bin/env python3


class Pose:
    def __init__(self):
        rospy.init_node('pose_node', anonymous=True)

        # Subscribers
        self.imu_sub        = rospy.Subscriber('/auv/devices/imu', Imu, self.imu_callback)
        self.compass_sub    = rospy.Subscriber('/auv/devices/compass', Float64, self.compass_callback)
        self.baro_sub       = rospy.Subscriber('/auv/devices/baro', Float64, self.depth_callback)
        self.fog_sub        = rospy.Subscriber('/auv/devices/fog', Float64, self.depth_callback)
        self.dvl_sub        = rospy.Subscriber('/auv/devices/dvl', Float64, self.depth_callback)
        # Publisher
        self.pose_pub       = rospy.Publisher('/auv/status/pose', PoseStamped, queue_size=10)

        # State variables
        self.orientation = None
        self.heading = None
        self.depth = None
        
        # Lock for thread safety
        self.lock = Lock()

    def imu_callback(self, msg):
        self.orientation = msg.orientation

    def compass_callback(self, msg):
        self.heading = msg.data

    def baro_callback(self, msg):
        self.depth = msg.data

    def fog_callback(sefl, msg):
        pass

    def dvl_callback(self, msg):
        pass
    
    def estimate_pose(self):
        if self.orientation is None or self.heading is None or self.depth is None:
            rospy.logwarn("Waiting for all sensor data...")
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        with self.lock:
            # Use sensor data to estimate pose
            # TODO: Correct between quaternion orientation and euler angles using transforms3d library
            pose.pose.orientation.z = self.heading
            pose.pose.position.z = self.depth
            # Assuming heading is used for yaw (simplified example)
            pose.pose.position.x = 0.0  # Placeholder
            pose.pose.position.y = 0.0  # Placeholder


        self.pose_pub.publish(pose)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.estimate_pose()
            rate.sleep()

if __name__ == '__main__':
    try:
        estimator = Pose()
        estimator.run()
    except rospy.ROSInterruptException:
        pass