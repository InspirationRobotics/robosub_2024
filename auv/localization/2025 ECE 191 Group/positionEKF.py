'''
This file contains an implementation of an Extended Kalman Filter that will be used to 
fuse camera info from orbslam3d, velocity from DVL, acceleration from IMU, and 
quaternions from gyroscope data. Much of the functions can be taken from the sensorfuse_kf.py file
under compass/altimu10v5. 
'''
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL
import time
from geometry_msgs.msg import PoseStamped
import threading
import rospy
from sensor_msgs.msg import Imu


class PoseEKF:
    def __init__(self, use_simulated_data=False):
        self.use_simulated_data = use_simulated_data

        if not self.use_simulated_data:
            # Initialize ROS node if not using simulated data
            rospy.init_node('pose_ekf', anonymous=True)

            # Initialize publisher for estimated pose
            self.pose_pub = rospy.Publisher("/estimated_pose", PoseStamped, queue_size=10)

            # Initialize camera data storage
            self.camera_pose = np.zeros(7)  # [x, y, z, qw, qx, qy, qz]

            # Subscribe to ORB-SLAM3 pose topic
            rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.camera_callback)

            # Initialize IMU data storage
            self.imu_acceleration = np.zeros(3)  # [ax, ay, az]
            self.imu_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]

            # Subscribe to IMU topic
            rospy.Subscriber("/imu/data", Imu, self.imu_callback)

            # Initialize DVL data storage
            self.dvl_velocity = np.zeros(3)  # [vx, vy, vz]

            # Subscribe to DVL topic
            rospy.Subscriber("/dvl/velocity", Imu, self.dvl_callback)

        # Initialize EKF
        self.dt = 1.0 / 100  # Time step (100 Hz)
        self.ekf = self.create_filter()
        self.last_time = time.time()

    def camera_callback(self, msg):
        """
        Callback for ORB-SLAM3 camera pose.
        """
        # Extract position and quaternion
        self.camera_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])

    def imu_callback(self, msg):
        """
        Callback for IMU data.
        """
        # Extract linear acceleration
        self.imu_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Extract orientation (quaternion)
        self.imu_quaternion = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def dvl_callback(self, msg):
        """
        Callback for DVL velocity data.
        """
        # Extract velocity
        self.dvl_velocity = np.array([
            msg.linear_velocity.x,
            msg.linear_velocity.y,
            msg.linear_velocity.z
        ])

    def create_filter(self) -> ExtendedKalmanFilter:
        """
        Initialize the Extended Kalman Filter.
        """
        # Initialize EKF with state vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        ekf = ExtendedKalmanFilter(dim_x=10, dim_z=7, dim_u=0)

        # Initialize state vector
        ekf.x = np.zeros(10)
        ekf.x[6] = 1.0  # Initialize quaternion to identity (qw = 1)

        # Initialize covariance matrix
        ekf.P = np.eye(10) * 1000  # High initial uncertainty

        # Process noise covariance matrix
        ekf.Q = np.eye(10) * 0.1  # Tune based on sensor noise

        # Measurement noise covariance matrix
        ekf.R = np.diag([
            0.1, 0.1, 0.1,  # Position noise (x, y, z)
            0.01, 0.01, 0.01, 0.01  # Quaternion noise (qw, qx, qy, qz)
        ])

        # Assign process and measurement models
        ekf.f = self.process_model
        ekf.F = self.process_jacobian
        ekf.hx = self.measurement_model
        ekf.H = self.measurement_jacobian

        return ekf

    def process_model(self, x, dt):
        """
        Nonlinear process model.
        """
        # Extract state variables
        position = x[:3]
        velocity = x[3:6]
        quaternion = x[6:10]

        # Subtract gravity from IMU acceleration (assuming IMU is in body frame)
        gravity = np.array([0, 0, 9.81])  # Adjust based on your coordinate system
        acceleration_body = self.imu_acceleration - gravity

        # Transform acceleration to world frame using quaternion
        acceleration_world = self.quaternion_rotate(quaternion, acceleration_body)

        # Predict velocity using acceleration
        velocity_new = velocity + acceleration_world * dt

        # Predict position using velocity
        position_new = position + velocity * dt

        # Predict quaternion (assume no change in orientation)
        quaternion_new = quaternion
        quaternion_new = quaternion_new / np.linalg.norm(quaternion_new)  # Normalize quaternion

        # Return the new state
        return np.concatenate([position_new, velocity_new, quaternion_new])

    @staticmethod
    def quaternion_rotate(q, v):
        """
        Rotate a vector v by a quaternion q.
        """
        qw, qx, qy, qz = q
        vx, vy, vz = v
        return np.array([
            (1 - 2 * qy**2 - 2 * qz**2) * vx + (2 * qx * qy - 2 * qz * qw) * vy + (2 * qx * qz + 2 * qy * qw) * vz,
            (2 * qx * qy + 2 * qz * qw) * vx + (1 - 2 * qx**2 - 2 * qz**2) * vy + (2 * qy * qz - 2 * qx * qw) * vz,
            (2 * qx * qz - 2 * qy * qw) * vx + (2 * qy * qz + 2 * qx * qw) * vy + (1 - 2 * qx**2 - 2 * qy**2) * vz
        ])

    def process_jacobian(self, x, dt):
        """
        Jacobian of the process model.
        """
        return np.array([
            [1., 0., 0., dt, 0., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., dt, 0., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., dt, 0., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 1.]
        ])

    def measurement_model(self, x):
        """
        Nonlinear measurement model.
        """
        # Extract position and quaternion from the state vector
        position = x[:3]
        quaternion = x[6:10]
        return np.concatenate([position, quaternion])

    def measurement_jacobian(self, x):
        """
        Jacobian of the measurement model.
        """
        H = np.zeros((7, 10))  # 7 measurements (3 position + 4 quaternion)
        H[:3, :3] = np.eye(3)  # Position part
        H[3:7, 6:10] = np.eye(4)  # Quaternion part
        return H

    def update_filter(self):
        """
        Update the EKF with the latest sensor data.
        """
        # Check for valid sensor data
        if np.any(np.isnan(self.camera_pose)) or np.any(np.isnan(self.dvl_velocity)) or np.any(np.isnan(self.imu_quaternion)):
            rospy.logwarn("Invalid sensor data detected. Skipping update.")
            return

        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Predict the next state
        self.ekf.predict()

        # Update the filter with camera measurements
        camera_measurement = self.camera_pose  # [x, y, z, qw, qx, qy, qz]
        self.ekf.update(camera_measurement, self.measurement_jacobian, self.measurement_model)

        # Update the filter with DVL velocity measurements
        dvl_measurement = self.dvl_velocity  # [vx, vy, vz]
        H_dvl = np.zeros((3, 10))  # Jacobian for DVL velocity measurement
        H_dvl[:3, 3:6] = np.eye(3)  # Map velocity state to DVL measurement
        self.ekf.update(dvl_measurement, H_dvl, lambda x: x[3:6])  # Update velocity

        # Update the filter with IMU orientation measurements
        imu_orientation_measurement = self.imu_quaternion  # [qw, qx, qy, qz]
        H_imu = np.zeros((4, 10))  # Jacobian for IMU orientation measurement
        H_imu[:4, 6:10] = np.eye(4)  # Map quaternion state to IMU measurement
        self.ekf.update(imu_orientation_measurement, H_imu, lambda x: x[6:10])  # Update orientation

        # Publish the estimated pose
        if not self.use_simulated_data:
            self.publish_pose()

    def publish_pose(self):
        """
        Publish the estimated pose as a ROS topic.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"

        # Populate position
        pose_msg.pose.position.x = self.ekf.x[0]
        pose_msg.pose.position.y = self.ekf.x[1]
        pose_msg.pose.position.z = self.ekf.x[2]

        # Populate orientation
        pose_msg.pose.orientation.w = self.ekf.x[6]
        pose_msg.pose.orientation.x = self.ekf.x[7]
        pose_msg.pose.orientation.y = self.ekf.x[8]
        pose_msg.pose.orientation.z = self.ekf.x[9]

        # Publish the message
        self.pose_pub.publish(pose_msg)

    def get_estimated_pose(self):
        """
        Return the estimated pose (x, y, z, qw, qx, qy, qz).
        """
        return self.ekf.x[[0, 1, 2, 6, 7, 8, 9]]