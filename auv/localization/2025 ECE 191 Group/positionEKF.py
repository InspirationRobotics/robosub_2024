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


class poseEKF:
    def __init__(self, dvl: DVL, use_simulated_data=False):
        self.use_simulated_data = use_simulated_data

        if not self.use_simulated_data:
            # Initialize ROS node if not using simulated data
            rospy.init_node('pose_ekf', anonymous=True)

            # Initialize camera data storage
            self.camera_position = np.zeros(3)  # [x, y, z]
            self.camera_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]

            # Subscribe to ORB-SLAM3 pose topic
            rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.camera_callback)

            # Initialize IMU data storage
            self.imu_data = {
                "angular_velocity": np.zeros(3),  # [wx, wy, wz]
                "linear_acceleration": np.zeros(3),  # [ax, ay, az]
                "orientation": np.array([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]
            }

            # Subscribe to IMU topic
            rospy.Subscriber("/imu/data", Imu, self.gyro_callback)

        # Initialize filter, DVL, IMU, dt, and last_time
        self.dt = 1.0 / 100  # IMU time step (100 Hz)
        self.ekf = self.create_filter()
        self.DVL = dvl
        self.imu = {"ax": 0, "ay": 0, "az": 0}
        self.last_time = time.time()

        if not self.use_simulated_data:
            self.start_imu_listener()

    def camera_callback(self, msg):
        """
        Callback for ORB-SLAM3 camera pose.
        """
        # Extract position
        self.camera_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Extract orientation (quaternion)
        self.camera_quaternion = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])

    def gyro_callback(self, msg):
        """
        Callback for IMU (gyroscope) data.
        """
        # Extract angular velocity
        self.imu_data["angular_velocity"] = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Extract linear acceleration
        self.imu_data["linear_acceleration"] = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Extract orientation (quaternion)
        self.imu_data["orientation"] = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def start_imu_listener(self):
        """
        Starts the IMU subscriber and runs rospy.spin() in a separate thread.
        """
        thread = threading.Thread(target=rospy.spin)
        thread.daemon = True
        thread.start()

    def f(self, x, dt):
        """
        Nonlinear process model.
        """
        # Extract state variables
        position = x[:3]
        velocity = x[3:6]
        quaternion = x[6:10]
        gyro_bias = x[10:13]

        # Predict position
        position_new = position + velocity * dt

        # Predict velocity (assuming constant velocity model for simplicity)
        velocity_new = velocity

        # Predict quaternion
        omega = self.imu_data["angular_velocity"] - gyro_bias  # Measured angular velocity minus bias
        q_dot = 0.5 * self.quaternion_multiply(quaternion, np.array([0, omega[0], omega[1], omega[2]]))
        quaternion_new = quaternion + q_dot * dt
        quaternion_new = quaternion_new / np.linalg.norm(quaternion_new)  # Normalize quaternion

        # Gyro bias is assumed constant (random walk)
        gyro_bias_new = gyro_bias

        # Return the new state
        return np.concatenate([position_new, velocity_new, quaternion_new, gyro_bias_new])

    @staticmethod
    def FJacobian_at(x, dt):
        """
        Jacobian of the process model.
        """
        return np.array([
            [1., 0., 0., dt, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., dt, 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., dt, 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.]
        ])

    @staticmethod
    def hx(x):
        """
        Nonlinear measurement model.
        """
        # Extract position and quaternion from the state vector
        position = x[:3]
        quaternion = x[6:10]
        return np.concatenate([position, quaternion])

    @staticmethod
    def HJacobian_at(x):
        """
        Jacobian of the measurement model.
        """
        H = np.zeros((7, 13))  # 7 measurements (3 position + 4 quaternion)
        H[:3, :3] = np.eye(3)  # Position part
        H[3:7, 6:10] = np.eye(4)  # Quaternion part
        return H

    def create_filter(self) -> ExtendedKalmanFilter:
        """
        Initialize the Extended Kalman Filter.
        """
        # Initialize EKF with extended state vector
        ekf = ExtendedKalmanFilter(dim_x=13, dim_z=7, dim_u=0)  # 13 states, 7 measurements (position + quaternion)

        # Initialize state vector
        ekf.x = np.zeros(13)
        ekf.x[6] = 1.0  # Initialize quaternion to identity (qw = 1)

        # Initialize covariance matrix
        ekf.P = np.eye(13) * 1000  # High initial uncertainty

        # Process noise covariance matrix
        ekf.Q = np.eye(13) * 0.1  # Tune this based on sensor noise characteristics

        # Measurement noise covariance matrix
        ekf.R = np.eye(7) * 0.1  # Tune this based on sensor noise characteristics

        # Assign process and measurement models
        ekf.f = self.f
        ekf.F = self.FJacobian_at
        ekf.hx = self.hx
        ekf.H = self.HJacobian_at

        return ekf

    def update_dvl(self):
        """
        Update DVL data.
        """
        if self.use_simulated_data:
            # Use simulated data
            self.dvl_vel = np.array([self.DVL.vel_rot[0], self.DVL.vel_rot[1], self.DVL.vel_rot[2]])
        else:
            # Use live data
            if self.DVL.sub_type == "onyx":
                self.DVL.read_onyx()
            elif self.DVL.sub_type == "graey":
                self.DVL.read_graey()
            # Assuming DVL.vel_rot is a list or array with [vel_x, vel_y, vel_z]
            self.dvl_vel = np.array([self.DVL.vel_rot[0], self.DVL.vel_rot[1], self.DVL.vel_rot[2]])

    def update_imu(self):
        """
        Update IMU data.
        """
        self.imu["ax"] = self.imu_data["linear_acceleration"][0]
        self.imu["ay"] = self.imu_data["linear_acceleration"][1]
        self.imu["az"] = self.imu_data["linear_acceleration"][2]

    def update_filter(self):
        """
        Update the EKF with the latest sensor data.
        """
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update DVL data
        self.update_dvl()

        # Update IMU data
        self.update_imu()

        # Update the state transition matrix F with the new dt
        self.ekf.F = self.FJacobian_at(self.ekf.x, dt)

        # Predict the next state
        self.ekf.predict()

        # Update the filter with camera measurements
        camera_measurement = np.concatenate([self.camera_position, self.camera_quaternion])
        self.ekf.update(camera_measurement, self.HJacobian_at(), self.hx(self.ekf.x))

        # Update the state with IMU data
        self.ekf.x[3:6] = np.array([self.imu["ax"], self.imu["ay"], self.imu["az"]])  # Update velocity
        self.ekf.x[10:13] = self.imu_data["angular_velocity"]  # Update gyro bias

        # Update position
        self.position += dt * self.ekf.x[:3]

    def get_estimated_velocity(self):
        """
        Return the estimated velocity (x, y, z).
        """
        return self.ekf.x[:3]

    def get_position_uncertainty(self):
        """
        Return the position uncertainty (standard deviation) from the EKF's covariance matrix.
        """
        position_variance = np.diag(self.ekf.P)[:3]  # First 3 elements correspond to position (x, y, z)
        position_uncertainty = np.sqrt(position_variance)  # Convert variance to standard deviation
        return position_uncertainty

    @staticmethod
    def quaternion_multiply(q1, q2):
        """
        Multiply two quaternions.
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])