'''
This file contains an implementation of an Extended Kalman Filter that will be used to 
fuse camera info from orbslam3d, velocity from DVL, and acceleration and quaternions
from the IMU. Many of the functions were taken from the sensorfuse_kf.py file
under compass/altimu10v5. 
'''
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from auv.device.dvl import DVL
import time
# from geometry_msgs.msg import PoseStamped
# import rospy
# from sensor_msgs.msg import Imu


import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from auv.device.dvl import DVL
import time
# from geometry_msgs.msg import PoseStamped
# import rospy
# from sensor_msgs.msg import Imu


class PoseEKF:
    def __init__(self, dvl: DVL, use_simulated_data=False, simulated_data=None):
        self.use_simulated_data = use_simulated_data
        self.simulated_data = simulated_data  # Assign simulated_data to self.simulated_data

        # Initialize camera data storage
        self.camera_pose = np.zeros(7)  # [x, y, z, qw, qx, qy, qz]

        # Initialize DVL data storage
        self.dvl_velocity = np.zeros(3)  # [vx, vy, vz]

        # Initialize IMU data storage
        self.imu = {"ax": 0, "ay": 0, "az": 0, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}
        
        if not self.use_simulated_data:
            # Initialize ROS node if not using simulated data
            rospy.init_node('pose_ekf', anonymous=True)

            # Initialize publisher for estimated pose
            self.pose_pub = rospy.Publisher("/estimated_pose", PoseStamped, queue_size=10)

            # Subscribe to ORB-SLAM3 pose topic
            rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.camera_callback)

            # Subscribe to IMU topic
            rospy.Subscriber("/imu/data", Imu, self.imu_callback)

            self.start_imu_listener()

        # Initialize filter, DVL, IMU, dt, and last_time
        self.dt = 1.0 / 100  # IMU time step (100 Hz)
        self.ekf = self.create_filter()
        self.DVL = dvl
        self.last_time = time.time()

    def camera_callback(self, msg):
        """
        Callback for ORB-SLAM3 camera pose.
        """
        if isinstance(msg, dict):  # Handle dictionary input
            self.camera_pose = np.array([
                msg['position']['x'],
                msg['position']['y'],
                msg['position']['z'],
                msg['orientation']['w'],
                msg['orientation']['x'],
                msg['orientation']['y'],
                msg['orientation']['z']
            ])
        else:  # Handle ROS PoseStamped message
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
        if isinstance(msg, dict):  # Handle dictionary input
            self.imu["ax"] = msg['linear_acceleration']['x']
            self.imu["ay"] = msg['linear_acceleration']['y']
            self.imu["az"] = msg['linear_acceleration']['z']
            self.imu["qw"] = msg['orientation']['w']
            self.imu["qx"] = msg['orientation']['x']
            self.imu["qy"] = msg['orientation']['y']
            self.imu["qz"] = msg['orientation']['z']
        else:  # Handle ROS Imu message
            self.imu["ax"] = msg.linear_acceleration.x
            self.imu["ay"] = msg.linear_acceleration.y
            self.imu["az"] = msg.linear_acceleration.z
            self.imu["qw"] = msg.orientation.w
            self.imu["qx"] = msg.orientation.x
            self.imu["qy"] = msg.orientation.y
            self.imu["qz"] = msg.orientation.z

    def update_dvl(self):
        # Read DVL data
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



    def create_filter(self) -> ExtendedKalmanFilter:
        """
        Initialize the Extended Kalman Filter.
        """
        # Initialize EKF with state vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        ekf = ExtendedKalmanFilter(dim_x=10, dim_z=3, dim_u=0)

        # Initialize state vector
        ekf.x = np.zeros(10)
        ekf.x[6] = 1.0  # Initialize quaternion to identity (qw = 1)

        # Initialize covariance matrix
        ekf.P = np.eye(10) * 1000  # High initial uncertainty

        # Process noise covariance matrix
        ekf.Q = np.eye(10) * 0.1  # Tune based on sensor noise

        # Measurement noise covariance matrix
        ekf.R = np.diag([
            10, 10, 10,  # Position noise (x, y, z)
            100, 100, 100, 100,  # Quaternion noise (qw, qx, qy, qz)
            0.1, 0.1, 0.1  # Velocity noise (vx, vy, vz)
        ])

        # Assign process and measurement models
        ekf.f = self.process_model
        ekf.F = self.process_jacobian(ekf.x, self.dt)  # Compute F as a NumPy array
        ekf.hx = self.measurement_model
        ekf.H = self.measurement_jacobian(ekf.x)  # Compute H as a NumPy array

        return ekf

    def process_model(self, x, dt):
        """
        Nonlinear process model for EKF.
        """
        # Extract state variables
        position = x[:3]
        velocity = x[3:6]
        quaternion = x[6:10]

        # Predict position using velocity
        position_new = position + velocity * dt

        # Predict velocity using acceleration
        acceleration_world = self.quaternion_rotate(quaternion, np.array([self.imu["ax"], self.imu["ay"], self.imu["az"]]))
        velocity_new = velocity + acceleration_world * dt

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
        Combined measurement model for camera (position + orientation) and DVL (velocity).
        """
        # Extract position, quaternion, and velocity from the state vector
        position = x[:3]
        quaternion = x[6:10]
        velocity = x[3:6]  # Velocity in the world frame

        # Combine camera and DVL measurements
        return np.concatenate([position, quaternion, velocity])

    def measurement_jacobian(self, x):
        """
        Combined Jacobian for camera (position + orientation) and DVL (velocity).
        """
        # Initialize the Jacobian matrix (10x10)
        H = np.zeros((10, 10))

        # Camera part: Position and orientation
        H[:3, :3] = np.eye(3)  # Position part
        H[3:7, 6:10] = np.eye(4)  # Quaternion part

        # DVL part: Velocity
        H[7:10, 3:6] = np.eye(3)  # Velocity part

        return H

    def get_combined_measurement(self):
        """
        Combine camera and DVL measurements into a single measurement vector.
        """
        return np.concatenate([self.camera_pose, self.dvl_velocity])

    def update_filter(self):
        """
        Update the EKF with the latest sensor data.
        """
        # Check for valid sensor data
        if np.any(np.isnan(self.camera_pose)) or np.any(np.isnan(self.dvl_velocity)) or np.any(np.isnan(self.imu["qw"])):
            rospy.logwarn("Invalid sensor data detected. Skipping update.")
            return

        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update DVL data
        self.update_dvl()

        # Update the state transition matrix (F) with the current time step
        self.ekf.F = self.process_jacobian(self.ekf.x, dt)

        # Predict the next state
        self.ekf.predict()

        # Update the filter with combined measurements
        combined_measurement = self.get_combined_measurement()
        self.ekf.update(combined_measurement, self.measurement_jacobian, self.measurement_model)

        # Publish the estimated pose
        if not self.use_simulated_data:
            self.publish_pose()

    def get_estimated_velocity(self):
        """
        Return the estimated velocity (x, y, z).
        """
        return self.ekf.x[3:6]

    def get_position_uncertainty(self):
        """
        Return the position uncertainty (standard deviation) from the EKF's covariance matrix.
        """
        position_variance = np.diag(self.ekf.P)[:3]  # First 3 elements correspond to position (x, y, z)
        position_uncertainty = np.sqrt(position_variance)  # Convert variance to standard deviation
        return position_uncertainty
    
