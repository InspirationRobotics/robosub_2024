
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL
import time
import threading
from statistics import mean
import rospy
from struct import pack, unpack
from mavros_msgs.msg import Mavlink
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from transforms3d.euler import quat2euler


class SensorFuse:
    def __init__(self):
        # This is the ekf node that takes DVL and imu data and give estimation of velocity
        # Initialize node
        rospy.init_node('ekfNode', anonymous=True)
        self.pub = rospy.Publisher('/auv/state/pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.ekf_lock = threading.Lock()
        

        # Create subscriber for imu and dvl
        # TODO: Fix IMU rostopic architecture
        self.imu_sub        = rospy.Subscriber("/auv/devices/vectornav", Imu, self.imu_callback)
        self.imu_acc_data   = {"ax": 0, "ay": 0, "az": 0}
        self.imu_ori_data   = {"qx": 0, "qy": 0, "qz": 0, "qw": 0}  # store one line of IMU data for ekf predict
        self.imu_array = np.zeros((3, 1))  # Before first IMU callback

        self.dvl_sub    = rospy.Subscriber("/auv/devices/dvl/velocity", Vector3Stamped, self.dvl_callback)
        self.dvl_data   = {"vx": 0, "vy": 0, "vz": 0}
        self.dvl_array  = np.zeros((3, 1)) # used for passing into the ekf

        self.baro_sub           = rospy.Subscriber("/mavlink/from", Mavlink, self.barometer_callback)
        self.barometer_depth    = None
        self.depth_calib        = 0
        self.calibrated         = False
        # initialize filter, dvl, imu, dt, and last_time
        # initialize dt before creating the filter
        self.dt = 1.0 / 100  # IMU time step (100 Hz)
        self.ekf = self.create_filter()
        # tracks the cumulative position
        self.position = np.zeros((3, 1))
        self.last_time = time.time()

    def imu_callback(self,msg):
        # (self.imu_data["ax"], self.imu_data["ay"], self.imu_data["az"]) = quat2euler(orientation_list)
        self.imu_acc_data["ax"] = msg.linear_acceleration.x
        self.imu_acc_data["ay"] = msg.linear_acceleration.y
        self.imu_acc_data["az"] = msg.linear_acceleration.z

        self.imu_ori_data['qx'] = msg.orientation.x
        self.imu_ori_data['qy'] = msg.orientation.y
        self.imu_ori_data['qz'] = msg.orientation.z
        self.imu_ori_data['qw'] = msg.orientation.w

        self.imu_array = np.array([
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
        ]).reshape(-1, 1)  # Store as column vector

        # update state
        print("Before updating state, ekf.x shape:", self.ekf.x.shape)
        self.update_state()

    def dvl_callback(self,msg):
        self.dvl_data["vx"] = msg.vector.x
        self.dvl_data["vy"] = msg.vector.y
        self.dvl_data["vz"] = msg.vector.z
        self.dvl_array = np.array([self.dvl_data["vx"], self.dvl_data["vy"], self.dvl_data["vz"]])

        # update filter
        print("Before updating dvl, ekf.x shape:", self.ekf.x.shape)

        self.update_dvl()

    def barometer_callback(self, msg):
        """
        Handles barometric data by unpacking, calculating depth from raw data, then publishes raw data

        Args:
            msg: Barometric data from corresponding publisher
        """
        try:
            # If the barometric data message has the right ID
            if msg.msgid == 143:
                # Unpack the data
                p = pack("QQ", *msg.payload64)
                time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p) # Pressure is in mBar

                # Calculate the depth based on the pressure
                press_diff = round(press_diff, 2)
                press_abs = round(press_abs, 2)
                self.barometer_depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib
            
            if self.calibrated:
                self.update_depth()
        # Handle exceptions
        except Exception as e:
            rospy.logerr("Barometer unpacking failed")
            rospy.logerr(e)

    def calibrate_depth(self, sample_time=3):
        """
        To calibrate the depth data

        Args:
            sample_time (int): The number of seconds taken to calibrate the data        
        """
        rospy.loginfo("Starting Depth Calibration...")
        samples = []

        # Wait for depth data
        while self.barometer_depth == None:
            rospy.sleep(0.1)
            pass


        prevDepth = self.barometer_depth
        start_time = time.time()

        # Collect data for sample_time seconds, then calculate the mean
        while time.time() - start_time < sample_time:
            if self.barometer_depth == prevDepth:
                continue

            samples.append(self.barometer_depth)
            prevDepth = self.barometer_depth

        self.depth_calib = mean(samples)
        self.calibrated = True
        rospy.loginfo(f"depth calibration Finished. Surface is: {self.depth_calib}")

    def update_state(self):
        # Accept both (9,) and (9,1) shapes
        assert self.ekf.x.shape in [(9,), (9,1)], \
            f"ekf.x corrupted: shape {self.ekf.x.shape}"
            
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update the state transition matrix F with the new dt
        self.ekf.F = self.FJacobian_at(self.ekf.x, dt)

        # Update the state with IMU data
        # Ensure imu_array is column vector before assignment
        self.ekf.x[6:] = self.imu_array.reshape(-1, 1)

        # Predict the next state
        self.ekf.predict()


    def update_dvl(self):
        with self.ekf_lock:
            z = self.dvl_array.reshape(-1, 1)  # Keep as 1D vector (3,)
            self.ekf.update(z, self.H_velocity, self.hx_velocity)
            self.position = self.ekf.x[0:3]
            self.publish()
    
    def update_depth(self):
        with self.ekf_lock:
            def h_z(x):
                res = np.array([[x[2,0]]]) 
                return res.reshape(-1,1)  # Return as 1x1 matrix
                
            def H_z(x):
                return np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0]])
                
            z = np.array([[self.barometer_depth]])  # 1x1 matrix
            
            R_z = np.array([0.05])  # 1D measurement noise

            self.ekf.update(z, H_z, h_z, R=R_z)


    def publish(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"  # or "odom", "base_link", etc.

        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        
        pose_msg.pose.orientation.w = self.imu_ori_data['qw']
        pose_msg.pose.orientation.x = self.imu_ori_data['qx']
        pose_msg.pose.orientation.y = self.imu_ori_data['qy']
        pose_msg.pose.orientation.z = self.imu_ori_data['qz']

        self.pub.publish(pose_msg)
        self.rate.sleep()
    
    @staticmethod
    def f(x, dt):
        # Non-linear state transition matrix
        # predicts the next state based on the current state and time step
        # Convert to 1D for calculations
        x_flat = x.ravel()
        x_new = np.zeros(9)
        
        # Position update: p += v * dt + 0.5 * a * dt^2
        x_new[0] += x[3] * dt + 0.5 * x[6] * dt**2
        x_new[1] += x[4] * dt + 0.5 * x[7] * dt**2
        x_new[2] += x[5] * dt + 0.5 * x[8] * dt**2

        # Velocity update: v += a * dt
        x_new[3] += x[6] * dt
        x_new[4] += x[7] * dt
        x_new[5] += x[8] * dt

        # Acceleration assumed constant (no update)
        return x_new.reshape(-1, 1)  # Return as column vector

    @staticmethod
    def FJacobian_at(x, dt):
        # Linearizes f(x) for state: [px, py, pz, vx, vy, vz, ax, ay, az]
        return np.array([
            [1., 0., 0.,  dt, 0., 0., 0.5*dt**2,       0.,        0.],
            [0., 1., 0.,  0., dt, 0.,      0., 0.5*dt**2,        0.],
            [0., 0., 1.,  0., 0., dt,      0.,       0., 0.5*dt**2],
            
            [0., 0., 0.,  1., 0., 0.,      dt,        0.,        0.],
            [0., 0., 0.,  0., 1., 0.,      0.,       dt,        0.],
            [0., 0., 0.,  0., 0., 1.,      0.,        0.,       dt ],
            
            [0., 0., 0.,  0., 0., 0.,      1.,        0.,        0.],
            [0., 0., 0.,  0., 0., 0.,      0.,        1.,        0.],
            [0., 0., 0.,  0., 0., 0.,      0.,        0.,        1.]
        ])
    
    # @staticmethod
    # def FJacobian_at(x, dt):
    #     # Linearizes f(x)
    #     return np.array([
    #         [1., 0., 0., dt, 0., 0.],  # dvl_vel_x depends on vx and ax
    #         [0., 1., 0., 0., dt, 0.],  # dvl_vel_y depends on vy and ay
    #         [0., 0., 1., 0., 0., dt],  # dvl_vel_z depends on vz and az
    #         [0., 0., 0., 1., 0., 0.],  # imu_accel_x depends on ax
    #         [0., 0., 0., 0., 1., 0.],  # imu_accel_y depends on ay
    #         [0., 0., 0., 0., 0., 1.]   # imu_accel_z depends on az
    #     ])

    @staticmethod
    def hx_velocity(x):
        return x[3:6]  # Return slice as column vector (3x1)

    @staticmethod
    def H_velocity(x):
        H = np.zeros((3, 9))
        H[0, 3] = 1
        H[1, 4] = 1
        H[2, 5] = 1
        return H  # shape: (3, 9)
    
    @staticmethod
    def hx(x):
        return x[0:3].reshape(-1, 1)
    
    @staticmethod
    def HJacobian_at(x):
        # Define the Jacobian matrix for the measurement function
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        return H

    # ------------- Extended Kalman Filter ----------------
    def create_filter(self) -> ExtendedKalmanFilter:
        # Full state: [px, py, pz, vx, vy, vz, ax, ay, az]
        ekf = ExtendedKalmanFilter(dim_x=9, dim_z=3, dim_u=0)

        # Initial state (all zeros)
        ekf.x = np.zeros((9, 1))
        print(f"DEBUG: ekf.x is {ekf.x} with shape {ekf.x.shape}")
        # Nonlinear transition and measurement functions
        ekf.f = self.f
        ekf.F = self.FJacobian_at(ekf.x, self.dt)  # output is a numpy array

        ekf.hx = self.hx_velocity  # assume initial measurement function is for velocity (DVL)
        ekf.H = self.H_velocity

        # State covariance (uncertainty in initial state)
        ekf.P = np.eye(9) * 1000.0  # large uncertainty

        # Process noise covariance (how much we trust our model)
        ekf.Q = np.eye(9) * 0.1  # tunable

        # Measurement noise (how noisy are the DVL measurements)
        ekf.R = np.eye(3) * 0.1  # for velocity [vx, vy, vz]

        return ekf



if __name__=="__main__":
    ekf = SensorFuse()
    time.sleep(2)
    ekf.calibrate_depth()
    rospy.loginfo("ekf node running")
    rospy.spin()