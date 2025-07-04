import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL
import time
import threading
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, TwistStamped, Vector3Stamped
from geometry_msgs.msg import PointStamped


class SensorFuse:
    def __init__(self):
        # This is the ekf node that takes DVL and imu data and give estimation of velocity
        # Initialize node
        rospy.init_node('ekfNode', anonymous=True)
        self.pub = rospy.Publisher('/auv/state/position', PointStamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        

        # Create subscriber for imu and dvl
        # TODO: Fix IMU rostopic architecture
        self.imu_sub = rospy.Subscriber("/auv/device/vectornav", Vector3, self.imu_callback)
        self.imu_data = {"ax": 0, "ay": 0, "az": 0}  # store one line of IMU data for ekf predict
        self.imu_array = None # used for passing into the ekf

        self.dvl_sub = rospy.Subscriber("/auv/device/dvl/velocity", Vector3Stamped, self.dvl_callback)
        self.dvl_data = {"vx": 0, "vy": 0, "vz": 0}
        self.dvl_array = None # used for passing into the ekf
        # initialize filter, dvl, imu, dt, and last_time
        # initialize dt before creating the filter
        self.dt = 1.0 / 100  # IMU time step (100 Hz)
        self.ekf = self.create_filter()
        self.imu = {"ax": 0, "ay": 0, "az": 0}
        # tracks the cumulative position
        self.position = np.array([0.0, 0.0, 0.0])
        self.last_time = time.time()

    def imu_callback(self,msg):
        self.imu_data["ax"] = msg.linear_acceleration.x
        self.imu_data["ay"] = msg.linear_acceleration.y
        self.imu_data["az"] = msg.linear_acceleration.z
        self.imu_array = np.array([self.imu_data["ax"], self.imu_data["ay"], self.imu_data["az"]])
        # update state
        self.update_state()

    def dvl_callback(self,msg):
        self.dvl_data["vx"] = msg.twist.linear.x
        self.dvl_data["vy"] = msg.twist.linear.y
        self.dvl_data["vz"] = msg.twist.linear.z
        self.dvl_array = np.array([self.dvl_data["vx"], self.dvl_data["vy"], self.dvl_data["vz"]])
        # update filter
        self.update_filter()

    def update_state(self):
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update the state transition matrix F with the new dt
        self.ekf.F = self.FJacobian_at(self.ekf.x, dt)

        # Update the state with IMU data
        self.ekf.x[3:] = self.imu_array

        # Predict the next state
        self.ekf.predict()

        # Integrate position and publish it
        self.position += dt * np.array(self.ekf.x[0:3])
        self.publish()

    def update_filter(self):
        # Update the filter with the latest DVL measurements
        self.ekf.update(self.dvl_array, self.HJacobian_at, self.hx)
    

    def publish(self):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "base_link"  # or "odom", "base_link", etc.

        point_msg.point.x = self.position[0]
        point_msg.point.y = self.position[1]
        point_msg.point.z = self.position[2]

        self.pub.publish(point_msg)
    
    @staticmethod
    def f(x, dt):
        # Non-linear state transition matrix
        # predicts the next state based on the current state and time step
        x_new = np.copy(x)
        x_new[0] += x[3] * dt  # vel_x update
        x_new[1] += x[4] * dt  # vel_y update
        x_new[2] += x[5] * dt  # vel_z update
        return x_new

    @staticmethod
    def FJacobian_at(x, dt):
        # Linearizes f(x)
        return np.array([
            [1., 0., 0., dt, 0., 0.],  # dvl_vel_x depends on vx and ax
            [0., 1., 0., 0., dt, 0.],  # dvl_vel_y depends on vy and ay
            [0., 0., 1., 0., 0., dt],  # dvl_vel_z depends on vz and az
            [0., 0., 0., 1., 0., 0.],  # imu_accel_x depends on ax
            [0., 0., 0., 0., 1., 0.],  # imu_accel_y depends on ay
            [0., 0., 0., 0., 0., 1.]   # imu_accel_z depends on az
        ])

    @staticmethod
    def hx(x):
        # non-linear measurement matrix
        return np.array([x[0], x[1], x[2]])  # extracting velocity from the state vector

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
        # and 3 measurements (vel_x, vel_y, vel_z)
        ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3, dim_u=0)

        # Init everything to 0
        # These are the sensor measurements
        ekf.x = np.array([0., 0., 0., 0., 0., 0.])

        # Create non-linear state transition matrix
        # Each row corresponds to a measurement from sensors
        # Each column indicates the dependence of the measurement on a sensor measurement
        # ie. Since Vx depends on vx dvl and ax imu we put 1's in the vx dvl and ax imu cols
        # 1's for acceleration are placeholder for dt which you'll see in the predict class
        ekf.f = self.f
        ekf.F = self.FJacobian_at(ekf.x, self.dt)

        # Create the non-linear measurement matrix
        # Each row corresponds to a predicted val so vx vy vz
        # Each col corresponds to the sensor vals
        # Convert the predicted state estimate into predicted sensor values so just pulling out the vel values
        ekf.hx = self.hx
        ekf.H = self.HJacobian_at(ekf.x)

        # Covariance matrix (P): initial uncertainty in the state
        # Covariance matrix uncertainty will change over time to be more certain
        ekf.P = np.eye(6) * 1000  # High uncertainty at the start

        # Create the process noise covariance matrix
        # model noise so that the predict model can account for it (can be tuned thru trial and error)
        # Large Q means trusting actual sensor observations more than predicted measurements
        ekf.Q = np.eye(6)

        # Create the measurement noise matrix
        # uncertainty in the predicted vals (can be tuned)
        ekf.R = np.array([
            [0.1, 0., 0.],  # vel_x
            [0., 0.1, 0.],  # vel_y
            [0., 0., 0.1]   # vel_z
        ])


        return ekf


if __name__=="__main__":
    ekf = SensorFuse()
    time.sleep(2)
    rospy.loginfo("ekf node running")
    rospy.spin()