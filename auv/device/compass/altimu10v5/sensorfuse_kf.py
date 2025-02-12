import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL

class SensorFuse:
    def __init__(self, *, dvl_port = deviceHelper.dataFromConfig("dvl"), dvl_baudrate : int = 115200):
        self.ekf = self.create_filter()
        self.DVL = DVL()
        self.raw_data = self.DVL.read_onyx
        # self.IMU =
    #some random stuff here


    # to here

# ------------- Extended Kalman Filter ----------------
    def create_filter(self) -> ExtendedKalmanFilter:
        # A Kalman filter with 6 states (dvl_vel_x, dvl_vel_y, dvl_vel_z, imu_accel_x, imu_accel_y, imu_accel_z)
        # and 3 measurements (vel_x, vel_y, vel_z)
        ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3, dim_u=0)
        # Init everything to 0
        ekf.x = np.array([0., 0., 0., 0., 0., 0.])
        # Create state transition matrix
        ekf.F = np.array([  [1., 0., 0., 1., 0., 0.],   #dvl_vel_x
                            [0., 1., 0., 0., 1., 0.],   #dvl_vel_y
                            [0., 0., 1., 0., 0., 1.],   #dvl_vel_z
                            [0., 0., 0., 1., 0., 0.],   #imu_accel_x
                            [0., 0., 0., 0., 1., 0.],   #imu_accel_y
                            [0., 0., 0., 0., 0., 1.]])  #imu_accel_z
        # Create the measurement matrix
        ekf.H = np.array([  [1., 0., 0., 0., 0., 0.],   #vel_x
                            [0., 1., 0., 0., 0., 0.],   #vel_y  
                            [0., 0., 1., 0., 0., 0.]])  #vel_z
        
        # Covariance matrix (P): initial uncertainty in the state
        ekf.P = np.eye(6) * 1000  # High uncertainty at the start

        # Create the process noise matrix
        ekf.Q = np.eye(6) 

        # Create the measurement noise matrix
        ekf.R = np.array([  [0.1, 0., 0.],  # vel_x
                            [0., 0.1, 0.],  # vel_y
                            [0., 0., 0.1]]) # vel_z
        
        return ekf


