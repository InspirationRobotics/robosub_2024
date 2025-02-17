import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL
from auv.device.pix_standalone import AUV 

#considerations:
#add functionality for onyx and graey
#update frequency of dvl vs imu

class SensorFuse:
    def __init__(self, sub: AUV):
        self.sub = sub
        self.ekf = self.create_filter()
        self.DVL = DVL()
        self.imu = {"ax" : 0, "ay": 0, "az": 0}

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

#-----------Note that update functions access raw sensor data------------
    #Obtain updated IMU data
    #IMU acceleration data can be acquired by 
    def update_imu(self):
        #grab most recent data
        msg = self.sub.TOPIC_GET_IMU_DATA.get_data_last
        #set imu vals = most recent data
        if msg:
            self.imu["ax"] = msg.linear_acceleration.x
            self.imu["ay"] = msg.linear_acceleration.y
            self.imu["az"] = msg.linear_acceleration.z

        return 
    
    #update DVL data
    #desired dvl data can be acquired by doing dvl.vel_rot[0, 1 or 2]?
    def update_dvl(self):
        self.DVL.read_onyx
        return 
    
    #perform prediction
    def update_vel(self):
        #calculate time delta
        #vel to acceleration
        #update filter
        return
