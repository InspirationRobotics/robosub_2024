import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL
import time

#Rospy libraries for getting IMU data
import rospy
from sensor_msgs.msg import Imu
import threading



#Documentation on EKF from filterpy can be found here:
#https://filterpy.readthedocs.io/en/latest/kalman/ExtendedKalmanFilter.html
#Example implementation can be found in the associated textbook in the EKF chapter
#To paraphrase: EKF works by linearizing the problem (jacobian) around the current point

#considerations:
#add functionality for onyx and graey
#update frequency of dvl vs imu

class SensorFuse:
    #DVL and IMU are passed in from the robot_control.py class in initialization
    def __init__(self, dvl: DVL):
        #191 Group TODO: Subscribe to Rostopic for IMU
        #figure out slicing for acquiring imu data
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.imu_data = {}

        #self.imu = sliced data

        #initialize filter, dvl, imu, dt, and last_time
        self.ekf = self.create_filter()
        self.DVL = dvl
        self.imu = {"ax" : 0, "ay": 0, "az": 0}
        self.dt = 0
        self.last_time = 0
        
        self.start_imu_listener

    def f(x, dt):
        # Non-linear state transistion matrix
        # predicts the next state based on the current state and time step
        x_new = np.copy(x)
        x_new[0] += x[3] * dt  # vel_x update
        x_new[1] += x[4] * dt  # vel_y update
        x_new[2] += x[5] * dt  # vel_z update
        return x_new

    def FJacobian_at(x, dt):
        # Linearizes f(x) 
        return np.array([   [1., 0., 0., dt, 0., 0.],   #dvl_vel_x depends on vx and ax
                            [0., 1., 0., 0., dt, 0.],   #dvl_vel_y depends on vy and ay
                            [0., 0., 1., 0., 0., dt],   #dvl_vel_z depends on vz and az
                            [0., 0., 0., 1., 0., 0.],   #imu_accel_x depends on ax
                            [0., 0., 0., 0., 1., 0.],   #imu_accel_y depends on ay
                            [0., 0., 0., 0., 0., 1.]])  #imu_accel_z depends on az
    
    def hx(x):
        # non-linear measurement matrix
        return np.array([x[0], x[1], x[2]])  # extracting velocity from the state vector

    def HJacobian_at():
        # Computes the Jacobian matrix H of h(x) with respect to x
        return np.array([   [1., 0., 0., 0., 0., 0.],   #vel_x
                            [0., 1., 0., 0., 0., 0.],   #vel_y  
                            [0., 0., 1., 0., 0., 0.]])  #vel_z

# ------------- Extended Kalman Filter ----------------
    def create_filter(self) -> ExtendedKalmanFilter:
    
        # A Kalman filter with 6 states (dvl_vel_x, dvl_vel_y, dvl_vel_z, imu_accel_x, imu_accel_y, imu_accel_z)
        # and 3 measurements (vel_x, vel_y, vel_z)
        ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3, dim_u=0)
        
        # Init everything to 0
        #These are the sensor measurements
        ekf.x = np.array([0., 0., 0., 0., 0., 0.])

        # Create non-linear state transition matrix
        #Each row corresponds to a measurement from sensors
        #Each column indicates the dependence of the measurement on a sensor measurement
        #ie. Since Vx depends on vx dvl and ax imu we put 1's in the vx dvl and ax imu cols
        #1's for acceleration are placeholder for dt which you'll see in the predict class
        ekf.f = self.f(ekf.x, self.dt)
        ekf.F = self.FJacobian_at(ekf.x, self.dt)
        
        # Create the non-linear measurement matrix
        # Each row corresponds to a predicted val so vx vy vz
        # Each col corresponds to the sensor vals
        # Convert the predicted state estimate into predicted sensor values so just pulling out the vel values
        ekf.hx = self.hx(ekf.x)
        ekf.H = self.HJacobian_at()
        
        # Covariance matrix (P): initial uncertainty in the state
        # Covariance matrix uncertainty will change over time to be more certain
        ekf.P = np.eye(6) * 1000  # High uncertainty at the start

        # Create the process noise covariance matrix
        # model noise so that the predict model can account for it (can be tuned thru trail and error)
        # Large Q means trusting actual sensor observations more than predicted measurements
        ekf.Q = np.eye(6) 

        # Create the measurement noise matrix
        # uncertainty in the predicted vals (can be tuned)
        ekf.R = np.array([  [0.1, 0., 0.],  # vel_x
                            [0., 0.1, 0.],  # vel_y
                            [0., 0., 0.1]]) # vel_z

        #tracks the cumulative position
        self.position = np.zeros(3)

        return ekf

#-----------Note that update functions access raw sensor data------------
    #Obtain updated IMU data
    def imu_callback(self, imu_msg):
        if imu_msg:
            self.imu["ax"] = float(imu_msg.linear_acceleration.x)
            self.imu["ay"] = float(imu_msg.linear_acceleration.y)
            self.imu["az"] = float(imu_msg.linear_acceleration.z)
        return
    
    def start_imu_listener(self):
        """Starts the IMU subscriber and runs rospy.spin() in a separate thread."""
        rospy.init_node('imu_listener', anonymous=True)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        
        # Run ROS spin in a separate thread so it doesn’t block execution
        thread = threading.Thread(target=rospy.spin)
        thread.daemon = True
        thread.start()
    
    #update DVL data
    #desired dvl data can be acquired by doing dvl.vel_rot[0, 1 or 2]?
    def update_dvl(self):
        # Read DVL data
        if self.DVL.sub_type == "onyx":
            self.DVL.read_onyx()
        elif self.DVL.sub_type == "graey":
            self.DVL.read_graey
        # Assuming DVL.vel_rot is a list or array with [vel_x, vel_y, vel_z]
        self.dvl_vel = np.array([self.DVL.vel_rot[0], self.DVL.vel_rot[1], self.DVL.vel_rot[2]])

    def update_filter(self):
        # Calculate time delta
        #grab the current time
        current_time = time.time() 
        #calculate dt, initially last time is 0
        dt = current_time - self.last_time
        #set last time to the current time
        self.last_time = current_time

        #update dvl data
        self.update_dvl

        # Update the state transition matrix F with the new dt
        self.ekf.F = np.array([  [1., 0., 0., dt, 0., 0.],   #dvl_vel_x
                                  [0., 1., 0., 0., dt, 0.],   #dvl_vel_y
                                  [0., 0., 1., 0., 0., dt],   #dvl_vel_z
                                  [0., 0., 0., 1., 0., 0.],   #imu_accel_x
                                  [0., 0., 0., 0., 1., 0.],   #imu_accel_y
                                  [0., 0., 0., 0., 0., 1.]])  #imu_accel_z


        # Update the filter with the latest DVL measurements
        #In more detail this updates the parameters (Kalman gain, x, R, etc...) with the new measurements 
        self.ekf.update(self.dvl_vel, self.H, self.hx(self.ekf.x))

        # Predict the next state
        self.ekf.predict()

        # Update the state with IMU data
        self.ekf.x[3:] = np.array([self.imu["ax"], self.imu["ay"], self.imu["az"]])

        self.position += dt*self.ekf.x[:3]

        return

    def get_estimated_velocity(self):
        # Return the estimated velocity (x, y, z)
        return self.ekf.x[:3]


    def get_position_uncertainty(self):
        """
        Returns the position uncertainty (standard deviation) from the EKF's covariance matrix.
        """
        # The diagonal elements of the covariance matrix represent the variance of the state estimates
        position_variance = np.diag(self.ekf.P)[:3]  # First 3 elements correspond to position (x, y, z)
        position_uncertainty = np.sqrt(position_variance)  # Convert variance to standard deviation
        return position_uncertainty
