import numpy as np

class SimpleEKF:
    def __init__(self, process_noise_std, measurement_noise_dict):
        self.dt = 0.01  # Default timestep

        # State: [x, y, vx, vy, yaw]
        self.x = np.zeros((5, 1))
        self.P = np.eye(5)

        # Process noise covariance
        self.Q = np.eye(5) * process_noise_std**2

        # Measurement noise covariance per sensor type
        self.R_dict = {
            'velocity': np.eye(2) * measurement_noise_dict['dvl']**2,
            'angle': np.eye(1) * measurement_noise_dict['fog']**2,
        }

        self.I = np.eye(5)

    def predict(self, imu_accel, yaw_rate, dt=None):
        """
        Predict step uses acceleration and yaw_rate as control inputs.

        imu_accel: np.array([ax, ay]) acceleration in m/s²
        yaw_rate: scalar angular velocity (rad/s)
        """
        dt = dt or self.dt
        self.dt = dt

        ax, ay = imu_accel.flatten()
        omega = yaw_rate  # angular rate

        # State transition model
        F = np.eye(5)
        F[0, 2] = dt  # x position updated by vx
        F[1, 3] = dt  # y position updated by vy
        F[4, 4] = 1   # yaw evolves by integrating angular rate

        # Control input matrix (B)
        B = np.zeros((5, 3))
        # linear acceleration affects vx, vy, and position through integration
        B[0, 0] = 0.5 * dt**2  # x pos due to ax
        B[1, 1] = 0.5 * dt**2  # y pos due to ay
        B[2, 0] = dt           # vx due to ax
        B[3, 1] = dt           # vy due to ay
        B[4, 2] = dt           # yaw due to yaw_rate

        u = np.array([[ax], [ay], [omega]])

        # Predict
        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, H, R):
        """
        Generic EKF update.

        z: measurement vector
        H: measurement matrix
        R: measurement noise covariance
        """
        h_x = H @ self.x
        y = z - h_x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ H) @ self.P

    def update_velocity(self, velocity):
        """
        Update with velocity measurement [vx, vy].
        """
        H = np.zeros((2, 5))
        H[0, 2] = 1
        H[1, 3] = 1
        R = self.R_dict['velocity']
        z = velocity.reshape((2, 1))
        self.update(z, H, R)

    def update_angle(self, angle):
        """
        Update with yaw angle measurement (scalar).
        """
        H = np.zeros((1, 5))
        H[0, 4] = 1
        R = self.R_dict['angle']
        z = np.array([[angle]])
        self.update(z, H, R)

    def get_state(self):
        return self.x.flatten()
