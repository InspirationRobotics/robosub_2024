from tests.fusion.EKF import SimpleEKF
import numpy as np

ekf = SimpleEKF(
    process_noise_std=0.1,
    measurement_noise_dict={'imu': 0.2, 'dvl': 0.3, 'fog': 0.05}
)

dt = 0.01
imu_accel = np.array([0.1, 0.0])  # m/s²
yaw_rate = 0.01  # rad/s

# Predict with accel + yaw_rate
ekf.predict(imu_accel, yaw_rate, dt)

# Update with velocity measurement
velocity_meas = np.array([1.2, -0.3])
ekf.update_velocity(velocity_meas)

# Update with angle measurement
yaw_angle_meas = 0.5
ekf.update_angle(yaw_angle_meas)

print("State:", ekf.get_state())
