import numpy as np
import pandas as pd
import json
from datetime import datetime
import os

class SensorSimulator:
    """Generates simulated IMU, DVL, and camera data for a square path"""
    def __init__(self, side_length=5.0, imu_rate=100, dvl_rate=10, cam_rate=30):
        self.side_length = side_length
        self.imu_dt = 1.0 / imu_rate
        self.dvl_dt = 1.0 / dvl_rate
        self.cam_dt = 1.0 / cam_rate
        self.cruise_speed = 0.5  # m/s
        self.turn_rate = np.pi/4  # rad/s (45 deg/s)
        
        # Noise parameters
        self.imu_accel_noise_std = 0.1  # m/s^2
        self.imu_gyro_noise_std = 0.01  # rad/s
        self.imu_quat_noise_std = 0.01  # unitless
        self.dvl_vel_noise_std = 0.02   # m/s
        self.dvl_pos_noise_std = 0.01   # m
        self.cam_pose_noise_std = 0.1  # m
        self.cam_quat_noise_std = 0.1  # unitless

        # Initialize storage and state
        self.reset()
        
    def reset(self):
        """Reset simulator state"""
        self.imu_data = []
        self.dvl_data = []
        self.cam_data = []
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = 0.0
        
    def _add_quaternion_noise(self, quat, noise_std):
        """
        Add noise to a quaternion and ensure it remains normalized.
        """
        noise = np.random.normal(0, noise_std, 4)
        noisy_quat = quat + noise
        noisy_quat = noisy_quat / np.linalg.norm(noisy_quat)  # Normalize
        return noisy_quat
        
    def _simulate_straight(self, start_time, direction, duration):
        """Simulate straight line motion"""
        t = start_time
        vel = direction * self.cruise_speed
        
        while t < start_time + duration:
            # IMU data (100 Hz)
            if int(t / self.imu_dt) > int((t - self.imu_dt) / self.imu_dt):
                accel_noise = np.random.normal(0, self.imu_accel_noise_std, 3)
                gyro_noise = np.random.normal(0, self.imu_gyro_noise_std, 3)
                
                # Add noise to IMU quaternion
                true_quat = np.array([
                    np.cos(self.orientation/2),
                    0.0,
                    0.0,
                    np.sin(self.orientation/2)
                ])
                noisy_quat = self._add_quaternion_noise(true_quat, self.imu_quat_noise_std)
                
                self.imu_data.append({
                    'Time': t,
                    'header.seq': int(t * 1000),
                    'header.stamp.secs': int(t),
                    'header.stamp.nsecs': int((t % 1) * 1e9),
                    'header.frame_id': "imu_link",
                    'orientation.x': noisy_quat[1],
                    'orientation.y': noisy_quat[2],
                    'orientation.z': noisy_quat[3],
                    'orientation.w': noisy_quat[0],
                    'angular_velocity.x': gyro_noise[0],
                    'angular_velocity.y': gyro_noise[1],
                    'angular_velocity.z': gyro_noise[2],
                    'linear_acceleration.x': vel[0] + accel_noise[0],
                    'linear_acceleration.y': vel[1] + accel_noise[1],
                    'linear_acceleration.z': vel[2] + accel_noise[2]
                })
            
            # DVL data (10 Hz)
            if int(t / self.dvl_dt) > int((t - self.dvl_dt) / self.dvl_dt):
                vel_noise = np.random.normal(0, self.dvl_vel_noise_std, 3)
                pos_noise = np.random.normal(0, self.dvl_pos_noise_std, 3)
                
                self.dvl_data.append({
                    'time': t,
                    'vx': vel[0] + vel_noise[0],
                    'vy': vel[1] + vel_noise[1],
                    'vz': vel[2] + vel_noise[2],
                    'x': self.position[0] + pos_noise[0],
                    'y': self.position[1] + pos_noise[1],
                    'z': self.position[2] + pos_noise[2],
                    'error': self.dvl_vel_noise_std,
                    'valid': True
                })
            
            # Camera data (30 Hz)
            if int(t / self.cam_dt) > int((t - self.cam_dt) / self.cam_dt):
                pose_noise = np.random.normal(0, self.cam_pose_noise_std, 3)
                
                # Add noise to camera quaternion
                true_quat = np.array([
                    np.cos(self.orientation/2),
                    0.0,
                    0.0,
                    np.sin(self.orientation/2)
                ])
                noisy_quat = self._add_quaternion_noise(true_quat, self.cam_quat_noise_std)
                
                self.cam_data.append({
                    'time': t,
                    'x': self.position[0] + pose_noise[0],
                    'y': self.position[1] + pose_noise[1],
                    'z': self.position[2] + pose_noise[2],
                    'qw': noisy_quat[0],
                    'qx': noisy_quat[1],
                    'qy': noisy_quat[2],
                    'qz': noisy_quat[3]
                })
            
            self.position += vel * self.imu_dt
            t += self.imu_dt
            
        return t
        
    def _simulate_turn(self, start_time, turn_angle, duration):
        """Simulate turning motion"""
        t = start_time
        omega = turn_angle / duration
        
        while t < start_time + duration:
            # IMU data
            if int(t / self.imu_dt) > int((t - self.imu_dt) / self.imu_dt):
                accel_noise = np.random.normal(0, self.imu_accel_noise_std, 3)
                gyro_noise = np.random.normal(0, self.imu_gyro_noise_std, 3)
                
                # Add noise to IMU quaternion
                true_quat = np.array([
                    np.cos(self.orientation/2),
                    0.0,
                    0.0,
                    np.sin(self.orientation/2)
                ])
                noisy_quat = self._add_quaternion_noise(true_quat, self.imu_quat_noise_std)
                
                self.imu_data.append({
                    'Time': t,
                    'header.seq': int(t * 1000),
                    'header.stamp.secs': int(t),
                    'header.stamp.nsecs': int((t % 1) * 1e9),
                    'header.frame_id': "imu_link",
                    'orientation.x': noisy_quat[1],
                    'orientation.y': noisy_quat[2],
                    'orientation.z': noisy_quat[3],
                    'orientation.w': noisy_quat[0],
                    'angular_velocity.x': gyro_noise[0],
                    'angular_velocity.y': gyro_noise[1],
                    'angular_velocity.z': omega + gyro_noise[2],
                    'linear_acceleration.x': accel_noise[0],
                    'linear_acceleration.y': accel_noise[1],
                    'linear_acceleration.z': accel_noise[2]
                })
            
            # DVL data
            if int(t / self.dvl_dt) > int((t - self.dvl_dt) / self.dvl_dt):
                vel_noise = np.random.normal(0, self.dvl_vel_noise_std, 3)
                pos_noise = np.random.normal(0, self.dvl_pos_noise_std, 3)
                
                self.dvl_data.append({
                    'time': t,
                    'vx': vel_noise[0],
                    'vy': vel_noise[1],
                    'vz': vel_noise[2],
                    'x': self.position[0] + pos_noise[0],
                    'y': self.position[1] + pos_noise[1],
                    'z': self.position[2] + pos_noise[2],
                    'error': self.dvl_vel_noise_std,
                    'valid': True
                })
            
            # Camera data
            if int(t / self.cam_dt) > int((t - self.cam_dt) / self.cam_dt):
                pose_noise = np.random.normal(0, self.cam_pose_noise_std, 3)
                
                # Add noise to camera quaternion
                true_quat = np.array([
                    np.cos(self.orientation/2),
                    0.0,
                    0.0,
                    np.sin(self.orientation/2)
                ])
                noisy_quat = self._add_quaternion_noise(true_quat, self.cam_quat_noise_std)
                
                self.cam_data.append({
                    'time': t,
                    'x': self.position[0] + pose_noise[0],
                    'y': self.position[1] + pose_noise[1],
                    'z': self.position[2] + pose_noise[2],
                    'qw': noisy_quat[0],
                    'qx': noisy_quat[1],
                    'qy': noisy_quat[2],
                    'qz': noisy_quat[3]
                })
            
            self.orientation += omega * self.imu_dt
            t += self.imu_dt
            
        return t

    def generate_square_path(self):
        """Generate data for square path"""
        movements = [
            (np.array([0.0, 1.0, 0.0]), 0.0),    # Forward
            (np.array([0.0, 0.0, 0.0]), np.pi/2), # Turn left
            (np.array([-1.0, 0.0, 0.0]), 0.0),   # Left
            (np.array([0.0, 0.0, 0.0]), np.pi/2), # Turn left
            (np.array([0.0, -1.0, 0.0]), 0.0),   # Back
            (np.array([0.0, 0.0, 0.0]), np.pi/2), # Turn left
            (np.array([1.0, 0.0, 0.0]), 0.0),    # Right
        ]
        
        t = 0.0
        for direction, turn_angle in movements:
            if np.any(direction != 0):
                duration = self.side_length / self.cruise_speed
                t = self._simulate_straight(t, direction, duration)
            elif turn_angle != 0:
                duration = abs(turn_angle / self.turn_rate)
                t = self._simulate_turn(t, turn_angle, duration)
        
        return self.imu_data, self.dvl_data, self.cam_data
    
    def save_data(self, imu_file='simulated_imu.csv', dvl_file='simulated_dvl.json', cam_file='simulated_cam.json'):
        """Save simulated data to files"""
        pd.DataFrame(self.imu_data).to_csv(imu_file, index=False)
        with open(dvl_file, 'w') as f:
            json.dump(self.dvl_data, f, indent=2)
        with open(cam_file, 'w') as f:
            json.dump(self.cam_data, f, indent=2)

# data_visualizer.py
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class DataVisualizer:
    """Visualizes IMU, DVL, and processed navigation data"""
    def __init__(self):
        self.fig = None
        self.axes = None
    
    def load_data(self, imu_data=None, dvl_data=None, cam_data=None, ekf_data=None, ground_truth=None):
        """Load data for visualization."""
        if isinstance(imu_data, list):
            self.imu_df = pd.DataFrame(imu_data)
        else:
            self.imu_df = imu_data
            
        if isinstance(dvl_data, list):
            self.dvl_df = pd.DataFrame(dvl_data)
        else:
            self.dvl_df = dvl_data
            
        if isinstance(cam_data, list):
            self.cam_df = pd.DataFrame(cam_data)
        else:
            self.cam_df = cam_data
            
        self.ekf_df = ekf_data  # Assign EKF data

        self.ground_truth = ground_truth  # Added ground truth data for comparison
            
    def plot_all(self, show_ekf=True, output_file='comparison_plot.png'):
        """Create comprehensive visualization and save to file."""
        self.fig = plt.figure(figsize=(20, 12))  # Increase figure size to accommodate additional subplot
        
        # Create subplots
        self.axes = {
            'position': plt.subplot(221),  # Position plot
            'quaternion': plt.subplot(222),  # Quaternion plot
            'path': plt.subplot(223),  # 2D path plot
            'error': plt.subplot(224)  # EKF error plot
        }
        
        # Plot all components
        self._plot_position()
        self._plot_quaternion()
        self._plot_2d_path()
        
        # Plot EKF error if EKF data is available
        if show_ekf and self.ekf_df is not None:
            self._plot_ekf_error()
        
        plt.tight_layout()
        plt.savefig(output_file)
        plt.close()
        
    def _plot_position(self):
        """Plot position over time"""
        ax = self.axes['position']
        
        # Plot noisy camera position
        if self.cam_df is not None:
            ax.plot(self.cam_df['time'], self.cam_df['x'], label='X (Camera)')
            ax.plot(self.cam_df['time'], self.cam_df['y'], label='Y (Camera)')
        
        # Plot EKF position
        if self.ekf_df is not None:
            ax.plot(self.ekf_df['time'], self.ekf_df['x'], '--', label='X (EKF)')
            ax.plot(self.ekf_df['time'], self.ekf_df['y'], '--', label='Y (EKF)')
        
        ax.set_title('Position vs Time')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.grid(True)
        ax.legend()
    
    def _plot_velocity(self):
        """Plot velocities"""
        ax = self.axes['velocity']
        if self.dvl_df is not None:
            ax.plot(self.dvl_df['time'], self.dvl_df['vx'], label='Vx (DVL)')
            ax.plot(self.dvl_df['time'], self.dvl_df['vy'], label='Vy (DVL)')
        
        if self.ekf_df is not None:
            ax.plot(self.ekf_df['time'], self.ekf_df['vx'], '--', label='Vx (EKF)')
            ax.plot(self.ekf_df['time'], self.ekf_df['vy'], '--', label='Vy (EKF)')
        
        ax.set_title('Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.grid(True)
        ax.legend()
    
    def _plot_acceleration(self):
        """Plot accelerations"""
        ax = self.axes['acceleration']
        if self.imu_df is not None:
            ax.plot(self.imu_df['Time'], 
                   self.imu_df['linear_acceleration.x'], 
                   label='Ax (IMU)')
            ax.plot(self.imu_df['Time'], 
                   self.imu_df['linear_acceleration.y'], 
                   label='Ay (IMU)')
        
        ax.set_title('Acceleration')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        ax.grid(True)
        ax.legend()
    
    def _plot_angular_velocity(self):
        """Plot angular velocities"""
        ax = self.axes['angular']
        if self.imu_df is not None:
            ax.plot(self.imu_df['Time'], 
                   self.imu_df['angular_velocity.z'], 
                   label='ωz (IMU)')
        
        ax.set_title('Angular Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.grid(True)
        ax.legend()
    
    def _plot_2d_path(self):
        """Plot 2D path with directional arrows"""
        ax = self.axes['path']
        
        # Get position data
        dt = np.diff(self.dvl_df['time']).mean()
        x = np.array(np.cumsum(self.dvl_df['vx'] * dt))  # Convert to numpy array
        y = np.array(np.cumsum(self.dvl_df['vy'] * dt))  # Convert to numpy array
        
        # Plot main path
        ax.plot(x, y, 'b-', label='Path', zorder=1)
        
        # Plot start and end points
        ax.plot(x[0], y[0], 'go', label='Start', markersize=10, zorder=3)
        ax.plot(x[-1], y[-1], 'ro', label='End', markersize=10, zorder=3)
        
        # Add directional arrows
        num_arrows = 20
        indices = np.linspace(0, len(x)-2, num_arrows, dtype=int)
        
        for i in indices:
            dx = x[i+1] - x[i]
            dy = y[i+1] - y[i]
            magnitude = np.sqrt(dx**2 + dy**2)
            if magnitude > 0:
                dx = dx / magnitude
                dy = dy / magnitude
                arrow_length = 0.2
                ax.arrow(x[i], y[i], 
                        dx * arrow_length, dy * arrow_length,
                        head_width=0.1, head_length=0.15, 
                        fc='r', ec='r', alpha=0.5,
                        zorder=2)
        
        # Add sequence numbers
        num_markers = 4
        indices = np.linspace(0, len(x)-1, num_markers+1, dtype=int)[1:-1]
        for i, idx in enumerate(indices, 1):
            ax.text(x[idx], y[idx], str(i), 
                    bbox=dict(facecolor='white', edgecolor='none', alpha=0.7),
                    ha='center', va='center', zorder=4)
        
        if self.ekf_df is not None:
            # Plot EKF path
            ax.plot(self.ekf_df['x'], self.ekf_df['y'], 
                'g--', label='EKF Path', zorder=2)
        
        ax.set_title('2D Path')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.grid(True)
        ax.axis('equal')
        ax.legend()

        # Add margins
        margin = 0.5  # meters
        ax.set_xlim(min(x) - margin, max(x) + margin)
        ax.set_ylim(min(y) - margin, max(y) + margin)

    def _plot_quaternion(self):
        """Plot quaternion components over time."""
        ax = self.axes['quaternion']
        
        # Plot ground truth quaternion (if available)
        if self.ground_truth is not None:
            ax.plot(self.ground_truth['time'], self.ground_truth['qw'], 'g-', label='qw (Ground Truth)')
            ax.plot(self.ground_truth['time'], self.ground_truth['qx'], 'b-', label='qx (Ground Truth)')
            ax.plot(self.ground_truth['time'], self.ground_truth['qy'], 'r-', label='qy (Ground Truth)')
            ax.plot(self.ground_truth['time'], self.ground_truth['qz'], 'm-', label='qz (Ground Truth)')
        
        # Plot noisy camera quaternion (if available)
        if self.cam_df is not None:
            ax.plot(self.cam_df['time'], self.cam_df['qw'], 'g--', label='qw (Noisy Camera)')
            ax.plot(self.cam_df['time'], self.cam_df['qx'], 'b--', label='qx (Noisy Camera)')
            ax.plot(self.cam_df['time'], self.cam_df['qy'], 'r--', label='qy (Noisy Camera)')
            ax.plot(self.cam_df['time'], self.cam_df['qz'], 'm--', label='qz (Noisy Camera)')
        
        # Plot EKF quaternion (if available)
        if self.ekf_df is not None:
            if 'qw' in self.ekf_df.columns:
                ax.plot(self.ekf_df['time'], self.ekf_df['qw'], 'k-', label='qw (EKF)')
            if 'qx' in self.ekf_df.columns:
                ax.plot(self.ekf_df['time'], self.ekf_df['qx'], 'c-', label='qx (EKF)')
            if 'qy' in self.ekf_df.columns:
                ax.plot(self.ekf_df['time'], self.ekf_df['qy'], 'y-', label='qy (EKF)')
            if 'qz' in self.ekf_df.columns:
                ax.plot(self.ekf_df['time'], self.ekf_df['qz'], label='qz (EKF)')
        
        ax.set_title('Quaternion Components')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Quaternion Value')
        ax.grid(True)
        ax.legend()

    def _plot_ekf_error(self):
        """Plot EKF estimation errors."""
        ax = self.axes['error']  # Use the 'error' subplot
        if self.ekf_df is not None and self.dvl_df is not None:
            # Calculate true positions from DVL
            dt = np.diff(self.dvl_df['time']).mean()
            true_x = np.cumsum(self.dvl_df['vx'] * dt)
            true_y = np.cumsum(self.dvl_df['vy'] * dt)
            
            # Interpolate true positions to EKF timestamps
            from scipy.interpolate import interp1d
            f_x = interp1d(self.dvl_df['time'], true_x, bounds_error=False)
            f_y = interp1d(self.dvl_df['time'], true_y, bounds_error=False)
            
            interp_x = f_x(self.ekf_df['time'])
            interp_y = f_y(self.ekf_df['time'])
            
            # Calculate errors
            if 'x' in self.ekf_df.columns and 'y' in self.ekf_df.columns:
                x_error = self.ekf_df['x'] - interp_x
                y_error = self.ekf_df['y'] - interp_y
                
                # Plot errors
                ax.plot(self.ekf_df['time'], x_error, label='X Error')
                ax.plot(self.ekf_df['time'], y_error, label='Y Error')
                
                # Plot uncertainty bounds if available
                if 'x_std' in self.ekf_df.columns and 'y_std' in self.ekf_df.columns:
                    ax.fill_between(self.ekf_df['time'], 
                                -self.ekf_df['x_std'], self.ekf_df['x_std'],
                                alpha=0.2, label='X Uncertainty')
                    ax.fill_between(self.ekf_df['time'], 
                                -self.ekf_df['y_std'], self.ekf_df['y_std'],
                                alpha=0.2, label='Y Uncertainty')
            
        ax.set_title('EKF Position Error')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.grid(True)
        ax.legend()