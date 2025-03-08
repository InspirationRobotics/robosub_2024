import numpy as np
import pandas as pd
import json
from datetime import datetime
import os

class SensorSimulator:
    """Generates simulated IMU and DVL data for a square path"""
    def __init__(self, side_length=5.0, imu_rate=100, dvl_rate=10):
        self.side_length = side_length
        self.imu_dt = 1.0 / imu_rate
        self.dvl_dt = 1.0 / dvl_rate
        self.cruise_speed = 0.5  # m/s
        self.turn_rate = np.pi/4  # rad/s (45 deg/s)
        
        # Noise parameters
        self.imu_accel_noise_std = 0.01  # m/s^2
        self.imu_gyro_noise_std = 0.001  # rad/s
        self.dvl_vel_noise_std = 0.002   # m/s
        
        # Initialize storage and state
        self.reset()
        
    def reset(self):
        """Reset simulator state"""
        self.imu_data = []
        self.dvl_data = []
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = 0.0
        
    def _simulate_straight(self, start_time, direction, duration):
        """Simulate straight line motion"""
        t = start_time
        vel = direction * self.cruise_speed
        
        while t < start_time + duration:
            # IMU data (100 Hz)
            if int(t / self.imu_dt) > int((t - self.imu_dt) / self.imu_dt):
                accel_noise = np.random.normal(0, self.imu_accel_noise_std, 3)
                gyro_noise = np.random.normal(0, self.imu_gyro_noise_std, 3)
                
                self.imu_data.append({
                    'Time': t,
                    'header.seq': int(t * 1000),
                    'header.stamp.secs': int(t),
                    'header.stamp.nsecs': int((t % 1) * 1e9),
                    'header.frame_id': "imu_link",
                    'orientation.x': 0.0,
                    'orientation.y': 0.0,
                    'orientation.z': np.sin(self.orientation/2),
                    'orientation.w': np.cos(self.orientation/2),
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
                self.dvl_data.append({
                    'time': t,
                    'vx': vel[0] + vel_noise[0],
                    'vy': vel[1] + vel_noise[1],
                    'vz': vel[2] + vel_noise[2],
                    'error': self.dvl_vel_noise_std,
                    'valid': True
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
                
                self.imu_data.append({
                    'Time': t,
                    'header.seq': int(t * 1000),
                    'header.stamp.secs': int(t),
                    'header.stamp.nsecs': int((t % 1) * 1e9),
                    'header.frame_id': "imu_link",
                    'orientation.x': 0.0,
                    'orientation.y': 0.0,
                    'orientation.z': np.sin(self.orientation/2),
                    'orientation.w': np.cos(self.orientation/2),
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
                self.dvl_data.append({
                    'time': t,
                    'vx': vel_noise[0],
                    'vy': vel_noise[1],
                    'vz': vel_noise[2],
                    'error': self.dvl_vel_noise_std,
                    'valid': True
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
        
        return self.imu_data, self.dvl_data
    
    def save_data(self, imu_file='simulated_imu.csv', dvl_file='simulated_dvl.json'):
        """Save simulated data to files"""
        pd.DataFrame(self.imu_data).to_csv(imu_file, index=False)
        with open(dvl_file, 'w') as f:
            json.dump(self.dvl_data, f, indent=2)


# data_visualizer.py
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class DataVisualizer:
    """Visualizes IMU, DVL, and processed navigation data"""
    def __init__(self):
        self.fig = None
        self.axes = None
    
    def load_data(self, imu_data=None, dvl_data=None, ekf_data=None):
        """Load data for visualization"""
        if isinstance(imu_data, list):
            self.imu_df = pd.DataFrame(imu_data)
        else:
            self.imu_df = imu_data
            
        if isinstance(dvl_data, list):
            self.dvl_df = pd.DataFrame(dvl_data)
        else:
            self.dvl_df = dvl_data
            
        self.ekf_df = ekf_data
    
    def plot_all(self, show_ekf=True, output_file='comparison_plot.png'):
        """Create comprehensive visualization and save to file"""
        self.fig = plt.figure(figsize=(20, 10))
        
        # Create subplots
        self.axes = {
            'position': plt.subplot(231),
            'velocity': plt.subplot(232),
            'acceleration': plt.subplot(233),
            'angular': plt.subplot(234),
            'path': plt.subplot(235),
            'error': plt.subplot(236)
        }
        
        self._plot_position()
        self._plot_velocity()
        self._plot_acceleration()
        self._plot_angular_velocity()
        self._plot_2d_path()
        if show_ekf and self.ekf_df is not None:
            self._plot_ekf_error()
        
        plt.tight_layout()
        plt.savefig(output_file)
        plt.close()
    
    def _plot_position(self):
        """Plot position over time"""
        ax = self.axes['position']
        if self.dvl_df is not None:
            dt = np.diff(self.dvl_df['time']).mean()
            x = np.cumsum(self.dvl_df['vx'] * dt)
            y = np.cumsum(self.dvl_df['vy'] * dt)
            ax.plot(self.dvl_df['time'], x, label='X (DVL)')
            ax.plot(self.dvl_df['time'], y, label='Y (DVL)')
        
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

    def _plot_ekf_error(self):
        """Plot EKF estimation errors"""
        ax = self.axes['error']
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
            x_error = self.ekf_df['x'] - interp_x
            y_error = self.ekf_df['y'] - interp_y
            
            # Plot errors
            ax.plot(self.ekf_df['time'], x_error, label='X Error')
            ax.plot(self.ekf_df['time'], y_error, label='Y Error')
            
            # Plot uncertainty bounds if available
            if 'x_std' in self.ekf_df.columns:
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

# Example usage:
if __name__ == "__main__":
    # Generate simulated data
    simulator = SensorSimulator(
        side_length=5.0,
        imu_rate=100,  # 100 Hz
        dvl_rate=10    # 10 Hz
    )
    imu_data, dvl_data = simulator.generate_square_path()
    
    # Save data
    simulator.save_data('simulated_imu.csv', 'simulated_dvl.json')
    
    # Create mock EKF data for testing visualization
    # In reality, this would come from your EKF processing
    dvl_df = pd.DataFrame(dvl_data)
    dt = np.diff(dvl_df['time']).mean()
    ekf_data = {
        'time': dvl_df['time'],
        'x': np.cumsum(dvl_df['vx'] * dt) + np.random.normal(0, 0.1, len(dvl_df)),
        'y': np.cumsum(dvl_df['vy'] * dt) + np.random.normal(0, 0.1, len(dvl_df)),
        'vx': dvl_df['vx'] + np.random.normal(0, 0.05, len(dvl_df)),
        'vy': dvl_df['vy'] + np.random.normal(0, 0.05, len(dvl_df)),
        'x_std': np.ones(len(dvl_df)) * 0.2,
        'y_std': np.ones(len(dvl_df)) * 0.2
    }
    ekf_df = pd.DataFrame(ekf_data)
    
    # Create a directory for this run
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('/Users/poosh/Desktop/ECE191/test_robosub_2024/auv/utils/EKFComp', timestamp)
    os.makedirs(output_dir, exist_ok=True)
    
    # Visualize raw simulated data and save to file
    visualizer = DataVisualizer()
    visualizer.load_data(imu_data, dvl_data, None)
    raw_output_file = os.path.join(output_dir, 'raw_simulated_data.png')
    visualizer.plot_all(show_ekf=False, output_file=raw_output_file)
    
    # Visualize EKF-processed data and save to file
    visualizer.load_data(imu_data, dvl_data, ekf_df)
    ekf_output_file = os.path.join(output_dir, 'ekf_processed_data.png')
    visualizer.plot_all(show_ekf=True, output_file=ekf_output_file)
    
    print(f"Raw simulated data plot saved to {raw_output_file}")
    print(f"EKF processed data plot saved to {ekf_output_file}")