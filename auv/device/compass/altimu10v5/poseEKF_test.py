import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
from datetime import datetime
import numpy as np
import pandas as pd
from auv.utils.poseEKF_Sim import SensorSimulator, DataVisualizer
from positionEKF import PoseEKF
import time

class EKFTester:
    def __init__(self):
        # Create simulator and generate data
        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=100,  # 100 Hz
            dvl_rate=10,    # 10 Hz
            cam_rate=30     # 30 Hz
        )
        self.imu_data, self.dvl_data, self.camera_data = self.simulator.generate_square_path()
        
        # Convert to DataFrames and sort by time
        self.imu_df = pd.DataFrame(self.imu_data).sort_values('Time')
        self.dvl_df = pd.DataFrame(self.dvl_data).sort_values('time')
        self.camera_df = pd.DataFrame(self.camera_data).sort_values('time')
        
        # Generate ground truth data
        self.ground_truth = self._generate_ground_truth()  # <-- Added this line
        
        # Create mock DVL object with expected interface
        mock_dvl = type('DVL', (), {
            'vel_rot': [0, 0, 0],  # Initial velocities
            'sub_type': 'onyx',    # Or 'graey' depending on your setup
            'dt': 1.0/10           # DVL time step (10 Hz)
        })()
        
        # Create simulated_data dictionary
        simulated_data = {
            'imu_data': self.imu_data,
            'dvl_data': self.dvl_data,
            'cam_data': self.camera_data
        }
        
        # Initialize EKF with mock DVL and simulated data
        self.ekf = PoseEKF(mock_dvl, use_simulated_data=True, simulated_data=simulated_data)
        self.ekf.dt = 1.0/100      # IMU time step (100 Hz)
        
        # Storage for EKF results
        self.ekf_results = []
        
    def simulate_live_processing(self):
        """Process data sequentially to mimic live operation"""
        print("Starting simulated live processing...")
        
        # Get start times
        start_time = min(self.imu_df['Time'].min(), self.dvl_df['time'].min(), self.camera_df['time'].min())
        end_time = max(self.imu_df['Time'].max(), self.dvl_df['time'].max(), self.camera_df['time'].max())
        
        # Process data in time order
        current_time = start_time
        while current_time <= end_time:
            # Get IMU and camera data at current time
            imu_msg = self.imu_df[self.imu_df['Time'] == current_time]
            camera_msg = self.camera_df[self.camera_df['time'] == current_time]
            
            if not imu_msg.empty:
                # Create IMU message in correct format
                imu_data = {
                    'linear_acceleration': {
                        'x': imu_msg['linear_acceleration.x'].iloc[0],
                        'y': imu_msg['linear_acceleration.y'].iloc[0],
                        'z': imu_msg['linear_acceleration.z'].iloc[0],
                    },
                    'orientation': {
                        'w': imu_msg['orientation.w'].iloc[0],
                        'x': imu_msg['orientation.x'].iloc[0],
                        'y': imu_msg['orientation.y'].iloc[0],
                        'z': imu_msg['orientation.z'].iloc[0]
                    }
                }
                # Update EKF with IMU data
                self.ekf.imu_callback(imu_data)

            if not camera_msg.empty:
                # Create camera message in correct format
                camera_data = {
                    'position': {
                        'x': camera_msg['x'].iloc[0],  # Access 'x' column
                        'y': camera_msg['y'].iloc[0],  # Access 'y' column
                        'z': camera_msg['z'].iloc[0]   # Access 'z' column
                    },
                    'orientation': {
                        'w': camera_msg['qw'].iloc[0],  # Access 'qw' column
                        'x': camera_msg['qx'].iloc[0],  # Access 'qx' column
                        'y': camera_msg['qy'].iloc[0],  # Access 'qy' column
                        'z': camera_msg['qz'].iloc[0]   # Access 'qz' column
                    }
                }
                # Update EKF with camera data
                self.ekf.camera_callback(camera_data)
            
            # Get DVL data at current time
            dvl_msg = self.dvl_df[self.dvl_df['time'] == current_time]
            if not dvl_msg.empty:
                # Create DVL data in correct format (matching dvl.py format)
                dvl_data = {
                    'vel_rot': [
                        dvl_msg['vx'].iloc[0],
                        dvl_msg['vy'].iloc[0],
                        dvl_msg['vz'].iloc[0]
                    ]
                }
                # Update EKF with DVL data
                self.ekf.DVL.vel_rot = dvl_data['vel_rot']
                self.ekf.update_filter()
            
            # Store EKF results
            self.ekf_results.append({
                'time': current_time,
                'x': self.ekf.ekf.x[0],
                'y': self.ekf.ekf.x[1],
                'z': self.ekf.ekf.x[2],
                'vx': self.ekf.ekf.x[3],
                'vy': self.ekf.ekf.x[4],
                'vz': self.ekf.ekf.x[5],
                'qw': self.ekf.ekf.x[6],  # Quaternion component
                'qx': self.ekf.ekf.x[7],  # Quaternion component
                'qy': self.ekf.ekf.x[8],  # Quaternion component
                'qz': self.ekf.ekf.x[9]   # Quaternion component
            })
            
            # Increment time
            current_time += self.ekf.dt
        
        # Convert results to DataFrame
        self.ekf_df = pd.DataFrame(self.ekf_results)
        print("Simulated live processing completed.")

    def plot_comparison(self, output_dir):
        """Plot comparison of raw and EKF-processed data and save to file"""
        visualizer = DataVisualizer()
        
        # Plot raw simulated data
        visualizer.load_data(self.imu_df, self.dvl_df, self.camera_df, ground_truth=self.ground_truth)  # <-- Added ground_truth
        raw_output_file = os.path.join(output_dir, 'raw_simulated_data.png')
        visualizer.plot_all(show_ekf=False, output_file=raw_output_file)
        
        # Plot EKF-processed data
        visualizer.load_data(self.imu_df, self.dvl_df, self.camera_df, self.ekf_df, ground_truth=self.ground_truth)  # <-- Added ground_truth
        visualizer.print_errors()
        ekf_output_file = os.path.join(output_dir, 'ekf_processed_data.png')
        visualizer.plot_all(show_ekf=True, output_file=ekf_output_file)
        
        print(f"Raw simulated data plot saved to {raw_output_file}")
        print(f"EKF processed data plot saved to {ekf_output_file}")
        
    def _generate_ground_truth(self):
        """Generate ground truth data for position and orientation."""
        ground_truth = []
        t = 0.0
        orientation = 0.0
        position = np.zeros(3)
        
        # Simulate the square path to generate ground truth
        movements = [
            np.array([0.0, 1.0, 0.0]),  # Forward
            np.array([0.0, 0.0, 0.0]),  # Turn left
            np.array([-1.0, 0.0, 0.0]), # Left
            np.array([0.0, 0.0, 0.0]),  # Turn left
            np.array([0.0, -1.0, 0.0]),# Back
            np.array([0.0, 0.0, 0.0]),  # Turn left
            np.array([1.0, 0.0, 0.0]),  # Right
        ]
        
        # Define turn angles for each movement
        turn_angles = [0.0, np.pi/2, 0.0, np.pi/2, 0.0, np.pi/2, 0.0]
        
        for direction, turn_angle in zip(movements, turn_angles):  # <-- Fixed here
            if np.any(direction != 0):
                duration = self.simulator.side_length / self.simulator.cruise_speed
                vel = direction * self.simulator.cruise_speed
                while t < duration:
                    position += vel * self.simulator.imu_dt
                    ground_truth.append({
                        'time': t,
                        'x': position[0],
                        'y': position[1],
                        'z': position[2],
                        'qw': np.cos(orientation / 2),
                        'qx': 0.0,
                        'qy': 0.0,
                        'qz': np.sin(orientation / 2)
                    })
                    t += self.simulator.imu_dt
            elif turn_angle != 0:
                duration = abs(turn_angle / self.simulator.turn_rate)
                omega = turn_angle / duration
                while t < duration:
                    orientation += omega * self.simulator.imu_dt
                    ground_truth.append({
                        'time': t,
                        'x': position[0],
                        'y': position[1],
                        'z': position[2],
                        'qw': np.cos(orientation / 2),
                        'qx': 0.0,
                        'qy': 0.0,
                        'qz': np.sin(orientation / 2)
                    })
                    t += self.simulator.imu_dt
        
        return pd.DataFrame(ground_truth)
if __name__ == "__main__":
    # Create a directory for this run
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join("C:/Users/joshy/OneDrive/Documents/24-25/191/robosub_2024/auv/device/compass/altimu10v5/EKFComp", timestamp)
    os.makedirs(output_dir, exist_ok=True)
    
    tester = EKFTester()
    tester.simulate_live_processing()
    
    # Plot comparison of raw and EKF-processed data and save to file
    tester.plot_comparison(output_dir=output_dir)