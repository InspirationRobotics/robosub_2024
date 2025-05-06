import pandas as pd
from visualize import plot_data
from filter import apply_filter
def positionCallback(prev, curr, dt):
    return ((prev + curr) / 2) * dt    



class stimulator:
    def __init__(self, data:pd.DataFrame):
        pass
        self.data = data
        self.columns = self.data.columns
        self.localization:list
        self.imu_last_time = None



    def imuCallback(self, df:pd.DataFrame):
        current_time = df[0]

        if self.imu_last_time is None:
            self.imu_last_time = current_time
            self.acc_x = df["AccX"]
            self.acc_y = df["AccY"]
            self.acc_z = df["AccZ"]
            return

        dt = current_time - self.imu_last_time

        # X-axis
        prev_vel = self.imu_vel_x
        self.imu_vel_x += (self.acc_x + df["AccX"]) * dt / 2
        self.imu_pos_x += positionCallback(prev_vel, self.imu_vel_x, dt)

        # Y-axis
        prev_vel = self.imu_vel_y
        self.imu_vel_y += (self.acc_y + df["AccY"]) * dt / 2
        self.imu_pos_y += positionCallback(prev_vel, self.imu_vel_y, dt)

        # Z-axis
        prev_vel = self.imu_vel_z
        self.imu_vel_z += (self.acc_z + df["AccZ"]) * dt / 2
        self.imu_pos_z += positionCallback(prev_vel, self.imu_vel_z, dt)

        # Update last acceleration and time
        self.acc_x = df["AccX"]
        self.acc_y = df["AccY"]
        self.acc_z = df["AccZ"]
        
        self.imu_last_time = current_time
        

    def run(self):
        for i in range(self.data.shape[0]):
            self.imuCallback(self.data[i,:]) # pass in a row

        plot_data(self.localization)
        
        
    