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
        self.localization = {}
        self.imu_last_time = None
                
        self.imu_last_time = None

        self.acc_x = None
        self.acc_y = None
        self.acc_z = None

        self.imu_vel_x: float = 0.0
        self.imu_vel_y: float = 0.0
        self.imu_vel_z: float = 0.0

        self.imu_pos_x: float = 0.0
        self.imu_pos_y: float = 0.0
        self.imu_pos_z: float = 0.0


    def imuCallback(self, df:pd.DataFrame):
        current_time = df[0]

        if self.imu_last_time is None:
            self.imu_last_time = current_time
            self.acc_x = df["AccX"]
            self.acc_y = df["AccY"]
            self.acc_z = df["AccZ"]

            self.localization["time"] = []
            self.localization["x"] = []
            self.localization["y"] = []
            self.localization["z"] = []
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

        self.localization["time"].apppend(self.imu_last_time)
        self.localization["x"].apppend(self.imu_pos_x)
        self.localization["y"].apppend(self.imu_pos_y)
        self.localization["z"].apppend(self.imu_pos_z)
        

    def run(self):
        for i in range(self.data.shape[0]):
            self.imuCallback(self.data.iloc[i])  # correct Pandas row access

        loc = pd.DataFrame(data=self.localization)
        plot_data(loc)



if __name__ == "__main__":
    path = r"\\dohome2.pusd.dom\Home2$\Student2\1914840\Chrome Downloads\2025-04-27_17-28-16_imu_data.csv"
    data = pd.read_csv(path)

    ss = stimulator(data=data)

    ss.run()
        
        
    