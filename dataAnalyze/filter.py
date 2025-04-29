import numpy as np
import queue
import time
import threading
import matplotlib.pyplot as plt

class FilteredData:
    def __init__(self, size: int):
        self.size = size
        self.data = []

    def add(self, msg: float):
        if len(self.data) >= self.size:
            self.data.pop(0)
        self.data.append(msg)

    def get(self, index: int) -> float:
        return self.data[index]
    
    def remove(self,index:int):
        self.data.pop(index)
    
    def to_numpy(self) -> np.ndarray:
        return np.array(self.data)

    def is_empty(self) -> bool:
        return not len(self.data)>0
def publish_thread(df: np.ndarray):
    """
    A publisher thread to simulate publishing messages at 10 Hz.
    """
    for i in range(len(df[:, 1])):
        time.sleep(0.1)  # 10 Hz

def callback():
    pass  # Placeholder for future callback functionality

def lowpass_filter(alpha: float, filtered: FilteredData, incoming: float) -> float:
    """
    Apply lowpass filter: smooths signal by allowing low frequencies to pass.
    """
    return alpha * incoming + (1 - alpha) * filtered.get(-1)

def highpass_filter(alpha: float, filtered: FilteredData, incoming: float, previous: float) -> float:
    """
    Apply highpass filter: removes slow (low-frequency) variations.
    """
    return alpha * (filtered.get(-1) + incoming - previous)

def rolling_mean(df: np.ndarray, window_size: int) -> np.ndarray:
    """
    Apply a rolling mean (moving average) filter to the signal.
    """
    return np.convolve(df[:, 1], np.ones(window_size)/window_size, mode='valid')

def kalman_filter(filtered: FilteredData, incoming: float, P: float, Q: float, R: float) -> tuple[float, float]:
    """
    Apply a simple 1D Kalman filter.
    - P: estimate uncertainty
    - Q: process noise covariance
    - R: measurement noise covariance
    """
    x_est_last = filtered.get(-1)

    # Predict phase
    x_pred = x_est_last
    P = P + Q

    # Update phase
    K = P / (P + R)
    x_est = x_pred + K * (incoming - x_pred)
    P = (1 - K) * P

    return x_est, P

def apply_filter(df: np.ndarray, mode: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Filter data using specified mode: 'lowpass', 'highpass', or 'kalman'.
    """
    filtered = []
    fdata = FilteredData(size=10)
    previous = df[0]  # Needed for highpass filter
    P = 1.0  # Initial estimate uncertainty
    Q = 1e-5  # Process noise
    R = 0.01  # Measurement noise

    for msg in df:
        if fdata.is_empty():
            fdata.add(0)
            continue
        if mode == "lowpass":
            filtered_value = lowpass_filter(alpha=0.1, filtered=fdata, incoming=msg)
            filtered.append(filtered_value)
            fdata.add(filtered_value)
        elif mode == "highpass":
            filtered_value = highpass_filter(alpha=0.5, filtered=fdata, incoming=msg, previous=previous)
            filtered.append(filtered_value)
            fdata.add(filtered_value)
            previous = msg
        elif mode == "kalman":
            filtered_value, P = kalman_filter(filtered=fdata, incoming=msg, P=P, Q=Q, R=R)
            filtered.append(filtered_value)
            fdata.add(filtered_value)
        elif mode == "highpass + kalman":
            kalman_value, P = kalman_filter(filtered=fdata, incoming=msg, P=P, Q=Q, R=R)
            filtered_value = highpass_filter(alpha=0.8,filtered=fdata, incoming=kalman_value,previous=previous)
            filtered.append(filtered_value)
            fdata.add(filtered_value)
            previous = kalman_value
        elif mode == "lowpass + kalman":
            kalman_value, P = kalman_filter(filtered=fdata, incoming=msg, P=P, Q=Q, R=R)
            filtered_value = lowpass_filter(alpha=0.1, filtered=fdata, incoming=msg)
            filtered.append(filtered_value)
            fdata.add(filtered_value)

        else:
            raise ValueError(f"Unsupported filter mode: {mode}")

    return np.array(filtered), df


def plot(data: list[np.ndarray], titles: list[str]):
    """
    Plot multiple datasets, each in its own window.
    """
    for arr, title in zip(data, titles):
        plt.figure(figsize=(8, 6))  # Create a new figure for each dataset
        plt.plot(arr, label=title)  # Plot the data
        plt.title(title)            # Set the title for each dataset
        plt.xlabel('Sample Index')  # Label for X-axis
        plt.ylabel('Value')         # Label for Y-axis
        plt.grid(True)              # Optional: add grid
        plt.legend()                # Display legend
    
    plt.show()                  # Show the plot in its own window

if __name__ == "__main__":
    file_path = r"C:\Users\chase\Downloads\2025-04-27_17-28-16_imu_data.csv"
    df = np.loadtxt(file_path, delimiter=",", skiprows=1)
    print(df.shape)
    filtered, raw = apply_filter(df=df[:,3], mode="lowpass + kalman")
    plot(data=[filtered, raw], titles=["Filtered", "Raw"])
