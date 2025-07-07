from scipy.signal import butter, lfilter
import numpy as np
import matplotlib.pyplot as plt

def butter_highpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    norm_cutoff = cutoff / nyq
    b, a = butter(order, norm_cutoff, btype='high', analog=False)
    return b, a

def highpass_filter_cutoff(data, cutoff=0.1, fs=10.0, order=2):
    b, a = butter_highpass(cutoff, fs, order)
    data = np.array(data)
    return lfilter(b=b, a=a, x=data)


file_path = r"\\dohome2.pusd.dom\Home2$\Student2\1914840\Chrome Downloads\2025-04-27_17-28-16_imu_data.csv"
df = np.loadtxt(file_path, delimiter=",", skiprows=1)

acc_z = df[:,3]

res = []
for i in range(1,acc_z.size-1):
    res.append(highpass_filter_cutoff(data=acc_z[i-1:i]))
    i +=2
    pass

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

plot([res,acc_z],["z","raw"])