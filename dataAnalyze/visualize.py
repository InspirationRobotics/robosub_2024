import matplotlib.pyplot as plt
import pandas as pd

def plot_data(data):
    """
    Plot the given data using matplotlib.

    Args:
        x (list): X-axis data (e.g., time).
        
                        y (list): Y-axis data (e.g., sensor values).
        title (str): The title of the plot.
        xlabel (str): The label for the x-axis.
        ylabel (str): The label for the y-axis.
    """
    x = data.iloc[:,0]
    
    xlabel = "time"
    
    for column in data.columns[1:]:
        title = f'{column} vs Time'
        ylabel = column
        plt.figure(figsize=(10, 6))
        plt.scatter(x, data[column], label=ylabel, color='blue', marker='o')
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    filename = r"\\dohome2.pusd.dom\Home2$\Student2\1914840\Chrome Downloads\2025-04-27_17-28-16_imu_data.csv"
    data = pd.read_csv(filename)

    time = data.iloc[:, 0]  # First column is assumed to be timestamp

    plot_data(data)
