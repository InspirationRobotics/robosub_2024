import matplotlib.pyplot as plt
import pandas as pd

def plot_data(x, y, title='Data Plot', xlabel='Time (s)', ylabel='Value'):
    """
    Plot the given data using matplotlib.

    Args:
        x (list): X-axis data (e.g., time).
        y (list): Y-axis data (e.g., sensor values).
        title (str): The title of the plot.
        xlabel (str): The label for the x-axis.
        ylabel (str): The label for the y-axis.
    """
    plt.figure(figsize=(10, 6))
    plt.scatter(x, y, label=title, color='blue', marker='o')
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    filename = "/home/chaser/fog_data.csv"
    data = pd.read_csv(filename)

    time = data.iloc[:, 0]  # First column is assumed to be timestamp

    for column in data.columns[1:]:  # Skip the first column (time)
        plot_data(time, data[column], title=f'{column} vs Time', ylabel=column)
