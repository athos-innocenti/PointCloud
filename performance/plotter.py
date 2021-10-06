import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def data_reader(data):
    return np.array(pd.Series(data.iloc[:, len(data.columns) - 1]).tolist(), dtype='float')


def plotter(errors, time, angles):
    plt.plot(angles, errors, color="b", marker="o", linestyle="-", linewidth=1, markersize=5)
    plt.xlabel('Angles')
    plt.ylabel('Error')
    plt.show()
    plt.plot(angles, time)
    plt.xlabel('Angles')
    plt.ylabel('Time')
    plt.show()


def main():
    error_values = data_reader(pd.read_csv('error.csv', header=None))
    time_values = data_reader(pd.read_csv('time.csv', header=None))
    plotter(error_values, time_values, angles=np.arange(0, 360 + 1, 30))


if __name__ == '__main__':
    main()
