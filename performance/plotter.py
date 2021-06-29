import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def data_reader(data):
    return np.array(pd.Series(data.iloc[1:, len(data.columns) - 1]).tolist(), dtype='float')


def plotter(errors, time, iteration):
    plt.plot(iteration, errors)
    plt.xlabel('Iterations')
    plt.ylabel('Error')
    plt.show()
    plt.plot(iteration, time)
    plt.xlabel('Iterations')
    plt.ylabel('Time')
    plt.show()


def main():
    error_values = data_reader(pd.read_csv('error.csv', header=None))
    time_values = data_reader(pd.read_csv('time.csv', header=None))
    plotter(error_values, time_values, iteration=[x + 1 for x in range(len(error_values))])


if __name__ == '__main__':
    main()
