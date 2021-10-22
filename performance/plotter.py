import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def data_reader(data):
    return np.array(pd.Series(data.iloc[:, len(data.columns) - 1]).tolist(), dtype='float')


def plotter(values, y_label):
    plt.plot(np.arange(0, 360 + 1, 30), values, color="b", marker="o", linestyle="-", linewidth=1, markersize=5)
    plt.xlabel('Angles')
    plt.ylabel(y_label)
    plt.show()


def main():
    sources = ['Error', 'Time']
    for src in sources:
        values = data_reader(pd.read_csv(src.lower() + '.csv', header=None))
        plotter(values, src)


if __name__ == '__main__':
    main()
