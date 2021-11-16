import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def data_reader(data):
    return np.array(pd.Series(data.iloc[:, len(data.columns) - 1]).tolist(), dtype='float')


def get_avg(file_name, max_file_num):
    vls = []
    for i in range(max_file_num):
        vls.append(data_reader(pd.read_csv(file_name.lower() + str(i + 1) + '.csv', header=None)))
    avg = []
    for i in range(len(vls[0])):
        values = []
        for j in range(max_file_num):
            values.append(vls[j][i])
        if file_name == 'iter':
            avg.append(int(sum(values) / len(values)))
        avg.append(sum(values) / len(values))
    print(avg)
    return avg


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

    avg = get_avg(file_name='Error', max_file_num=4)
    plotter(avg, 'Error')


if __name__ == '__main__':
    main()
