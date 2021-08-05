import scipy.io as scio
import matplotlib.pyplot as plt
import numpy as np   
import pandas as pd 

if __name__ == '__main__':
    path_data = './mc-build/expdata.txt'
    data_set = np.array(pd.read_csv(path_data))
    plt.figure()
    for i in range(80):
        plt.plot(data_set[2*i,2::4])
        # plt.figure()
        plt.plot(data_set[2*i+1,2::4])
    plt.show()

    """ path_data = './mc-build/simVSreal.txt'
    data_set = np.array(pd.read_csv(path_data))

    plt.figure()
    plt.plot(data_set[:, 1])
    plt.plot(data_set[:, 7])
    plt.title("velocity: estiamted v.s. actual")
    plt.figure()
    plt.plot(data_set[:, 4])
    plt.plot(data_set[:, 10])
    plt.title("acceleration: estiamted v.s. actual")
    plt.show() """
    