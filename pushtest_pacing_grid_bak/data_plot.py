import scipy.io as scio
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if __name__ == "__main__":
    path_data = "mc-build/simVSreal.txt"
    data_set = np.array(pd.read_csv(path_data))
    plt.subplot(3,1,1)
    plt.plot(data_set[:,0:12:6])
    plt.subplot(3,1,2)
    plt.plot(data_set[:,1:12:6])
    plt.subplot(3,1,3)
    plt.plot(data_set[:,2:12:6])
    plt.show()