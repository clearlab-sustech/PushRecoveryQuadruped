import scipy.io as scio
import matplotlib.pyplot as plt


if __name__ == '__main__':
    # path_data = '/home/nimapng/Desktop/pushtest/results.mat'
    path_data = '/home/nimapng/Desktop/pushtest/results/results_bounding000.mat'
    data_set = scio.loadmat(path_data)
    data = data_set['results']
    print(data)
    # plt.plot(range(int(data.size / 6)), data[3::6])
    # plt.show()