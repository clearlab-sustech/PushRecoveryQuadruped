import scipy.io as scio
import matplotlib.pyplot as plt
import numpy as np    

if __name__ == '__main__':
    path_data = 'results/bak/trotting6715_BASE_CONTACT.mat'
    data_set = scio.loadmat(path_data)
    results = data_set['results']
    print(results)