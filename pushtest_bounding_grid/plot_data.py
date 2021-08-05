import scipy.io as scio
import matplotlib.pyplot as plt
import numpy as np    

if __name__ == '__main__':
    path_data = 'results/bounding6710.mat'
    data_set = scio.loadmat(path_data)
    results = data_set['results']       
    print(results)