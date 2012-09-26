#!/usr/bin/python

import numpy as np
import mlpy
import matplotlib.pyplot as plt

import sys
from dataset import Dataset

if __name__ == '__main__':
    dataset = Dataset('../data/', 'standard')
    #data, ids = dataset.load(['F20_20_G', 'F20_20_B', 'R20'])
    X, Y, labels = dataset.load('all')
    print 'Dataset dimensions:', X.shape
    # Learn PCA and visualize
    pca = mlpy.PCA(whiten=True)
    pca.learn(X)
    z = pca.transform(X, k=2)
    plot = plt.scatter(z[:, 0], z[:, 1], c=Y)
    plt.set_cmap(plt.cm.Paired)
    fig1 = plt.figure(1)
    title = plt.title("PCA on dataset")
    # Learn SVM and visualize
    #svm = mlpy.LibSvm(kernel=mlpy.KernelGaussian(sigma=1.0))
    svm = mlpy.LibSvm(kernel_type='linear')
    svm.learn(z, Y.flatten())
    xmin, xmax = z[:, 0].min() - 0.1, z[:, 0].max() + 0.1
    ymin, ymax = z[:, 1].min() - 0.1, z[:, 1].max() + 0.1
    xx, yy = np.meshgrid(np.arange(xmin, xmax, 0.01), np.arange(ymin, ymax, 0.01))
    zgrid = np.c_[xx.ravel(), yy.ravel()]
    yp = svm.pred(zgrid)
    fig2 = plt.figure(2)
    plot1 = plt.pcolormesh(xx, yy, yp.reshape(xx.shape))
    plot2 = plt.scatter(z[:, 0], z[:, 1], c=Y)
    plt.set_cmap(plt.cm.Paired)
    title = plt.title("SVM (gaussian kernel) on principal components")
    labx = plt.xlabel("First component")
    laby = plt.ylabel("Second component")
    limx = plt.xlim(xmin, xmax)
    limy = plt.ylim(ymin, ymax)
    plt.show()

    # Store PCA and SVM to disk if okay
    #if confirm(prompt="Save PCA and SVM?", resp=True):
        #with open('pca_svm', 'wb') as output:
            #pickle.dump(pca, output, pickle.HIGHEST_PROTOCOL)
            #pickle.dump(svm, output, pickle.HIGHEST_PROTOCOL)
