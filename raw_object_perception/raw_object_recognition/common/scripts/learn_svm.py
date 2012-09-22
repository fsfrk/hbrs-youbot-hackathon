#!/usr/bin/python

import numpy as np
import mlpy
import matplotlib.pyplot as plt
import sys

from dataset import Dataset

if __name__ == '__main__':
    dataset = Dataset('../data/', 'standard')
    x, y = dataset.load()
    data = np.loadtxt(sys.argv[1], delimiter=' ')
    x, y = data[:, :5], data[:, 5].astype(np.int)
    print 'Dataset dimensions:', x.shape

    # Learn PCA and visualize
    pca = mlpy.PCA(whiten=True)
    pca.learn(x)
    z = pca.transform(x, k=2)
    plt.set_cmap(plt.cm.Paired)
    fig1 = plt.figure(1)
    title = plt.title("PCA on dataset")
    plot = plt.scatter(z[:, 0], z[:, 1], c=y)

    # Learn SVM and visualize
    svm = mlpy.LibSvm(kernel=mlpy.KernelGaussian(sigma=1.0))
    svm.learn(z, y)
    xmin, xmax = z[:, 0].min() - 0.1, z[:, 0].max() + 0.1
    ymin, ymax = z[:, 1].min() - 0.1, z[:, 1].max() + 0.1
    xx, yy = np.meshgrid(np.arange(xmin, xmax, 0.01), np.arange(ymin, ymax, 0.01))
    zgrid = np.c_[xx.ravel(), yy.ravel()]
    yp = svm.pred(zgrid)
    plt.set_cmap(plt.cm.Paired)
    fig2 = plt.figure(2)
    title = plt.title("SVM (gaussian kernel) on principal components")
    plot1 = plt.pcolormesh(xx, yy, yp.reshape(xx.shape))
    plot2 = plt.scatter(z[:, 0], z[:, 1], c=y)
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
