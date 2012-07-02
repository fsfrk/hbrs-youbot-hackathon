#!/usr/bin/python

import pickle
import numpy as np
import mlpy
import matplotlib.pyplot as plt
import sys

def confirm(prompt=None, resp=False):
    """prompts for yes or no response from the user. Returns True for yes and
    False for no.

    'resp' should be set to the default value assumed by the caller when
    user simply types ENTER.

    >>> confirm(prompt='Create Directory?', resp=True)
    Create Directory? [y]|n: 
    True
    >>> confirm(prompt='Create Directory?', resp=False)
    Create Directory? [n]|y: 
    False
    >>> confirm(prompt='Create Directory?', resp=False)
    Create Directory? [n]|y: y
    True

    """

    if prompt is None:
        prompt = 'Confirm'

    if resp:
        prompt = '%s [%s]|%s: ' % (prompt, 'y', 'n')
    else:
        prompt = '%s [%s]|%s: ' % (prompt, 'n', 'y')

    while True:
        ans = raw_input(prompt)
        if not ans:
            return resp
        if ans not in ['y', 'Y', 'n', 'N']:
            print 'please enter y or n.'
            continue
        if ans == 'y' or ans == 'Y':
            return True
        if ans == 'n' or ans == 'N':
            return False

if __name__ == '__main__':
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
    if confirm(prompt="Save PCA and SVM?", resp=True):
        with open('pca_svm', 'wb') as output:
            pickle.dump(pca, output, pickle.HIGHEST_PROTOCOL)
            pickle.dump(svm, output, pickle.HIGHEST_PROTOCOL)
