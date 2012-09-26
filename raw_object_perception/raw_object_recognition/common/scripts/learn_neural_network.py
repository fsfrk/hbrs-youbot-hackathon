#!/usr/bin/python

import numpy as np
from pybrain.datasets import ClassificationDataSet
from pybrain.utilities import percentError
from pybrain.tools.shortcuts import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules import SoftmaxLayer

from dataset import Dataset, JointDataset

if __name__ == '__main__':
    d1 = Dataset('../data/', 'standard')
    d2 = Dataset('../data/', 'standing')
    dataset = JointDataset([d1, d2])
    X, Y, labels = dataset.load('all')
    mean = np.mean(X, axis=0)
    std = np.std(X, axis=0)
    features = len(mean)
    X -= mean
    X /= std
    cds = ClassificationDataSet(features, class_labels=labels)
    for x, y in zip(X, Y):
        cds.addSample(x, y)
    print cds.calculateStatistics()
    tstdata, trndata = cds.splitWithProportion(0.2)
    trndata._convertToOneOfMany()
    tstdata._convertToOneOfMany()
    nn = buildNetwork(trndata.indim, 12, trndata.outdim, outclass=SoftmaxLayer)
    trainer = BackpropTrainer(nn, dataset=trndata, momentum=0.05, verbose=True,
                              weightdecay=0.01)
    trainer.trainUntilConvergence(maxEpochs=300)
    trnr = percentError(trainer.testOnClassData(), trndata['class'])
    tstr = percentError(trainer.testOnClassData(dataset=tstdata),
                        tstdata['class'])
    print "Epochs: %4d, train error: %5.2f%%, test error: %5.2f%%" % (
        trainer.totalepochs, trnr, tstr)
