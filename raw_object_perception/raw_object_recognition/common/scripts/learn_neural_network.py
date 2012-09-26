#!/usr/bin/python

import numpy as np
from pybrain.datasets import ClassificationDataSet
from pybrain.utilities import percentError
from pybrain.tools.shortcuts import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules import SoftmaxLayer

from dataset import Dataset

if __name__ == '__main__':
    dataset = Dataset('../data/', 'standard')
    X, Y, labels = dataset.load('all')
    mean = np.mean(X, axis=0)
    features = len(mean)
    X -= mean
    cds = ClassificationDataSet(features, class_labels=labels)
    for x, y in zip(X, Y):
        cds.addSample(x, y)
    tstdata, trndata = cds.splitWithProportion(0.2)
    trndata._convertToOneOfMany()
    tstdata._convertToOneOfMany()
    nn = buildNetwork(trndata.indim, 16, trndata.outdim, outclass=SoftmaxLayer)
    trainer = BackpropTrainer(nn, dataset=trndata, momentum=0.05, verbose=True,
                              weightdecay=0.01)
    trainer.trainUntilConvergence(maxEpochs=500)
    trnr = percentError(trainer.testOnClassData(), trndata['class'])
    tstr = percentError(trainer.testOnClassData(dataset=tstdata),
                        tstdata['class'])
    print "Epochs: %4d, train error: %5.2f%%, test error: %5.2f%%" % (
        trainer.totalepochs, trnr, tstr)
