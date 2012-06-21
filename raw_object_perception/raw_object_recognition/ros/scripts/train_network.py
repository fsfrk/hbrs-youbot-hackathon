#!/usr/bin/python

from pybrain.datasets import ClassificationDataSet
import argparse

from pybrain.utilities           import percentError
from pybrain.tools.shortcuts     import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules   import SoftmaxLayer
from pylab import ion, ioff, figure, draw, contourf, clf, show, hold, plot
from scipy import diag, arange, meshgrid, where
from numpy.random import multivariate_normal

def load_dataset(split=0.25):
    ds = ClassificationDataSet(4, class_labels=['Profile', 'Nut'])
    for line in open('dataset.txt', 'r'):
        values = line.split(' ')
        attr = [float(v) for v in values[0:4]]
        klass = int(values[4])
        ds.addSample(attr, [klass])
    ds.calculateStatistics()
    print 'Number of classes:', ds.nClasses
    print 'Class histogram:', ds.classHist
    test_data, train_data = ds.splitWithProportion(split)
    test_data._convertToOneOfMany()
    train_data._convertToOneOfMany()
    print "Number of training patterns: ", len(train_data)
    print "Input and output dimensions: ", train_data.indim, train_data.outdim
    return test_data, train_data

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    ''')
    args = parser.parse_args()
    test_data, train_data = load_dataset()
    fnn = buildNetwork(train_data.indim, 5, train_data.outdim, outclass=SoftmaxLayer)
    trainer = BackpropTrainer(fnn, dataset=train_data, momentum=0.1, verbose=True, weightdecay=0.01)
    for i in range(20):
        trainer.trainEpochs(1)
        train_result = percentError(trainer.testOnClassData(), train_data['class'])
        test_result = percentError(trainer.testOnClassData(dataset=test_data), test_data['class'])
        print "epoch: %4d" % trainer.totalepochs, \
              "  train error: %5.2f%%" % train_result, \
              "  test error: %5.2f%%" % test_result
