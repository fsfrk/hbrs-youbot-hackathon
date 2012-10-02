#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'

import roslib
roslib.load_manifest(PACKAGE)
import sys
from os.path import join
import argparse

# Import helper classes for managing training data and saving network
sys.path.append(join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'src'))
from dataset import Dataset, JointDataset
from classification_network import ClassificationNetwork

# Import numpy and pybrain tools
import numpy as np
from pybrain.datasets import ClassificationDataSet
from pybrain.utilities import percentError
from pybrain.tools.shortcuts import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules import SoftmaxLayer


def to_string(np_array):
    return str(np_array.tolist())

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Learn and save neural network for object recognition.
    The network will be saved to the "common/config/" folder.
    ''')
    parser.add_argument('--datasets', help='list of datasets to use',
                        nargs='*', default='standard')
    parser.add_argument('--objects', help='list of objects to use',
                        nargs='*', default='all')
    parser.add_argument('--output', help='output filename (without folder and '
                        'extension, default: "network")', default='network')
    args = parser.parse_args()
    data_folder = join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'data')
    cfg_folder = join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'config')
    if isinstance(args.datasets, list):
        datasets = [Dataset(data_folder, name) for name in args.datasets]
        dataset = JointDataset(datasets)
    else:
        dataset = Dataset(data_folder, args.datasets)
    X, Y, labels = dataset.load(args.objects)
    mean = np.mean(X, axis=0)
    std = np.std(X, axis=0)
    X -= mean
    X /= std
    cds = ClassificationDataSet(len(mean), class_labels=labels)
    for x, y in zip(X, Y):
        cds.addSample(x, y)
    print cds.calculateStatistics()
    tstdata, trndata = cds.splitWithProportion(0.2)
    trndata._convertToOneOfMany()
    tstdata._convertToOneOfMany()
    nn = buildNetwork(trndata.indim, 12, trndata.outdim, outclass=SoftmaxLayer)
    trainer = BackpropTrainer(nn, dataset=trndata, momentum=0.05, verbose=True,
                              weightdecay=0.01)
    trainer.trainUntilConvergence(maxEpochs=400)
    trnr = percentError(trainer.testOnClassData(), trndata['class'])
    tstr = percentError(trainer.testOnClassData(dataset=tstdata),
                        tstdata['class'])
    print 'Epochs: %4d, train error: %5.2f%%, test error: %5.2f%%' % (
        trainer.totalepochs, trnr, tstr)
    ClassificationNetwork(nn, labels, mean, std).save(join(cfg_folder,
                                                           args.output))
