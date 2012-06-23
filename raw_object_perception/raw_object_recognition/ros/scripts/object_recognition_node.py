#!/usr/bin/python

import pickle
import argparse
import numpy as np
import mlpy
import roslib; roslib.load_manifest('raw_object_recognition')
import sys
from raw_srvs.srv import *
import rospy

pca = None
svm = None

NAMES = ['Big nut', 'Big profile', 'Small profile', 'Small screw', 'Tube', 'Small nut', 'Big screw', 'Small profile silver', 'Big profile silver']

def recognize(request):
    global pca
    global svm
    rospy.loginfo('Received request to recognize object.')
    v = request.dimensions.vector
    x = [v.x, v.y, v.z, request.points, request.color]
    z = pca.transform(x, k=2)
    print 'Principal components:', z
    klass = svm.pred(z)
    print 'Klass:', klass
    rospy.loginfo('Object was classified as %s.' % NAMES[int(klass)])
    return RecognizeObjectResponse(NAMES[int(klass)])

if __name__ == '__main__':
    rospy.init_node('object_recognition_node')
    s = rospy.Service('recognize_object', RecognizeObject, recognize)
    '''
    try:
        with open(sys.argv[1], 'rb') as input:
            global pca
            global svm
            pca = pickle.load(input)
            svm = pickle.load(input)
    except:
        rospy.logfatal('Unable to load PCA and SVM from file "%s".' % args.pca_svm)
        exit()
    '''
    global pca
    global svm
    data = np.loadtxt(sys.argv[1], delimiter=' ')
    x, y = data[:, :5], data[:, 5].astype(np.int)
    pca = mlpy.PCA(whiten=True)
    pca.learn(x)
    z = pca.transform(x, k=2)
    svm = mlpy.LibSvm(kernel=mlpy.KernelGaussian(sigma=1.0))
    svm.learn(z, y)

    rospy.loginfo('Started object recognition service.')
    rospy.spin()
