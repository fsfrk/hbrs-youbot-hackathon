#!/usr/bin/python

import pickle
import argparse
import numpy as np
import mlpy
import roslib; roslib.load_manifest('raw_object_recognition')

from raw_srvs.srv import *
import rospy

pca = None
svm = None

NAMES = ['Big nut', 'Big profile', 'Small profile', 'Small screw', 'Tube', 'Small nut']

def recognize(request):
    global pca
    global svm
    rospy.loginfo('Received request to recognize object.')
    v = request.dimensions.vector
    x = [v.x, v.y, v.z, request.points]
    z = pca.transform(x, k=2)
    print 'Principal components:', z
    klass = svm.pred(z)
    print 'Klass:', klass
    rospy.loginfo('Object was classified as %s.' % NAMES[int(klass)])
    return RecognizeObjectResponse(NAMES[int(klass)])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Object recognition node. Provides a service.
    ''')
    parser.add_argument('pca_svm', help='path to a file where PCA and SVM are stored')
    args = parser.parse_args()

    rospy.init_node('object_recognition_node')
    s = rospy.Service('recognize_object', RecognizeObject, recognize)
    '''
    try:
        with open(args.pca_svm, 'rb') as input:
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
    data = np.loadtxt('alldata.txt', delimiter=' ')
    x, y = data[:, :4], data[:, 4].astype(np.int)
    pca = mlpy.PCA(whiten=True)
    pca.learn(x)
    z = pca.transform(x, k=2)
    svm = mlpy.LibSvm(kernel=mlpy.KernelGaussian(sigma=1.0))
    svm.learn(z, y)

    rospy.loginfo('Started object recognition service.')
    rospy.spin()
