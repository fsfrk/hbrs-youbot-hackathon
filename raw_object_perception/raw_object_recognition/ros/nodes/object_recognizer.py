#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'
NODE = 'object_recognizer'
SERVICE = 'recognize_object'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import sys
from os.path import join
import numpy as np

# Import helper class for loading trained network
sys.path.append(join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'src'))
from classification_network import ClassificationNetwork

from raw_srvs.srv import RecognizeObject
from raw_srvs.srv import AnalyzeCloudColor


def load_neural_network(filename):
    cfg_folder = join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'config')
    return ClassificationNetwork.load(join(cfg_folder, filename))

if __name__ == '__main__':
    rospy.init_node(NODE)
    net = load_neural_network(sys.argv[1])
    analyze = rospy.ServiceProxy('analyze_cloud_color', AnalyzeCloudColor)

    def recognize_object_cb(request):
        rospy.loginfo('Received [%s] request.' % SERVICE)
        r = analyze(request.cloud)
        d = request.dimensions
        d = sorted([d.x, d.y, d.z], reverse=True)
        features = np.array([d[0], d[1], d[2], r.points, r.mean, r.median])
        c, l = net.classify(features)
        return l

    s = rospy.Service(SERVICE, RecognizeObject, recognize_object_cb)
    rospy.loginfo('Started [%s] service.' % SERVICE)
    rospy.spin()
