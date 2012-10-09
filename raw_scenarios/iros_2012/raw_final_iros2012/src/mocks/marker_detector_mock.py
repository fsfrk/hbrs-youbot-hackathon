#!/usr/bin/env python

PACKAGE = 'raw_final_iros2012'
NODE = 'marker_detector_mock'
SERVICE = 'detect_markers'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import random

from hbrs_msgs.msg import Object
from hbrs_srvs.srv import GetObjects, GetObjectsResponse


def create_object(name):
    obj = Object()
    obj.name = name
    return obj

def detect_markers_cb(request):
    rospy.loginfo('Received [%s] request.' % SERVICE)
    r = random.random()
    response = GetObjectsResponse()
    response.stamp = rospy.Time.now()
    if r > 0.8: # no markers found
        return response
    if r > 0.6: # two markers found
        response.objects.append(create_object('B1'))
    if r > 0.3: # one marker found
        response.objects.append(create_object('B2'))
    else:
        response.objects.append(create_object('B3'))
    return response

if __name__ == '__main__':
    rospy.init_node(NODE)
    detect_markers_srv = rospy.Service(SERVICE, GetObjects, detect_markers_cb)
    rospy.loginfo('Started MOCK [%s] service.' % SERVICE)
    rospy.spin()
