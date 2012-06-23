#!/usr/bin/python

import roslib; roslib.load_manifest('raw_object_recognition')

from raw_srvs.srv import *
import rospy

if __name__ == '__main__':
    rospy.init_node('object_recognition_test')
    rospy.wait_for_service('recognize_object')
    try:
        recognize_object = rospy.ServiceProxy('recognize_object', RecognizeObject)
        request = RecognizeObjectRequest()
        request.dimensions.vector.x = 0.5
        request.dimensions.vector.y = 0.03
        request.dimensions.vector.z = 0.02
        request.points = 600
        request.color = 0.002800
        response = recognize_object(request)
        print response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

