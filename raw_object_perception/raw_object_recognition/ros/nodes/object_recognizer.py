#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'
NODE = 'object_recognizer'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from raw_srvs.srv import RecognizeObject

SERVICE = 'recognize_object'

if __name__ == '__main__':
    rospy.init_node(NODE)

    def recognize_object_cb(request):
        rospy.loginfo('Received [%s] request.' % SERVICE)
        return 'unknown object'

    s = rospy.Service(SERVICE, RecognizeObject, recognize_object_cb)
    rospy.loginfo('Started [%s] service.' % SERVICE)
    rospy.spin()
