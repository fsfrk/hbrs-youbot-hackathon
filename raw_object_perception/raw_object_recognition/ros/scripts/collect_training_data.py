#!/usr/bin/python

import roslib; roslib.load_manifest('raw_object_recognition')
import rospy
import argparse

from raw_srvs.srv import *

SERVICE_NAME = '/find_objects'

class Dataset:
    def __init__(self):
        self.file = open('dataset.txt', 'a')
    def __del__(self):
        self.file.close()
    def store(self, dim, pts, object_type):
        self.file.write('%.4f %.4f %.4f %i %s\n' % (dim[2], dim[1], dim[0], pts, object_type))

def get_one_sample():
    rospy.loginfo('Sending request...')
    response = get_objects()
    if not len(response.objects) == 1:
        rospy.loginfo('Found more than one object.')
        return None
    else:
        return response.objects[0]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Collect training data for object recognition.
    To store sample press Enter, to drop press any other key. 
    ''')
    parser.add_argument('object_type', help='type of the object we will learn')
    args = parser.parse_args()

    dataset = Dataset()
    rospy.loginfo('Waiting for find_objects service...')
    rospy.wait_for_service(SERVICE_NAME)
    get_objects = rospy.ServiceProxy(SERVICE_NAME, GetObjects)
    try:
        while True:
            object = get_one_sample()
            if not object:
                exit()
            vector = object.dimensions.vector
            dim = sorted([vector.x, vector.y, vector.z])
            pts = object.cluster.width
            rospy.loginfo('Found object: %.1f x %.1f x %.1f --- %i points. Store?' % (dim[2] * 100, dim[1] * 100, dim[0] * 100, pts))
            if not raw_input():
                dataset.store(dim, object.cluster.width, args.object_type)
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)
        exit()
