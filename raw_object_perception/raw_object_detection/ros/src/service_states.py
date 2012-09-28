#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'

import roslib
roslib.load_manifest(PACKAGE)

from smach import cb_interface
from smach_ros import ServiceState

import hbrs_srvs.srv as srv

find_workspace = ServiceState('find_workspace',
                              srv.FindWorkspace,
                              response_slots=['polygon'])

accumulate_tabletop_cloud = ServiceState('accumulate_tabletop_cloud',
                                         srv.AccumulateTabletopCloud,
                                         request_slots=['polygon'],
                                         response_slots=['cloud'])
cluster_tabletop_cloud = ServiceState('cluster_tabletop_cloud',
                                      srv.ClusterTabletopCloud,
                                      request_slots=['cloud', 'polygon'],
                                      response_slots=['clusters'])


@cb_interface(input_keys=['clusters', 'polygon'])
def make_boxes_request_cb(userdata, request):
    r = srv.MakeBoundingBoxesRequest()
    r.clouds = userdata.clusters
    r.axis.x = userdata.polygon.coefficients[0]
    r.axis.y = userdata.polygon.coefficients[1]
    r.axis.z = userdata.polygon.coefficients[2]
    return r


@cb_interface(input_keys=['bounding_boxes'], output_keys=['bounding_boxes'])
def make_boxes_response_cb(userdata, response):
    userdata.bounding_boxes = response.bounding_boxes
    for b in userdata.bounding_boxes:
        b.dimensions.x += 0.01

make_bounding_boxes = ServiceState('make_bounding_boxes',
                                   srv.MakeBoundingBoxes,
                                   request_cb=make_boxes_request_cb,
                                   response_cb=make_boxes_response_cb)
