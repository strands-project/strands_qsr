#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_random_estimator')

import sys

import rospy
from strands_qsr_msgs.msg import *
from strands_qsr_msgs.srv import *
from geometry_msgs.msg import *

def random_estimator_client(request):
    service_name = 'random_group_estimate'
    rospy.wait_for_service(service_name)
    try:
        get_group_estimate = rospy.ServiceProxy(service_name, GetGroupEstimate)
        response = get_group_estimate(request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "\nrandom_estimator_client.py <number_of_objects>\n\n\t<number of objects>\tnumber of objects in the scene\n"

        
if __name__ == "__main__":

    if len(sys.argv) != 2:
        print usage()
        sys.exit(0)

    req = GetGroupEstimateRequest()

    req.type = ['Monitor','Keyboard','Mouse', 'Cup', 'Bottle', 'MobilePhone']

    req.object_id = list()
    for i in range(int(sys.argv[1])):
        req.object_id.append('obj' + str(i))
        

    req.pose = list()
    req.bbox = list()
    req.estimate = list()
    
    for i in range(len(req.object_id)):
        req.pose.append(Pose())
        req.bbox.append(BBox())
        req.estimate.append(GroupEstimate())
        
    
    print random_estimator_client(req)
