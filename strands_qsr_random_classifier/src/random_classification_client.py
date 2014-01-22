#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_random_classifier')

import sys

import rospy
from strands_qsr_msgs.msg import *
from strands_qsr_msgs.srv import *
from geometry_msgs.msg import *

def random_classification_client(request):
    service_name = 'group_classification'
    rospy.wait_for_service(service_name)
    try:
        get_group_classification = rospy.ServiceProxy(service_name, GetGroupClassification)
        response = get_group_classification(request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "\nrandom_classification_client.py <number_of_objects>\n\n\t<number of objects>\tnumber of objects in the scene\n"

        
if __name__ == "__main__":

    if len(sys.argv) != 2:
        print usage()
        sys.exit(0)

    req = GetGroupClassificationRequest()

    req.type = ['Monitor','Keyboard','Mouse', 'Cup', 'Bottle', 'MobilePhone']

    req.object_id = list()
    for i in range(int(sys.argv[1])):
        req.object_id.append('obj' + str(i))
        

    req.pose = list()
    req.bbox = list()
    req.group_classification = list()
    
    for i in range(len(req.object_id)):
        req.pose.append(Pose())
        req.bbox.append(BBox())
        req.group_classification.append(ObjectClassification())
        
    
    print random_classification_client(req)
