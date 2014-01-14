#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_random_classifier')

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *
import rospy
import random

def handle_group_classification(req):

    res = GetGroupClassificationResponse()

    res.group_classification = list()
    
    for obj in req.object_id:
        
        obj_classification = ObjectClassification()

        obj_classification.object_id = obj
        obj_classification.type  = list()
        obj_classification.confidence  = list()

        for t in req.type:
            obj_classification.type.append(t)
            obj_classification.confidence.append(random.uniform(0,1))

        res.group_classification.append(obj_classification)

    return res

def random_classification_server():
    rospy.init_node('random_classification_server')
    s = rospy.Service('group_classification', GetGroupClassification, handle_group_classification)
    rospy.loginfo("Ready to classify groups")
    rospy.spin()

if __name__ == "__main__":
    random_classification_server()
