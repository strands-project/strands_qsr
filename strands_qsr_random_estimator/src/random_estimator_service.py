#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_random_estimator')

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *
import rospy
import random

def handle_group_estimate(req):

    res = GetGroupEstimateResponse()

    res.estimate = list()
    
    for obj in req.object_id:
        
        est = GroupEstimate()

        est.object_id = obj
        est.type  = list()
        est.weight  = list()

        for t in req.type:
            est.type.append(t)
            est.weight.append(random.uniform(0,1))

        res.estimate.append(est)

    return res

def random_estimator_server():
    rospy.init_node('random_estimator_server')
    s = rospy.Service('random_group_estimate', GetGroupEstimate, handle_group_estimate)
    rospy.loginfo("Ready to estimate groups")
    rospy.spin()

if __name__ == "__main__":
    random_estimator_server()
