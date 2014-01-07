#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_tpc_classifier')

import sys
import json
import rospy
from strands_qsr_msgs.msg import *
from strands_qsr_msgs.srv import *
from geometry_msgs.msg import *

def tpc_classification_client(request):
    service_name = 'group_classification'
    rospy.wait_for_service(service_name)
    try:
        get_group_classification = rospy.ServiceProxy(service_name, GetGroupClassification)
        response = get_group_classification(request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "\ntpc_classification_client.py <scene_file> <scene_no>\n\n\t<scene_file>\tfile including scene\n\t<scene_no>\tscene number to evalute\n"

        
if __name__ == "__main__":

    if len(sys.argv) != 3:
        print usage()
        sys.exit(0)

    with open(sys.argv[1]) as scn_file:    
        scenes = json.load(scn_file)

        scn = scenes[int(sys.argv[2])]

        req = GetGroupClassificationRequest()

        req.type = ['Bottle',
#                    'Calculator',
                    'Cup',
                    'PC',
#                    'Glass',
#                    'Headphone',
                    'Keyboard',
#                    'Keys',
#                    'Lamp',
#                    'Laptop',
#                    'MobilePhone',
                    'Monitor',
                    'Mouse'] #,
#                    'Pencil',
#                    'Stapler',
#                    'Telephone']

        req.object_id = list()
        req.pose = list()
        req.bbox = list()
        req.group_classification = list()

        obj_id = 0
        for obj in scn['objects']:

            req.object_id.append('obj' + str(obj_id))
            obj_id += 1
            
            pose = Pose()
            pose.position.x = scn['position'][obj][0]
            pose.position.y = scn['position'][obj][1]
            pose.position.z = scn['position'][obj][2]

            pose.orientation.w = scn['orientation'][obj][0]
            pose.orientation.x = scn['orientation'][obj][1]
            pose.orientation.y = scn['orientation'][obj][2]
            pose.orientation.z = scn['orientation'][obj][3]

            req.pose.append(pose)
            
            bbox = BBox()
            points = scn['bbox'][obj]
            for p in points:
                point = Point32()
                point.x = p[0]
                point.y = p[1]
                point.z = p[2]
                bbox.point.append(point)

            req.bbox.append(bbox)

            req.group_classification.append(ObjectClassification())
             


        res = tpc_classification_client(req)


        tp = 0
        fn = 0

        other = 0
        for i in range(len(res.group_classification)):

            cls = res.group_classification[i].type[0] 
            gt  = scn['type'][scn['objects'][i]]

            print cls, '(', gt, ')'
            
            if gt not in req.type:
                other += 1


            if cls == gt:
                tp += 1
            else:
                fn += 1


        print "True positives:", tp
        print "False negative:", fn
        print "Other:", other
        print "Performance:", float(tp) / float((len(res.group_classification) - other))
        


                
                
