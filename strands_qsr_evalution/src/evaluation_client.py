#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_evalution')

import sys
import json
import random
import rospy
from strands_qsr_msgs.msg import *
from strands_qsr_msgs.srv import *

from geometry_msgs.msg import *

import perception

#OBJ_TYPES = ['Bottle',
 #            'Book',
 #            'Calculator',
 #            'Cup',
 #            'PC',
 #            'Glass',
 #            'Headphone',
 #            'Keyboard',
 #            'Keys',
 #            'Lamp',
 #            'Laptop',
 #            'MobilePhone',
 #            'Monitor',
 #            'Mouse',
 #            'Pencil',
 #            'Stapler',
 #            'Telephone']

OBJ_TYPES = ['Monitor',
             'Keyboard',
             'Mouse',
             'Papers',
             'Book',
             'Notebook',
	     'Mobile',
             'Mug',
             'Glass',
             'Flask',
             'Jug']


def classification_client(request):
    service_name = 'spatial_relation_classifier'
    rospy.wait_for_service(service_name)
    try:
        get_group_classification = rospy.ServiceProxy(service_name, GetGroupClassification)
        response = get_group_classification(request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def gen_object_classification(obj,types,perception_distribution):

    # Fake object perception
    # - purely random
    # - based on object occurrences throughout scenes
    # - ground truth

    obj_classification = ObjectClassification()
    
    obj_classification.type  = list()
    obj_classification.confidence  = list()
    obj_classification.object_id = obj
    
    conf =  perception_distribution.create_perception({obj: types[obj],})
    obj_classification.type = perception_distribution.object_types
    obj_classification.confidence = conf[obj]

    return obj_classification


def evaluate_scene(no,scn,perception_distribution):

    req = GetGroupClassificationRequest()

    req.type = OBJ_TYPES
    
    req.object_id = list()
    req.pose = list()
    req.bbox = list()
    req.group_classification = list()

    gt_types = dict()
    
    obj_id = 0
    for obj in scn['objects']:

        if scn['type'][obj] not in OBJ_TYPES:
            continue

        gt_types[obj] = scn['type'][obj]

        obj_name = 'obj' + str(obj_id)
        
        req.object_id.append(obj)
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

        req.group_classification.append(gen_object_classification(obj,scn['type'],perception_distribution))
             

    res = classification_client(req)


    # evaluation of the classification result
    tp = 0
    fn = 0

    other = 0

    for i in range(len(res.group_classification)):

        max_idx = res.group_classification[i].confidence.index(max(res.group_classification[i].confidence))
        cls = res.group_classification[i].type[max_idx] 
        gt  = gt_types[res.group_classification[i].object_id] # scn['type'][scn['objects'][i]]

        #print res.group_classification[i]
        #print max_idx
        print cls, '(', gt, ')', 
        print "confidence=", res.group_classification[i].confidence[max_idx]
            
        if gt not in req.type:
            other += 1

        if cls == gt:
            tp += 1
        else:
            fn += 1
            
    print no, "True positives:", tp
    print no, "False negative:", fn
    print no, "Other:", other
    
    performance = float(tp) / float((len(res.group_classification) - other))
    print no, "Performance:", performance

    return performance

def inc_count(dic, key):

    if key not in dic:
        dic[key] = 1
    else:
        dic[key] += 1
        

def usage():
    return ("\nevaluation_client.py <scene_file> <init_method> <start> <end>\n"
            "\n\t<scene_file>\tfile of test scenes\n"
            "\t<init_method>\tmethod to initialize object classes [random, occurrence, ground_truth_N, ~library~]\n"
            "\t<start>\t\tfirst scene to evaluate\n\t<end>\t\tlast scene to evaluate (optional)\n")

        
if __name__ == "__main__":

    if not (len(sys.argv) == 4 or len(sys.argv) == 5):
        print usage()
        sys.exit(0)

    init_method = sys.argv[2]
    #if not init_method in ['random', 'occurrence', 'ground_truth', 'ground_truth_60', 'ground_truth_70','ground_truth_80','ground_truth_90']:
        #print usage()
        #sys.exit(0)
        
    perception_distribution = None

    for i in range(1, 101):
        if init_method == "ground_truth_{}".format(i):
            perception_distribution =  p = perception.PerceptionProb.create_simple(OBJ_TYPES, i/100.0)
            break
    else:
        if init_method == "random":
            perception_distribution =  p = perception.PerceptionProb.create_simple(OBJ_TYPES, 1.0/len(OBJ_TYPES))
        else:
            perception_distribution =  p = perception.PerceptionProb.create_from_library(init_method)
            
        
        
    with open(sys.argv[1]) as scn_file:

        start = int(sys.argv[3])
        end = start
        if len(sys.argv) == 5:
            end = int(sys.argv[4])
            
        num_of_scenes = end - start + 1

        scenes = json.load(scn_file)
        
        sum_p = 0.0
        
        for i in range(start,end + 1):
            sum_p += evaluate_scene(i, scenes[i], perception_distribution)

        avg_p = sum_p / num_of_scenes
            
        print "Average performance:", avg_p

                
                
