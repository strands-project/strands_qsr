#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_evalution')

import sys
import json
import random
import rospy
from strands_qsr_msgs.msg import *
from strands_qsr_msgs.srv import *

from geometry_msgs.msg import *

from optparse import OptionParser
import os


def classification_client(request):
    service_name = 'group_classification'
    rospy.wait_for_service(service_name)
    try:
        get_group_classification = rospy.ServiceProxy(service_name, GetGroupClassification)
        response = get_group_classification(request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def evaluate_scene(scene, perception, object_types):

    req = GetGroupClassificationRequest()

    req.type = object_types
    
    req.object_id = list()
    req.pose = list()
    req.bbox = list()
    req.group_classification = list()

    gt_types = dict()
    percept_types = dict()
    
    obj_id = 0
    for obj in scene['objects']:

        if scene['type'][obj] not in object_types:
            continue

        gt_types[obj] = scene['type'][obj]

        obj_name = 'obj' + str(obj_id)
        
        req.object_id.append(obj)
        obj_id += 1
        
        pose = Pose()
        pose.position.x = scene['position'][obj][0]
        pose.position.y = scene['position'][obj][1]
        pose.position.z = scene['position'][obj][2]
        
        pose.orientation.w = scene['orientation'][obj][0]
        pose.orientation.x = scene['orientation'][obj][1]
        pose.orientation.y = scene['orientation'][obj][2]
        pose.orientation.z = scene['orientation'][obj][3]

        req.pose.append(pose)
            
        bbox = BBox()
        points = scene['bbox'][obj]
        for p in points:
            point = Point32()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            bbox.point.append(point)
            
        req.bbox.append(bbox)
        
        obj_classification = ObjectClassification()
        obj_classification.object_id = obj
        obj_classification.type = perception[obj]["type"]
        obj_classification.confidence= perception[obj]["confidence"]
        
        scores = zip(perception[obj]["type"],   perception[obj]["confidence"])
        random.shuffle(scores)
        percept_types[obj], conf = max(scores, key=lambda x:x[1])
        
        req.group_classification.append(obj_classification)
             

    res = classification_client(req)

    # The restults
    ground_truth = []
    perception = []
    relations = []

    for i in range(len(res.group_classification)):

        scores = zip(res.group_classification[i].confidence,
                     res.group_classification[i].type)
        random.shuffle(scores)
        confidence, cls = max(scores, key=lambda x:x[0])
        
        ground_truth.append(gt_types[res.group_classification[i].object_id])
        perception.append(percept_types[res.group_classification[i].object_id])
        relations.append(cls)

    
    return ground_truth, perception, relations

        
if __name__ == "__main__":
   
    parser = OptionParser()
    parser.add_option("-p", "--perception_filename", dest="perception_filename",
                      help="read perceptions from FILE", metavar="FILE")
    parser.add_option("-s", "--scenes",
                      dest="scenes_filename", metavar="FILE", 
                      help="load the scenes from FILE")
    parser.add_option("-o", "--output",
                      dest="output_filename", metavar="FILE", 
                      help="store results in FILE")
    
    parser.add_option("-O", "--overwrite",
                      action="store_true", dest="overwrite", default=False, 
                      help="overwrite output file if already exists.")
    
    (options, args) = parser.parse_args()

    if not options.output_filename:
        parser.error("output file name is required")
    if not options.scenes_filename:
        parser.error("scenes file name is required")
    if not options.perception_filename:
        parser.error("a perception file must be specified.")
    
    if not os.path.isfile(options.scenes_filename):
        print "ERROR: scene file does not exist"
        sys.exit(1)
        
    with open(options.scenes_filename) as scn_file:
        scenes = json.load(scn_file)
        
    if not os.path.isfile(options.perception_filename):
        print "ERROR: perception file does not exist"
        sys.exit(1)
        
    with open(options.perception_filename) as p_file:
        perceptions = json.load(p_file)
        

    object_types = perceptions["_meta"]["objects"]
    
    print "Running with object types from perception file:", object_types

    if os.path.exists(options.output_filename):
        print "ERROR: output file already exists; not proceeding."
        sys.exit(1)
    
    outfile = open(options.output_filename, "w")
    
    sum_perception = 0
    sum_relations = 0
    total = 0
    for scene in scenes:
        gt, pc, rl = evaluate_scene(scene, perceptions[scene['scene_id']],
                              object_types)
        sum_perception += sum([1 if g==p else 0 for g, p in zip(gt, pc)])
        sum_relations+= sum([1 if g==r else 0 for g, r in zip(gt, rl)])
        total += len(gt)
        print scene['scene_id'], sum([1 if g==r else 0 for g, r in zip(gt, rl)]), "/", len(gt)
        
        outfile.write("--\n")
        outfile.write(scene['scene_id']+"\n")
        for s in [gt, pc, rl]:
            for i in s:
                outfile.write(i + "\t")
            outfile.write("\n")
        
    outfile.close()

    print "Perception performance:", 100 * (sum_perception / float(total))
    print "Relations performance:", 100 * (sum_relations/ float(total))
