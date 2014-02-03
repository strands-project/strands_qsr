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

    # The results
    ground_truth = {}
    percept = {}
    relations = {}

    for i in range(len(res.group_classification)):
        obj_id = res.group_classification[i].object_id
        scores = zip(res.group_classification[i].type,
                     res.group_classification[i].confidence)
        random.shuffle(scores)
        cls, confidence = max(scores, key=lambda x:x[1])

        relations[obj_id] = dict(scores)

        scores = zip(perception[obj_id]["type"],
                     perception[obj_id]["confidence"])

        percept[obj_id] = dict(scores)

        scores = dict(zip(perception[obj_id]["type"], [0]*len(perception[obj_id]["type"])))
        scores[gt_types[res.group_classification[i].object_id]] = 1
        ground_truth[obj_id] = scores


    return ground_truth, percept, relations


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

    if os.path.exists(options.output_filename):
        print "ERROR: output file already exists; not proceeding."
        sys.exit(1)

    outfile = open(options.output_filename, "w")

    sum_perception = 0
    sum_relations = 0
    total = 0
    for scene in scenes:
        ## check if it is a scene for new data format
        if scene.get('scene_id'):

            gt, pc, rl = evaluate_scene(scene, perceptions[scene['scene_id']],
                                        object_types)
            for o in gt.keys():
                truth = max(gt[o].items(), key=lambda x:x[1])[0]
                percept = max(pc[o].items(), key=lambda x:x[1])[0]
                relate = max(rl[o].items(), key=lambda x:x[1])[0]
                if truth == percept:
                    sum_perception += 1
                if truth == relate:
                    sum_relations += 1
                total += 1
            #sum_perception += sum([1 if g==p else 0 for g, p in zip(gt, pc)])
            #sum_relations+= sum([1 if g==r else 0 for g, r in zip(gt, rl)])
            #total += len(gt)
            #print scene['scene_id'], sum([1 if g==r else 0 for g, r in zip(gt, rl)]), "/", len(gt)

            outfile.write("--\n")
            outfile.write(scene['scene_id']+"\n")
            for (text, scores) in [("ground-truth", gt), ("perception", pc),
                                   ("percep+rel", rl)]:
                outfile.write("%s\n"%text)
                outfile.write("#######\t")
                for k in object_types:
                    outfile.write(k+"\t")
                outfile.write("\n")
                for obj in scene['objects']:
                    if scene['type'][obj] == "UNKNOWN":
                        continue
                    outfile.write(obj+"\t")
                    for k in object_types:
                        if not scores[obj].has_key(k):
                            outfile.write("0.0")
                        else:
                            outfile.write("%f"%scores[obj][k])

                        outfile.write("\t")
                    outfile.write("\n")


    outfile.close()

    print "Perception performance:", 100 * (sum_perception / float(total))
    print "Relations performance:", 100 * (sum_relations/ float(total))