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
from optparse import OptionParser
import os
from time import time
        
if __name__ == "__main__":
    
    parser = OptionParser()
    parser.add_option("-o", "--out", dest="output_filename",
                      help="write scene perceptions to FILE", metavar="FILE")
    parser.add_option("-s", "--scenes",
                      dest="scenes_filename", metavar="FILE", 
                      help="load the scenes from FILE")
    parser.add_option("-p", "--perception",
                      dest="perception_model", metavar="P_TYPE", 
                      help="generate perceptions of type P_TYPE")
    
    parser.add_option("-t", "--object_type",
                      action="append", dest="object_types", metavar="OBJ_CLASS", 
                      help="conside objects of type OBJ_CLASS")
    
    parser.add_option("-a", "--all_objects",
                      action="store_true", dest="all_objects", default=False, 
                      help="conside all objects in scenes (replaces -t)")
    
    
    parser.add_option("-O", "--overwrite",
                      action="store_true", dest="overwrite", default=False, 
                      help="overwrite output file if already exists.")
    
    (options, args) = parser.parse_args()

    if not options.scenes_filename:
        parser.error("scenes file name is required")
    if not options.perception_model:
        parser.error("a perception model must be specified.")
    if not options.object_types and not options.all_objects:
        parser.error("at lease one object type must be specified.")
    if not options.output_filename:
        d = os.path.dirname(options.scenes_filename)
        f = os.path.basename(options.scenes_filename)
        options.output_filename = os.path.join(d, '../perception', 
                                               f[:-5]+ "_" + options.perception_model
                                                + ".json")
        #parser.error("output file name is required")
        print "Using scenes filename appended with model as output filename."

    seed =  time()
    random.seed(seed)


    if not os.path.isfile(options.scenes_filename):
        print "ERROR: scene file does not exist"
        sys.exit(1)
        
    with open(options.scenes_filename) as scn_file:
        scenes = json.load(scn_file)


        
    if options.all_objects:
        # consider all objects in scenes
        objects = set()
        for s in scenes:
            for c in s['type'].values():
                objects.add(c)
        options.object_types = list(objects)
        print "Using all objects: ", options.object_types
            

    init_method = options.perception_model
    perception_types = ( ['gt_{}'.format(i) for i in range(101)] +
                         ['random'] +
                         perception.PerceptionProb.get_library_model_names())
    if init_method not in perception_types:
        print "ERROR: perception model '" + init_method + "' is not known."
        sys.exit(1)
        
    perception_distribution = None

    for i in range(1, 101):
        if init_method == "gt_{}".format(i):
            perception_distribution = perception.PerceptionProb.create_simple(options.object_types, i/100.0)
            break
    else:
        if init_method == "random":
            perception_distribution = perception.PerceptionProb.create_simple(options.object_types,
                                                                                   1.0/len(options.object_types))
        else:
            perception_distribution = perception.PerceptionProb.create_from_library(init_method)
            
            


    winner = looser = 0
    generated_perception = {}
    for scene in scenes:
	## add check if is a scene for new data format
	if scene.get('scene_id'):
		generated_perception[scene['scene_id']] = {}
		for obj in scene['objects']:
		    if scene['type'][obj] not in options.object_types:
		        continue
		    truth = scene['type'][obj]
		    conf = perception_distribution.create_perception({obj: scene['type'][obj],})
		    scores = zip(conf[obj],  perception_distribution.object_types)
		    random.shuffle(scores)
		    percept = max(scores, key=lambda x: x[0])[1]
		    if percept == truth:
		        winner += 1
		    else:
		        looser += 1
		    generated_perception[scene['scene_id']][obj] = {}
		    generated_perception[scene['scene_id']][obj]['object_id'] = obj
		    generated_perception[scene['scene_id']][obj]['type'] = perception_distribution.object_types
		    generated_perception[scene['scene_id']][obj]['confidence'] = conf[obj]

    generated_perception['_meta'] = {}
    generated_perception['_meta']['perception_type'] = init_method
    generated_perception['_meta']['objects'] = options.object_types
    generated_perception['_meta']['scene_file'] = options.scenes_filename
    generated_perception['_meta']['stamp'] = time()
    generated_perception['_meta']['random_seed'] = seed
    
    
    if os.path.exists(options.output_filename) and not options.overwrite:
        print "The specified output file already exists; perception NOT saved; use -O to force"
        sys.exit(1)
        
    with open(options.output_filename, "w") as outfile:
        json.dump(generated_perception, outfile)
        
    print "Perception saved to file '" + options.output_filename + "'"
    print "Perception scores:", winner, " good, ", looser, " bad:", 100.0 * (winner / float(winner + looser)), "%"
                
                
