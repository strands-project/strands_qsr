#!/usr/bin/python

# Create 'qualitators' from a training set.

from optparse import OptionParser
import strands_qsr_learned_qualitators.geometric_state as gs
import strands_qsr_learned_qualitators.density as ds
from strands_qsr_learned_qualitators import qualitators, scene
import json
import numpy as np
import os
import sys

if __name__ == '__main__':
    ''' Main Program '''
    parser = OptionParser()
    parser.add_option("-s", "--scenes",
                      dest="scenes_filename", metavar="FILE", 
                      help="load the scenes from FILE")
    parser.add_option("-q", "--qualitators",
                      dest="qualitators_filename", metavar="FILE", 
                      help="load the qualitators from FILE")
    parser.add_option("-o", "--output",
                      dest="output_filename", metavar="FILE", 
                      help="store qualitators in FILE")
    
    parser.add_option("-O", "--overwrite",
                      action="store_true", dest="overwrite", default=False, 
                      help="overwrite output file if already exists.")
    
    (options, args) = parser.parse_args()
    
    if not options.output_filename:
        parser.error("output file name is required")
    if not options.scenes_filename:
        parser.error("scenes file name is required")        
    if not options.qualitators_filename:
        parser.error("qualitators file name is required")        
    
    if not os.path.isfile(options.scenes_filename):
        print "ERROR: scene file does not exist"
        sys.exit(1)
      
    if not os.path.isfile(options.qualitators_filename):
        print "ERROR: qualitators file does not exist"
        sys.exit(1)
      

    print "Loading data file"
    #SCENES_FILE = "/home/chris/catkin_ws/src/strands_qsr/data/random-foldings/50-percent-train/TrainData_50p_0.json"
    #with open("/home/chris/catkin_ws/src/strands_qsr/data/real-world/131110_All_Scenes.json", "r") as f:
    #with open("data/real-world/Two_Days_All_Scenes.json", "r") as f:
    #with open("data/simulation/bham_2000.json", "r") as f:
    #with open("data/simulation/bham_office_desk_500.json", "r") as f:
    
    with open(options.scenes_filename, "r") as f:
        scenes = json.load(f)

    geo_states = [gs.GeometricState.from_scene_data(s) for s in scenes]

    qual = qualitators.Qualitators.load_from_disk(options.qualitators_filename)
    print "Loaded ", len(qual._qualitators), "qualitators from disk"
    rel_probs =  scene.RelationProbabilities()

    for i, geo in enumerate(geo_states):

        for ob1 in geo._objects.keys():
            for ob2 in geo._objects.keys():
                if ob1 == ob2:
                    continue
                delta = [a - b for a, b in zip(geo._objects[ob1].position,
                                               geo._objects[ob2].position)]
                s1 = geo._objects[ob1].bbox.get_size()
                s2 = geo._objects[ob2].bbox.get_size()
                for q in qual._qualitators:
                    if q.dimensions == 2:
                        rel_probs.update_probs_new_example(q.name,
                                                           (geo._objects[ob1].obj_type,
                                                            geo._objects[ob2].obj_type),
                                                           q(delta+list(s1)+list(s2)))
            for q in qual._qualitators:
                if q.dimensions == 1:
                    s1 = geo._objects[ob1].bbox.get_size()
                    rel_probs.update_probs_new_example(q.name,
                                                       (geo._objects[ob1].obj_type, ),
                                                       q(geo._objects[ob1].position+list(s1)))
                    
        
        #print ".", i, "."
        
    #print
    
     
    if os.path.exists(options.output_filename) and not options.overwrite:
        print "ERROR: output file already exists; not proceeding. Use -O to overwrite."
        sys.exit(1)
   
    rel_probs.save_to_disk(options.output_filename)
        

    #for i in rel_probs.probabilities:
        #for j in rel_probs.probabilities[i]:
            #print j, rel_probs.probabilities[i][j]
