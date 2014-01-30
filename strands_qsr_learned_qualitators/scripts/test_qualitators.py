#!/usr/bin/python

# Create 'qualitators' from a training set.

from optparse import OptionParser
import strands_qsr_learned_qualitators.geometric_state as gs
import strands_qsr_learned_qualitators.density as ds
from strands_qsr_learned_qualitators import qualitators, scene
import json
import numpy as np
import rospy

if __name__ == '__main__':
    rospy.init_node("vistest", anonymous=True)
    ''' Main Program '''
    print "Loading data file"
    #with open("data/simulation/bham_2000.json", "r") as f:
    #with open("data/simulation/bham_office_desk_500.json", "r") as f:
    with open("/home/chris/catkin_ws/src/strands_qsr/data/random-foldings/50-percent-train/TrainData_50p_0.json", "r") as f:
    #with open("/home/chris/catkin_ws/src/strands_qsr/data/real-world/131110_All_Scenes.json", "r") as f:
        scenes = json.load(f)
    geo_states = [gs.GeometricState.from_scene_data(s) for s in scenes]

    qual = qualitators.Qualitators.load_from_disk("/home/chris/catkin_ws/src/strands_qsr/results/random-foldings/chris/tpc_abs_size/TrainData_50p_0.qsrs")
    #print "Loaded ", len(qual._qualitators), "qualitators from disk"
    
    #rospy.loginfo("Loading scene probabilities")
    #prob =  scene.RelationProbabilities.load_from_disk("/home/chris/catkin_ws/src/strands_qsr/results/random-foldings/chris/tpc_abs_size/TrainData_50p_0.scene")

    for geo in geo_states:
        s =  qual.create_qualitative_state(geo)
        for c in s._clauses:
            #p = prob.get_prob(c[0], (geo.get_object_type(c[1]), geo.get_object_type(c[2])))
            print c #, "prob=", p
        viz = gs.GeometricStateViz(geo)
        viz.publish_visualisation()
        if rospy.is_shutdown():
            break
        raw_input(">")
        
