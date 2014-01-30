#!/usr/bin/python

# Visualise a JSON scenes file in RViz

import rospy
import strands_qsr_learned_qualitators.geometric_state as gs
from optparse import OptionParser
import json
import os

if __name__ == '__main__':
    rospy.init_node("vistest", anonymous=True)
    ''' Main Program '''
    
    parser = OptionParser()
    parser.add_option("-s", "--scenes",
                      dest="scenes_filename", metavar="FILE", 
                      help="load the scenes from json file FILE")
    (options, args) = parser.parse_args()
    
    if not options.scenes_filename:
        parser.error("scenes file name is required")        
        
    if not os.path.isfile(options.scenes_filename):
        print ("ERROR: scenes file '" + options.probs_filename +
               "' does not exist")
        sys.exit(1)
      
    print "Loading data file"
    with open(options.scenes_filename, "r") as f:
        scenes = json.load(f)
    geo_states = [gs.GeometricState.from_scene_data(s) for s in scenes]

    for i, geo in enumerate(geo_states):
        viz = gs.GeometricStateViz(geo)
        viz.publish_visualisation()
        rospy.sleep(0.1)
        if rospy.is_shutdown():
            break
        raw_input("<[{}/{}] Markers published, press return for next >".format(
            i+1, len(geo_states) )
                  )
        
