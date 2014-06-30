#!/usr/bin/python

# Create 'qualitators' from a training set.

from optparse import OptionParser
import strands_qsr_learned_qualitators.geometric_state as gs
import strands_qsr_learned_qualitators.density as ds
from strands_qsr_learned_qualitators import qualitators
import json
import numpy as np
import os
import sys
import math

import matplotlib
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.patches import Ellipse

from scipy.cluster.hierarchy import linkage
from scipy.cluster.vq import vq, kmeans

if __name__ == '__main__':
    ''' Main Program '''
    
    parser = OptionParser()
    parser.add_option("-s", "--scenes",
                      dest="scenes_filename", metavar="FILE", 
                      help="load the scenes from FILE")
    parser.add_option("-o", "--output",
                      dest="output_filename", metavar="FILE", 
                      help="store qualitators in FILE")
    parser.add_option("-t", "--type",
                      dest="qsr_type", metavar="{tpc,dist,relsize,connect},"
                      "underscore sep list",
                      help="type of qsrs")
    
    parser.add_option("-O", "--overwrite",
                      action="store_true", dest="overwrite", default=False, 
                      help="overwrite output file if already exists.")
    
    (options, args) = parser.parse_args()


    if not options.output_filename:
        parser.error("output file name is required")
    if not options.scenes_filename:
        parser.error("scenes file name is required")        
    if not options.qsr_type:
        options.qsr_type = []
    else:
        options.qsr_type = options.qsr_type.split("_")
        
    print "Types:", options.qsr_type
        
    #print "Loading data file"
    #if not os.path.isfile(options.scenes_filename):
        #print "ERROR: scene file does not exist"
        #sys.exit(1)
       
    #with open(options.scenes_filename, "r") as f:
        #scenes = json.load(f)
    #geo_states = [gs.GeometricState.from_scene_data(s) for s in scenes if s.get('scene_id')]
    #print "Loaded."
    #distances = []
    #sizes = []
    
    #for geo in geo_states:
        #for ob1 in geo._objects.keys():
            #for ob2 in geo._objects.keys():
                #if ob1 == ob2:
                    #continue
                #delta = [(a - b) * (a - b) for a, b in zip(geo._objects[ob1].position,
                                               #geo._objects[ob2].position)]
                #distances.append( math.sqrt(sum(delta)) )
            #sizes.append(geo._objects[ob2].bbox.get_volume())
        
    #print "Clustering sizes and distances..."
    #dists = sorted(kmeans(np.array(distances), 3)[0])
    #number_size_clusters = 15
    #sizs = sorted(kmeans(np.array(sizes), number_size_clusters)[0])
    #number_size_clusters = len(sizs)
    #print max(np.array(distances))
    #print dists
    #print sizs
        
    qs =  qualitators.Qualitators()
    
    ##Using Lars' Two Point stuff
    ############################
    ## TPC
    if "tpc" in options.qsr_type:
        qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("front", (0, 1, 7)))
        qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("right", (1, 2, 3)))
        qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("behind", (3, 4, 5)))
        qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("left", (5, 6, 7)))
    ############################
    
    
    
    #######################
    ## Distance
    if "dist" in options.qsr_type:
        #qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("veryclose", (0, (dists[0]+dists[1])/2.0)))
        #qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("close", ((dists[0]+dists[1])/2.0,
                                                                           #(dists[1]+dists[2])/2.0)))
        #qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("distant", ((dists[1]+dists[2])/2.0,
                                                                             #1e100)))
        qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("veryclose", (0, 0.15)))
        qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("close", (0.15 , 0.4 )))
        qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("distant", (0.4 , 1e100)))
    #######################
    
    #qs.add_qualitator(qualitators.AbsoluteSizeQualitator("sz0", (0, (sizs[0]+sizs[1])/2.0)))
    #for i in range(1, number_size_clusters-1):
        #qs.add_qualitator(qualitators.AbsoluteSizeQualitator("sz%d"%i, ((sizs[i-1]+sizs[i])/2.0,
                                                                           #(sizs[i]+sizs[i+1])/2.0)))
    #qs.add_qualitator(qualitators.AbsoluteSizeQualitator("sz%d"%(i+1), ((sizs[i]+sizs[i+1])/2.0,
                                                                       #1e1000)))
        
    ##qs.add_qualitator(qualitators.AbsoluteSizeQualitator("small", ((sizs[0]+sizs[1])/2.0,
                                                                       ##(sizs[1]+sizs[2])/2.0)))
    ##qs.add_qualitator(qualitators.AbsoluteSizeQualitator("large", ((sizs[1]+sizs[2])/2.0,
                                                                         ##1e100)))
    ##########################
    ## Relative size
    if "relsize" in options.qsr_type:
        qs.add_qualitator(qualitators.SizeQualitator("wider", 0))
        qs.add_qualitator(qualitators.SizeQualitator("deeper", 1))
        qs.add_qualitator(qualitators.SizeQualitator("taller", 2))
    ##########################
    
    ##########################
    ## Connectivity
    if "connectivity" in options.qsr_type:
        qs.add_qualitator(qualitators.ProjectionConectivity("x-connected", 0))
        qs.add_qualitator(qualitators.ProjectionConectivity("y-connected", 1))
    ##########################
    
        
    if os.path.exists(options.output_filename) and not options.overwrite:
        print "ERROR: output file already exists; not proceeding. Use -O to overwrite."
        sys.exit(1)
   
    qs.save_to_disk(options.output_filename)