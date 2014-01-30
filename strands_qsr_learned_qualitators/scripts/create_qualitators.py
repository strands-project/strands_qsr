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
    
    parser.add_option("-O", "--overwrite",
                      action="store_true", dest="overwrite", default=False, 
                      help="overwrite output file if already exists.")
    
    (options, args) = parser.parse_args()


    if not options.output_filename:
        parser.error("output file name is required")
    if not options.scenes_filename:
        parser.error("scenes file name is required")        
    
    
    print "Loading data file"
    #with open("data/simulation/bham_office_desk_500.json", "r") as f:
    #with open("data/real-world/Two_Days_All_Scenes.json", "r") as f:
    #with open("/home/chris/catkin_ws/src/strands_qsr/data/real-world/131110_All_Scenes.json", "r") as f:
    #with open("data/simulation/bham_2000.json", "r") as f:
    #with open("/home/chris/catkin_ws/src/strands_qsr/data/random-foldings/50-percent-train/TrainData_50p_0.json", "r") as f:
    if not os.path.isfile(options.scenes_filename):
        print "ERROR: scene file does not exist"
        sys.exit(1)
       
    with open(options.scenes_filename, "r") as f:
        scenes = json.load(f)
    geo_states = [gs.GeometricState.from_scene_data(s) for s in scenes]

    relatives = []

    great_plot = {} #turned out not to be so great :-(
    for geo in geo_states:
        #print "Processing State: ", geo._scene_id
        obs = [geo._objects[o].obj_type for o in geo._objects.keys()]
        print obs
        #print "Objects {}: {}".format(len(obs), obs)
        
        for ob1 in geo._objects.keys():
            for ob2 in geo._objects.keys():
                if ob1 == ob2:
                    continue
                pair = "{}-{}".format(geo._objects[ob1].obj_type,
                                      geo._objects[ob2].obj_type )
                delta = [a - b for a, b in zip(geo._objects[ob1].position,
                                               geo._objects[ob2].position)]
                s1 = geo._objects[ob1].bbox.get_size()
                s2 = geo._objects[ob2].bbox.get_size()
                relatives.append( delta  + list(s1) + list(s2) )
                if not great_plot.has_key(pair):
                    great_plot[pair] = []
                great_plot[pair].append(delta + list(s1) + list(s2) )
        
    print "Have", len(relatives), " relation sample points"
    print "Number object pair types:", len(great_plot.keys())
    print "Clustering samples..."
    
    Z = kmeans(np.array(relatives), 20)
    print "Kmeans clustering done, creating Gaussians"
    clusters = vq(np.array(relatives), Z[0])
    print clusters
    print ".."
    qs =  qualitators.Qualitators()
    for i, cluster in enumerate(Z[0]):
        s = []
        for pt, sample in zip(clusters[0], relatives):
            #print pt
            if pt == i:
                #print pt, "==", i
                s.append(sample)
        print np.array(s)
        g = ds.GaussKernel(np.array(s), 1)
        q =  qualitators.SimpleGaussianQualitator("rel%d"%i, 2, g)
        qs.add_qualitator(q)
        
    ## TODO: tidy this as it is hacked from some old code, 
    #dens = ds.DensityEstimator.createFromData(np.array(relatives), 0.8)

    #qs =  qualitators.Qualitators()
    #for i, k in enumerate(dens.kernels):
        #q =  qualitators.SimpleGaussianQualitator("rel%d"%i, 2, k)
        #qs.add_qualitator(q)
    
    #print "Number of QSRs:", len(dens.kernels)
    print "Number of QSRs:", len(qs)
    
    ########################################################
    ## Plot the clusters
    ########################################################
    #fig, ax = plt.subplots()

    #def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
        #"""
        #Plots an `nstd` sigma error ellipse based on the specified covariance
        #matrix (`cov`). Additional keyword arguments are passed on to the 
        #ellipse patch artist.
    
        #Parameters
        #----------
            #cov : The 2x2 covariance matrix to base the ellipse on
            #pos : The location of the center of the ellipse. Expects a 2-element
                #sequence of [x0, y0].
            #nstd : The radius of the ellipse in numbers of standard deviations.
                #Defaults to 2 standard deviations.
            #ax : The axis that the ellipse will be plotted on. Defaults to the 
                #current axis.
            #Additional keyword arguments are pass on to the ellipse patch.
    
        #Returns
        #-------
            #A matplotlib ellipse artist
        #"""
        #def eigsorted(cov):
            #vals, vecs = np.linalg.eigh(cov)
            #order = vals.argsort()[::-1]
            #return vals[order], vecs[:,order]
    
        #if ax is None:
            #ax = plt.gca()
    
        #vals, vecs = eigsorted(cov)
        #theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
    
        ## Width and height are "full" widths, not radius
        #width, height = 2 * nstd * np.sqrt(vals)
        #ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    
        #ax.add_artist(ellip)
        #return ellip
    
    ##for q in qs:
        ##cov =  q._gaussian.cov[0:2, 0:2]
        ##plot_cov_ellipse(cov, q._gaussian.mu[0:2], nstd=0.5, alpha=0.5)

    #consider = []
    #for rel, data in great_plot.items():
        #if len(data) > 0:
            #consider.append(rel)
        
    
    #colormap = plt.cm.gist_ncar
    #colours = [colormap(i) for i in np.linspace(0, 0.9, len(consider))]
##    plt.rc('axes', color_cycle=['r', 'g', 'b', 'y'])
    #isinstance(ax, plt.Axes)
    #rel = np.array(relatives)
    #ax.set_color_cycle(colours)
    #lbls = []
    #for rel in consider:
        #data = great_plot[rel]
        ##, data in great_plot.items():
        #d = np.array(data)
        #ax.plot(d[:, 0], d[:, 1], '.',  alpha=.5, markersize=5)
        ##for i in range(0, d.shape[0]):
            ##for qual in qs:
                ##if qual(d[i, :]):
                    ##x, y = zip(*[(d[i, 0], d[i, 1]),
                                 ##(qual._gaussian.mu[0],  qual._gaussian.mu[1])])
                    ##line, = ax.plot(x, y, 'go-', markersize=0)
        ##lbls.append(rel)
        
    ##plt.legend(consider, ncol=4, loc='upper center', 
           ##bbox_to_anchor=[0.5, 1.1], 
           ##columnspacing=1.0, labelspacing=0.0,
           ##handletextpad=0.0, handlelength=1.5,
           ##fancybox=True, shadow=True)
    
    #for q in qs:
        #cov =  q._gaussian.cov[0:2, 0:2]
        #plot_cov_ellipse(cov, q._gaussian.mu[0:2], nstd=0.4, alpha=0.5)
    
    #plt.rc('text', usetex=True)
    #fontsize = 20
    #plt.xlabel(r'$X$',  fontsize=20)
    #plt.ylabel(r'$Y$',  fontsize=20)
    #plt.title(r'A test using $x$',  fontsize=20)
    
    #plt.show()

    
    ######################################################
    ######################################################
    
    
     ##Using Lars' Two Point stuff
    #qs =  qualitators.Qualitators()
    #qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("behind", (0, 1, 7)))
    #qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("left", (1, 2, 3)))
    #qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("front", (3, 4, 5)))
    #qs.add_qualitator(qualitators.TwoPointCalcAngleQualitator("right", (5, 6, 7)))
    
    #qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("close", (0, 0.3)))
    #qs.add_qualitator(qualitators.TwoPointCalcDistQualitator("distant", (0.3, 1000)))
    
    #qs.add_qualitator(qualitators.SizeQualitator("wider", 0))
    #qs.add_qualitator(qualitators.SizeQualitator("deeper", 1))
    #qs.add_qualitator(qualitators.SizeQualitator("taller", 2))
    
    #qs.save_to_disk("test.qsrs")
    
    if os.path.exists(options.output_filename) and not options.overwrite:
        print "ERROR: output file already exists; not proceeding. Use -O to overwrite."
        sys.exit(1)
   
    qs.save_to_disk(options.output_filename)