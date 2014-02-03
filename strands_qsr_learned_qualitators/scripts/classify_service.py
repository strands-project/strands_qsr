#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_learned_qualitators')
import rospy
from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *

import strands_qsr_learned_qualitators.geometric_state as gs
import strands_qsr_learned_qualitators.density as ds
from strands_qsr_learned_qualitators import qualitators, scene
import json
import numpy as np
import random
import copy
import math

from optparse import OptionParser
import os
import sys


#from pyevolve import G1DList
#from pyevolve import GSimpleGA

class LearnedQualitatorGroupClassifier(object):
    def __init__(self, qsrs_file, scenes_file):
        rospy.loginfo("Loading qualitators")
        self.qual = qualitators.Qualitators.load_from_disk(qsrs_file)
        rospy.loginfo ("Loaded " + str(len(self.qual._qualitators)) + "qualitators from disk")
        
        rospy.loginfo("Loading scene probabilities")
        self.prob =  scene.RelationProbabilities.load_from_disk(scenes_file)

        self.srv_handle = rospy.Service('group_classification', GetGroupClassification, self.handle_request)
        
        rospy.loginfo("Ready to classify groups")
        self.number_done = 0
    
    def handle_request(self, req):
        res = GetGroupClassificationResponse()

        geo =  gs.GeometricState("testscene")
        for object_id, pose, bbox in zip(req.object_id, req.pose, req.bbox):
            position = [pose.position.x, pose.position.y, pose.position.z]
            box = []
            for p in bbox.point:
                box.append([p.x,p.y,p.z])
            bb = gs.BBoxArray(box)
            geo.add_object(object_id, "PC", position, None, box) # PC as it is valid, but unused here anywya

        state =  {}
        state_probs = {}
        for obj_class in req.group_classification:
            confidences = zip(obj_class.type, obj_class.confidence)
            random.shuffle(confidences)
            state[obj_class.object_id] = max(confidences, key=lambda f:f[1])[0]
            norm =  sum(obj_class.confidence)
            state_probs[obj_class.object_id] = {}
            for t, conf in zip(obj_class.type, obj_class.confidence):
                state_probs[obj_class.object_id][t] = conf / norm
                
        s = self.qual.create_qualitative_state(geo)
        #print "Number relations: ", len(s)
            
        #print "!-!-" * 20
        #print "Evaluating scene [", self.number_done, "]"
        #print "----" * 20
        #print "State:", state
        self.number_done += 1

        ## GA Approach
        #if len(s) > 1:
            #objects = state.keys()
            #types_available = req.type
            #def eval_chromo(state):
                #p = 1
                #for obj_name, i in zip(objects, state):
                    #p *= state_probs[obj_name][types_available[i]]
                #if p == 0:
                    #return math.log(1e-300) 
                #score =  math.log(p)
    
                #for rel in s:
                    #p_z_x = self.prob.get_prob(rel[0], (types_available[state[ objects.index(rel[1])]],
                                                        #types_available[state[ objects.index(rel[2])]]))
                    #score += math.log(p_z_x)
                #return 10000 + score
            
            #genome = G1DList.G1DList(len(objects))
            #genome.setParams(rangemin=0, rangemax=len(types_available)-1)
            #genome.evaluator.set(eval_chromo)
            #ga = GSimpleGA.GSimpleGA(genome)
            #ga.setGenerations(500)
            #ga.setMutationRate(0.01)
            #ga.evolve(freq_stats=10)
            #for obj_name, i in zip(objects, ga.bestIndividual()):
                #state[obj_name] = types_available[i]
                

        def evaluate_state(state):
            p = 1
            for obj_name, obj_class in state.items():
                p *= state_probs[obj_name][obj_class]
                #p = math.log(state_probs[obj_name][obj_class])
            if p == 0:
                return math.log(1e-300) 
            score =  math.log(p)

            #print "log(score) = ", score, 
            for rel in s:

                #print "Mult by",rel[0], (state[rel[1]], state[rel[2]]),  ":", self.prob.get_prob(rel[0], (state[rel[1]], state[rel[2]]) )
                consts = [state[rl] for rl in rel[1:]]
                p_z_x = self.prob.get_prob(rel[0], tuple(consts))
                                           
                #p_z_x = self.prob.get_prob(rel[0], (state[rel[1]], state[rel[2]]) ) 

                #print " + ", math.log(p_z_x), 
                #score *= p_z_x
                score += math.log(p_z_x)
                
            return score
       
        ######
        best_delta = 1
        while best_delta > 1e-14:
            score =  evaluate_state(state)

            best = []
            best_delta = 0
            for ob in state:
                for chg in req.type:
                    if chg == state[ob]:
                        continue
                    #ss = copy.copy(state)
                    tmp = state[ob]
                    state[ob] = chg
                    score_delta = evaluate_state(state)
                    state[ob] = tmp
                    
                    delta = score_delta - score
                    if delta > best_delta:
                        best_delta = delta
                        best = [ob, chg]
            #print best_delta, best
            if best_delta > 1e-14:
                #print best_delta
                state[best[0]] = best[1]

        #changes = 0
        #outer_changes = 1
        ##for i in range(5):
        #while outer_changes != 0:
            #outer_changes = 0
            #for ob in state:
                #best_chg_type = ""
                #best_chg_score = -1e10000
                #now_score = evaluate_state(state)
                #for chg in req.type: # relevant object types
                    #if chg == state[ob]:
                        #continue
                    #ss = copy.copy(state)
                    #ss[ob] = chg
                    #score = evaluate_state(ss)
                    
                    #if score > best_chg_score and score > now_score:
                        #best_chg_score = score
                        #best_chg_type = chg
            #if best_chg_score > now_score: #-1e10000:
                #state[ob] = best_chg_type
                #changes += 1
                #outer_changes += 1
            #print "score=", evaluate_state(state), "changes=", outer_changes

        #if changes < 1:
            #rospy.loginfo("Changes: " + str(changes))
            
                
        #rospy.loginfo("Found state:" + str(state))
        #rospy.loginfo("Ev count {}".format(ev_count))
        
        for obj_name, obj_class in state.items():
            obj_classification = ObjectClassification()
            obj_classification.object_id = obj_name
            obj_classification.type = [obj_class]
            obj_classification.confidence = [1.0]
            

            res.group_classification.append(obj_classification)
        
        return res


if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("-s", "--scene-prob",
                      dest="probs_filename", metavar="FILE", 
                      help="load the scene probabilities from FILE")
    parser.add_option("-q", "--qualitators",
                      dest="qualitators_filename", metavar="FILE", 
                      help="load the qualitators from FILE")
    
    (options, args) = parser.parse_args()
    
    if not options.probs_filename:
        parser.error("scene probabilities file name is required")        
    if not options.qualitators_filename:
        parser.error("qualitators file name is required")        
    
    if not os.path.isfile(options.probs_filename):
        print ("ERROR: scene probabilities file '" + 
               options.probs_filename +  "' does not exist")
        sys.exit(1)
      
    if not os.path.isfile(options.qualitators_filename):
        print "ERROR: qualitators file does not exist"
        sys.exit(1)
      
      
    rospy.init_node('learned_classification_server')
    service =  LearnedQualitatorGroupClassifier(options.qualitators_filename,
                                                options.probs_filename)
    rospy.spin()


