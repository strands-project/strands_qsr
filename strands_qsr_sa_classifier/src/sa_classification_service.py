#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_sa_classifier')

import rospy

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *

from geometry_msgs.msg import *

import getopt
import matplotlib
import json

#from matplotlib import cm
#import matplotlib.patches as mpatches
from matplotlib.mlab import griddata
#from mpl_toolkits.mplot3d import Axes3D
#import matplotlib.pyplot as plt
#from matplotlib.patches import Circle


import random
import math
import copy

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def help_msg():
    return """
  Usage: sa_classification_serive.py [-h] <training_set> 

    training_set       training set of scenes

    -h, --help for seeing this msg
"""



class Problem(object):

    def __init__(self, initial, types):
        self.initial = initial
        self.types = types
        
    def actions(self, state):
        abstract

    def result(self, state, action):
        abstract

    def value(self, state):
        abstract
        
class Node:

    def __init__(self, state):
        self.state = state

    def __repr__(self):
        return "<Node %s>" % (self.state)

    def expand(self, problem):
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        next = problem.result(self.state, action)
        return Node(next)


class GroupClassification(Problem):

    def __init__(self, initial, init_copy, positions, types, object_id, size, dist, direc, sac):
        super(GroupClassification, self).__init__(initial,types)
        self.positions = positions
        self.sac = sac
        self.size = size
        self.dist = dist
        self.direc = direc
        self.object_id = object_id
        self.init_copy = copy.deepcopy(init_copy)


    def actions(self,state):
        return ["replace"]

    def result(self,state,action):
        if action == "replace":
            idx = random.randint(0,len(state)-1)
            new_class = state[idx]
            while new_class == state[idx]:
                type_idx = random.randint(0,len(self.types)-1)
                new_class = self.types[type_idx]

            state[idx] = new_class

            return state


    def value(self, state):

        val = 0

        same_as_perception = 0 
        
        for i in range(len(state)):

            if state[i] == self.init_copy[i]:
                
                same_as_perception += 1
            
            key = [self.size[i], state[i]]
            val += (float(self.sac.get_count(self.sac.count_size,tuple(key))) / \
                        ( self.sac.get_count(self.sac.count_size_objs, tuple(state[i])) + 1)) + \
                        (float(self.sac.get_count(self.sac.count_size,tuple(key))) \
                             /  ( self.sac.get_count(self.sac.count_size_rels, self.size[i]) + 1)) 

            for j in range(len(state)):

                if i != j:                    

                    key_direc = [self.direc[tuple([self.object_id[i], self.object_id[j]])], state[i], state[j]]
                    key_direc_objs = [state[i], state[j]]
                    val += (float(self.sac.get_count(self.sac.count_dir,tuple(key_direc))) / \
                                (self.sac.get_count(self.sac.count_dir_objs, tuple(key_direc_objs)) + 1)) + \
                                (float(self.sac.get_count(self.sac.count_dir,tuple(key_direc))) / \
                                     (self.sac.get_count(self.sac.count_dir_rels, \
                                                             self.direc[tuple([self.object_id[i], self.object_id[j]])]) + 1))

                    key_dist = [self.dist[tuple([self.object_id[i], self.object_id[j]])], state[i], state[j]]
                    key_dist_objs = [state[i], state[j]]
                    val += (float(self.sac.get_count(self.sac.count_dist,tuple(key_dist))) / \
                                ( self.sac.get_count(self.sac.count_dist_objs, tuple(key_dist_objs)) + 1)) + \
                                (float(self.sac.get_count(self.sac.count_dist,tuple(key_dist))) / \
                                ( self.sac.get_count(self.sac.count_dist_rels, \
                                                         self.dist[tuple([self.object_id[i], self.object_id[j]])] ) + 1))


        perception_weight = float(same_as_perception) / len(state)

        #print "**************", perception_weight
        
        return math.exp(perception_weight) * float(val)

            
def probability(p):
    "Return true with probability p."
    return p > random.uniform(0.0, 1.0)

def if_(test, result, alternative):
    if test:
        if callable(result): return result()
        return result
    else:
        if callable(alternative): return alternative()
        return alternative

def exp_schedule(k=20, lam=0.005, limit=100):
    return lambda t: if_(t < limit, k * math.exp(-lam * t), 0)

def simulated_annealing(problem, schedule=exp_schedule()):

    current = Node(problem.initial)
    current_val = problem.value(current.state)
    
    # keep track of the best state
    best_state = copy.deepcopy(current)
    best_value = -sys.maxint - 1

    for t in xrange(sys.maxint):
        T = schedule(t)
        if T == 0:
            #return current
            return best_state
        neighbors = current.expand(problem)
        if not neighbors:
            #return current
            return best_state
        
        next = random.choice(neighbors)

        next_val = problem.value(next.state)
        delta_e = next_val - current_val

            
        if delta_e > 0 or probability(math.exp(delta_e/T)):
            current = next
            current_val = next_val
            
            if next_val > best_value:
                best_state = copy.deepcopy(current)
                best_value = next_val 
        
        #print(current, problem.value(current.state))
        #print("Current best state", best_state, best_value)

class SAClassifier():

    def __init__(self, scenes):

        # cache of probabilities
        self.count_dist = dict()
        self.count_dist_objs = dict()
        self.count_dist_rels = dict()

        self.count_size = dict()
        self.count_size_objs = dict()
        self.count_size_rels = dict()

        self.count_dir = dict()
        self.count_dir_objs = dict()
        self.count_dir_rels = dict()

        self.scenes = scenes

        rospy.init_node('sa_classification_server')
        rospy.loginfo("Started SA classification service")

        self.service = rospy.Service('group_classification', GetGroupClassification, self.handle_group_classification)

        service_name = 'get_qsr_description'
        rospy.wait_for_service(service_name)
        self.qsr_desc = rospy.ServiceProxy(service_name, GetQSRDescription)

        self.gen_stats()
        
        rospy.loginfo("Ready to classify groups")
        rospy.spin()
        rospy.loginfo("Stopped SA classification  service")

    def inc_count(self, dic, key):

        if key not in dic:
            dic[key]  = 1
        else:
            dic[key] += 1

    def get_count(self, dic, key):

        if key not in dic:
            return 0
        else:
            return dic[key]
        
    def gen_stats(self):

        for s in self.scenes:
            qsr_req = GetQSRDescriptionRequest()

            for obj in s['objects']:
                qsr_req.object_id.append(obj)
                
                pos = s['position'][obj]
                pose = Pose()
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]
                qsr_req.pose.append(pose)
                
                bbox = s['bbox'][obj]

                bbox_t = BBox()
                for point in bbox:
                    p = Point32()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = point[2]
                    bbox_t.point.append(p)
                qsr_req.bbox.append(bbox_t)

            qsr_res = self.qsr_desc(qsr_req)
            predicates = json.loads(qsr_res.predicates)

            for p in predicates:
                # replace instance names with types

                if p[1] in ['small', 'medium', 'large']:

                    types = list()
                    rels = list()
                    rels.append(p[1])
                    for i in range(2,len(p)):
                        rels.append(s['type'][p[i]])
                        types.append(s['type'][p[i]])

                    self.inc_count(self.count_size, tuple(rels))
                    self.inc_count(self.count_size_objs, tuple(types))
                    self.inc_count(self.count_size_rels, p[1])

                    
                elif p[1] in ['very_close', 'close', 'distant']:

                    types = list()
                    rels = list()
                    rels.append(p[1])
                    for i in range(2,len(p)):
                        rels.append(s['type'][p[i]])
                        types.append(s['type'][p[i]])

                    self.inc_count(self.count_dist, tuple(rels))
                    self.inc_count(self.count_dist_objs, tuple(types))
                    self.inc_count(self.count_dist_rels, p[1])

                else:
                    types = list()
                    rels = list()
                    rels.append(p[1])
                
                    for i in range(2,len(p)):
                        rels.append(s['type'][p[i]])
                        types.append(s['type'][p[i]])

                    self.inc_count(self.count_dir, tuple(rels))
                    self.inc_count(self.count_dir_objs, tuple(types))
                    self.inc_count(self.count_dir_rels, p[1])

#        print self.count_size
#        print self.count_size_objs
#        print self.count_dist
#        print self.count_dist_objs
#        print self.count_dir
#        print self.count_dir_objs
                
            
            

    def handle_group_classification(self,req):

        rospy.loginfo("Classifying group...")

        # Get QSR description for scene using service
        qsr_req = GetQSRDescriptionRequest()
        qsr_req.object_id = req.object_id
        qsr_req.pose = req.pose
        qsr_req.bbox = req.bbox
        qsr_res = self.qsr_desc(qsr_req)
        predicates = json.loads(qsr_res.predicates)


        objs =list ()
        size = list()
        dist = dict()
        direc = dict() 
        for obj in req.object_id:
            for p in predicates:
                if p[1] in ['small','medium','large'] and obj == p[2]:
                    size.append(p[1])
                    objs.append(p[2])
                elif p[1] in ['very_close','close','distant'] and obj == p[2]:
                    dist[tuple([p[2],p[3]])] = p[1] 
                elif p[1] not in ['small','medium','large'] and p[1] not in ['very_close', 'close','distant']: 
                    direc[tuple([p[2],p[3]])] = p[1]

                    
        # Generate initial state
        state = []
        for obj_cls in req.group_classification:
            max_idx = obj_cls.confidence.index(max(obj_cls.confidence))
            #print(obj_cls.object_id, obj_cls.type[max_idx])
            state.append(obj_cls.type[max_idx])

        positions = []
        for pos in req.pose:
            positions.append([pos.position.x, pos.position.y])


        best_state = None
        current_state = None
        
        for i in range(1):
            
            gc = GroupClassification(state, state,  positions, req.type, objs, size, dist, direc, self)
    
            current_state = simulated_annealing(gc, exp_schedule(k=20, lam=0.005, limit=100))
            if best_state == None:
                best_state = copy.deepcopy(current_state)
            else:
                if gc.value(current_state.state) > gc.value(best_state.state):
                    print "************* FOUND BETTER STATE **********************", gc.value(current_state.state),  gc.value(best_state.state)
                    best_state = copy.deepcopy(current_state)

        
        print ""
        print "Initial state:", state, "(", gc.value(state), ")"

        print "Best state:", best_state.state, "(", gc.value(best_state.state), ")"
        #print "Positions:", positions
        print ""


        # Create response
        res = GetGroupClassificationResponse()
        res.group_classification = list()
        for obj in req.object_id:
        
            obj_classification = ObjectClassification()

            obj_classification.object_id = obj
            obj_classification.type  = list()
            obj_classification.confidence  = list()
            obj_classification.type.append(best_state.state[req.object_id.index(obj)])
            obj_classification.confidence.append(1.0)

            res.group_classification.append(obj_classification)
        
        rospy.loginfo("Waiting for next request...")

        return res

    

if __name__ == "__main__":

    argv = None
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "h", ["help"])
        except getopt.error as msg:
            raise Usage(msg)

        if ('-h','') in opts or ('--help', '') in opts or len(args) is not 1:
            raise Usage(help_msg())
        
        with open(args[0]) as in_file:
            scenes = json.load(in_file)
            sac = SAClassifier(scenes)

    except Usage as err:
        print(err.msg)
        print("for help use --help")
