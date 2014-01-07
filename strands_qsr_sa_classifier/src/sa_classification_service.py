#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_sa_classifier')

import rospy

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *

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

    def __init__(self, initial, positions, types):
        super(GroupClassification, self).__init__(initial,types)
        self.positions = positions

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

        for i in range(len(state)):
            for j in range(len(state)):
                if i != j:
                    val += 1
        return val

            
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
        delta_e = problem.value(next.state) - problem.value(current.state)

            
        if delta_e > 0 or probability(math.exp(delta_e/T)):
            current = next

            if problem.value(next.state) > best_value:
                best_state = copy.deepcopy(current)
                best_value = problem.value(next.state)
        
        #print(current, problem.value(current.state))
        #print("Current best state", best_state, best_value)

class SAClassifier():

    def __init__(self):
        rospy.init_node('sa_classification_server')
        rospy.loginfo("Started SA classification service")

        self.service = rospy.Service('group_classification', GetGroupClassification, self.handle_group_classification)

        service_name = 'get_qsr_description'
        rospy.wait_for_service(service_name)
        self.qsr_desc = rospy.ServiceProxy(service_name, GetQSRDescription)
        
        rospy.loginfo("Ready to classify groups")
        rospy.spin()
        rospy.loginfo("Stopped SA classification  service")
    
    def handle_group_classification(self,req):

        rospy.loginfo("Classifying group...")

        # Get QSR description for scene using service
        qsr_req = GetQSRDescriptionRequest()
        qsr_req.object_id = req.object_id
        qsr_req.pose = req.pose
        qsr_req.bbox = req.bbox
        qsr_res = self.qsr_desc(qsr_req)
        print(json.loads(qsr_res.predicates))

        # Create response
        res = GetGroupClassificationResponse()

        res.group_classification = list()
    
        # Fake perception as there is none
        # - uniform
        # - according to object occurrences
        # - ground truth as input
        for obj in req.object_id:
        
            obj_classification = ObjectClassification()

            obj_classification.object_id = obj
            obj_classification.type  = list()
            obj_classification.confidence  = list()
            
            for t in req.type:
                obj_classification.type.append(t)
                obj_classification.confidence.append(random.uniform(0,1))

            res.group_classification.append(obj_classification)
        
            
        state = []
        for obj_cls in res.group_classification:
            max_idx = obj_cls.confidence.index(max(obj_cls.confidence))
            #print(obj_cls.object_id, obj_cls.type[max_idx])
            state.append(obj_cls.type[max_idx])

        positions = []
        for pos in req.pose:
            positions.append([pos.position.x, pos.position.y])
    
        
        gc = GroupClassification(state, positions, req.type)
    
        best_state = simulated_annealing(gc, exp_schedule(k=20, lam=0.005, limit=10000))

        print ""
        print "Initial state:", state, "(", gc.value(state), ")"
        print "Best state:", best_state.state, "(", gc.value(best_state.state), ")"
        print "Positions:", positions
        print ""

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
    sac = SAClassifier()
    
