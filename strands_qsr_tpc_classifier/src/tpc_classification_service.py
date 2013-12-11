#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_tpc_classifier')

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *
import rospy
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
        for cls in state:
            val += len(cls)
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
        
        print(current, problem.value(current.state))
        print("Current best state", best_state, best_value)

    
def handle_group_classification(req):

    res = GetGroupClassificationResponse()

    res.group_classification = list()
    
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
        print(obj_cls.object_id, obj_cls.type[max_idx])
        state.append(obj_cls.type[max_idx])

    print("Initial state:", state)
    
    gc = GroupClassification(state, req.type)

    simulated_annealing(gc, exp_schedule(k=20, lam=0.005, limit=10000))
        
    return res

def tpc_classification_server():
    rospy.init_node('tpc_classification_server')
    s = rospy.Service('tpc_group_classification', GetGroupClassification, handle_group_classification)
    rospy.loginfo("Ready to classify groups")
    rospy.spin()

if __name__ == "__main__":
    tpc_classification_server()
