#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_tpc_classifier')

import rospy

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *
from qsr_msgs.msg import *
from qsr_msgs.srv import *

import matplotlib

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

    def __init__(self, initial, positions, types, gmm_cache):
        super(GroupClassification, self).__init__(initial,types)
        self.positions = positions
        self.gmm_cache = gmm_cache
        service_name = 'qsr_to_gmm'
        rospy.wait_for_service(service_name)
        self.qsr_to_gmm = rospy.ServiceProxy(service_name, QSRToGMM)

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
                    rel_obj_pos = [o - l for o, l in zip(self.positions[i],self.positions[j])]
                    val += self.gmm_value(state[i],state[j],rel_obj_pos)
        return val

#    def value(self, state):
#        val = 0 
#        for cls in state:
#            val += len(cls)
#        return val

    def gmm_value(self, obj, landmark, rel_obj_position):

        # Please note: this is a simple workaround, because currently the QSR for a
        # objects of the same type is not supported
        if obj == landmark:
            return 0.0
        
        if (obj,landmark) not in self.gmm_cache:
            # get gmm from qsr_to_gmm service
            try:
                req = QSRToGMMRequest()
                req.object = obj
                req.landmark = landmark
                response = self.qsr_to_gmm(req)
                gmm = dict()
                gmm['weight'] = response.weight
                gmm['gaussian'] = response.gaussian
                gmm['probability'] = response.probability
                self.gmm_cache[(obj,landmark)] = gmm
            except rospy.ServiceException, e:
                gmm = dict()
                gmm['weight'] = []
                gmm['gaussian'] = []
                gmm['probability'] = response.probability
                self.gmm_cache[(obj,landmark)] = gmm
                print "Service call failed: %s"%e
                
        return self.compute_func_val(rel_obj_position, self.gmm_cache[(obj,landmark)])

    def compute_func_val(self, pos, gmm):
        #
        gmm_val = 0.0
        for j in range(len(gmm['weight'])):
            cov = gmm['gaussian'][j].covariance
            gmm_val +=  gmm['weight'][j] * matplotlib.mlab.bivariate_normal(pos[0],
                                                                            pos[1],
                                                                            math.sqrt(cov[0]),
                                                                            math.sqrt(cov[3]),
                                                                            gmm['gaussian'][j].mean[0],
                                                                            gmm['gaussian'][j].mean[1],
                                                                            cov[1])
        return gmm['probability'] * gmm_val


            
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

class TPCClassifier():

    def __init__(self):
        rospy.init_node('tpc_classification_server')
        rospy.loginfo("Started TPC classification service")

        self.gmm_cache = dict()

        self.service = rospy.Service('tpc_group_classification', GetGroupClassification, self.handle_group_classification)
        rospy.loginfo("Ready to classify groups")
        rospy.spin()
        rospy.loginfo("Stopped TPC classification  service")
    
    def handle_group_classification(self,req):

        rospy.loginfo("Classifying group...")

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
            #print(obj_cls.object_id, obj_cls.type[max_idx])
            state.append(obj_cls.type[max_idx])

        positions = []
        for pos in req.pose:
            positions.append([pos.position.x, pos.position.y])


        gc = GroupClassification(state, positions, req.type, self.gmm_cache)
    
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


        
        # obj = random.choice(["Cup", "Keyboard"])
        # x   = random.uniform(-1.0,1.0)
        # y   = random.uniform(-1.0,1.0)
        # print obj, x, y, "=> GMM value:", self.value(obj,"Monitor",[x,y])

        # cache the GMMs for the next service call
        self.gmm_cache = gc.gmm_cache
        
        rospy.loginfo("Waiting for next request...")

        return res

    

if __name__ == "__main__":
    tpcc = TPCClassifier()
    
