#!/usr/bin/python


import optparse
import strands_qsr_learned_qualitators.geometric_state as gs
import strands_qsr_learned_qualitators.density as ds
from strands_qsr_learned_qualitators import qualitators, scene
import json
import numpy as np
import random
import copy

class PerceptionProb(object):
    def __init__(self):
        self.prob_perceive_given_state = dict([(o,  dict([(o,  1) for o in gs.object_types]))
                                               for o in gs.object_types])

    def create_perception(self, state):
        # state is a dictionary of objects : name:type
        # returned is a dictionary of name: perceived type
        percept = {}
        for name, ob in state.items():
            i = random.randint(1, 100) / 100.0
            s = 0
            for perc in gs.object_types:
                s += self.prob_perceive_given_state[perc][ob]
                if s >= i - 0.1e-10 :
                    break
            else:
                print  s, ">=", i, ":", s >= i
                raise Exception("Overshot probs")
            percept[name] = perc
        return percept
    
full_percept_prob = PerceptionProb()
# from gunermic file
values = """

0.65	0.01	0.05	0.01	0.01	0.1	0.01	0.01	0.01	0.03	0.05	0.01	0.01	0.01	0.01	0.01	0.01
0.01	0.38	0.01	0.01	0.03	0.01	0.06	0.02	0.02	0.02	0.01	0.02	0.2	0.02	0.03	0.01	0.14
0.05	0.01	0.398	0.01	0.02	0.04	0.012	0	0.04	0.04	0.2	0.02	0.01	0.03	0.01	0.1	0.01
0.01	0.01	0.01	0.64	0.01	0.01	0.01	0	0.03	0.01	0.01	0.1	0.01	0.1	0.01	0.02	0.01
0.01	0.03	0.02	0.01	0.37	0.05	0.01	0.2	0.05	0.05	0.04	0.04	0.01	0.01	0.01	0.08	0.01
0.1	0.01	0.04	0.01	0.05	0.7	0.01	0.01	0	0	0.01	0.01	0.01	0.01	0.01	0.01	0.01
0.01	0.06	0.012	0.01	0.01	0.01	0.628	0.02	0.03	0.03	0.01	0.04	0.03	0.03	0.03	0.02	0.02
0.01	0.02	0	0	0.2	0.01	0.02	0.67	0.02	0.01	0.01	0	0.01	0	0.01	0	0.01
0.01	0.02	0.04	0.03	0.05	0	0.03	0.02	0.7	0.01	0.005	0.05	0.01	0.01	0.004	0.01	0.001
0.03	0.02	0.04	0.01	0.05	0	0.03	0.01	0.01	0.781	0.001	0.001	0.005	0.005	0.005	0.001	0.001
0.05	0.01	0.2	0.01	0.04	0.01	0.01	0.01	0.005	0.001	0.5	0.05	0.001	0.001	0.001	0.1	0.001
0.01	0.02	0.02	0.1	0.04	0.01	0.04	0	0.05	0.001	0.05	0.609	0.01	0.01	0.01	0.01	0.01
0.01	0.2	0.01	0.01	0.01	0.01	0.03	0.01	0.01	0.005	0.001	0.01	0.654	0	0.01	0.01	0.01
0.01	0.02	0.03	0.1	0.01	0.01	0.03	0	0.01	0.005	0.001	0.01	0	0.734	0.01	0.01	0.01
0.01	0.03	0.01	0.01	0.01	0.01	0.03	0.01	0.004	0.005	0.001	0.01	0.01	0.01	0.8	0.02	0.02
0.01	0.01	0.1	0.02	0.08	0.01	0.02	0	0.01	0.001	0.1	0.01	0.01	0.01	0.02	0.579	0.01
0.01	0.14	0.01	0.01	0.01	0.01	0.02	0.01	0.001	0.001	0.001	0.01	0.01	0.01	0.02	0.01	0.717
"""
perceived =  values.strip().split("\n")
for perceive, i in zip(gs.object_types, perceived):
    vals = i.split("\t")
    for state, v in zip(gs.object_types, vals):
        full_percept_prob.prob_perceive_given_state[perceive][state] = float(v)

if __name__ == '__main__':
    ''' Main Program '''
    print "Loading data file"
    with open("data/simulation/bham_office_desk_500.json", "r") as f:
    #with open("data/simulation/bham_2000.json", "r") as f:
        scenes = json.load(f)
    geo_states = [gs.GeometricState.from_scene_data(s) for s in scenes]

    print "Loading qualitators"
    qual = qualitators.Qualitators.load_from_disk("test.qsrs")
    print "Loaded ", len(qual._qualitators), "qualitators from disk"

    print "Loading scene probabilities"
    prob =  scene.RelationProbabilities.load_from_disk("test.probs")

    print "Going through scenes."
    good = bad = 0
    pgood = pbad = 0
    for geo in geo_states[:100]:
        s =  qual.create_qualitative_state(geo)

        true_state =  dict([(ob, geo.get_object(ob).obj_type) for ob in geo.get_object_names()])
        
        state =  full_percept_prob.create_perception(true_state) # dict([(ob, random.choice(gs.object_types)) for ob in geo.get_object_names()])
        percept = copy.copy(state)
        
        #state = copy.copy(true_state )
        
        state_score = pow(1.0 / len(gs.object_types), geo.number_of_objects())
        
        #print "Initial Guess: (score=", state_score,")", state
        #print "True State: ", true_state
        
        pgood += sum([1 if state[i]==true_state[i] else 0 for i in state.keys()])
        pbad += sum([1 if state[i]!=true_state[i] else 0 for i in state.keys()])


        def evaluate_state(state):
###            #prob is 0.7 if it is actually good, 0.3/no-1 if not
            #p = [0.7 if state[a] == true_state[a] else (0.3 / (len(gs.object_types) - 1))
                 #for a in state.keys()]
            p = 1
            for (i, c) in state.items():
                p *= full_percept_prob.prob_perceive_given_state[percept[i]][c]
            score = p
            # p = [full_percept_prob.prob_perceive_given_state[s][p] for s in state.keys()]
            # p_score = reduce(lambda x, y:x*y, p)
            # score = p_score
            
            # score = 1 #pow(1.0 / len(gs.object_types), geo.number_of_objects())
            
            for rel in s:
                # rel ob ob
                #print "Mult by ", rel[0], (state[rel[1]], state[rel[2]]), prob.get_prob(rel[0], (state[rel[1]], state[rel[2]]) )
                score *= prob.get_prob(rel[0], (state[rel[1]], state[rel[2]]) )
                #score *= p_score
            if score == 0:
                raise Exception("This is zero, oh shit")
            return score

        changes = 0
        for i in range(5):
            for ob in state:
                best_chg_type = ""
                best_chg_score = 0
                for chg in gs.object_types:
                    ss = copy.copy(state)
                    ss[ob] = chg
                    score = evaluate_state(ss)
                    if score > best_chg_score:
                        best_chg_score = score
                        best_chg_type = chg
                if best_chg_score > 0 :
                    state[ob] = best_chg_type
                    changes += 1

        if changes < 1:
            print "Changes: ", changes
                #print best_chg_score
                
            
        #no_good = 0
        #while no_good < 100:
            #score =  evaluate_state(state)
            ## random change an entry
            #p_s =  copy.copy(state)
            #change = random.choice(state.keys())
            #state[change] = random.choice(gs.object_types)
            #n_score = evaluate_state(state)
            #print no_good, ":=>", n_score
            #if n_score <  score:
                #state =  p_s
                #no_good += 1
            #else:
                #no_good = 0
                
        print "Found state:", state
        #print "True score = ", evaluate_state(true_state)
        
        good += sum([1 if state[i]==true_state[i] else 0 for i in state.keys()])
        bad += sum([1 if state[i]!=true_state[i] else 0 for i in state.keys()])
#        print "Guess score = ", evaluate_state(state)
        
        #raw_input(">")
    print good, bad
    print "Score: ", 100.0 * (good / float(good + bad)),  "%"
    print "Raw percept score: ", 100.0 * (pgood / float(pgood + pbad)),  "%"
        
