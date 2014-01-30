import pickle

class RelationProbabilities(object):
    def __init__(self):
        self.probabilities = {}
        
    def _check_prob_exists(self, relation, object_pair):
        if relation not in self.probabilities:
            self.probabilities[relation] = {}
        if object_pair not in self.probabilities[relation]:
            self.probabilities[relation][object_pair] = [0.001, 2]#[ 0.0001 , 2]
    
    def get_prob(self, relation, object_pair):
        self._check_prob_exists(relation, object_pair)
        return self.probabilities[relation][object_pair][0]
    
    def update_probs_new_example(self, relation, object_pair, pos_neg):
        self._check_prob_exists(relation, object_pair)
        if pos_neg: # positive
            self.probabilities[relation][object_pair][0] = (self.probabilities[relation][object_pair][0] *
                                                            self.probabilities[relation][object_pair][1] + 1) / (self.probabilities[relation][object_pair][1] + 1)
            self.probabilities[relation][object_pair][1] += 1
        else:
            self.probabilities[relation][object_pair][0] = (self.probabilities[relation][object_pair][0] * self.probabilities[relation][object_pair][1]) / (self.probabilities[relation][object_pair][1] + 1)
            self.probabilities[relation][object_pair][1] += 1
        self.probabilities[relation][object_pair][0]     
        
    def save_to_disk(self, filename):
        with open(filename, "w") as f:
            pickle.dump(self, f)
    
    @classmethod
    def load_from_disk(self, filename):
        with open(filename, "r") as f:
            ob = pickle.load(f)
        return ob
    
if __name__ == '__main__':
    ''' Main Program '''
    rel_probs = RelationProbabilities.load_from_disk("test.probs")
    
    for i in rel_probs.probabilities:
        for j in rel_probs.probabilities[i]:
            #if rel_probs.probabilities[i][j][0] > 0.1:
            print i, j, rel_probs.probabilities[i][j]
        print "-=" * 20