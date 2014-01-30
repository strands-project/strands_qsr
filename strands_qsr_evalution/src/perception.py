import random

class PerceptionProb(object):
    def __init__(self, object_types, values_matrix=None):
        """
        :Parameters:
            values_matrix : string
                a matrix of perception values, p(Z |  X). see bellow for examples.
            object_types : list
                list of object types that the value matrix represents. Column and row
                order must match and match the order of this list.
        """
        print "Creating perception object"
        print object_types
        print values_matrix
        print "*" * 10
        self.object_types = object_types
        self.prob_perceive_given_state = dict([(o,  dict([(o,  1)
                                                          for o in object_types]))
                                               for o in object_types])
        
        #TODO: check values matrix is good size etc
        if values_matrix is not None:
            perceived =  values_matrix.strip().split("\n")
            for perceive, i in zip(object_types, perceived):
                vals = i.split("\t")
                for state, v in zip(object_types, vals):
                    self.prob_perceive_given_state[perceive][state] = float(v)
                    
    def prob(self, perception, actual):
        """
		Return p(Z=val | X=val)
        
        :Parameters:
            perception : string
                the perceived class
            actual : string
                the actual class
        """
        if perception not in self.object_types:
            raise Exception("Unknown object type for perception in model")
        if actual not in self.object_types:
            raise Exception("Unknown object type for actual object in perception model")
        return self.prob_perceive_given_state[perception][actual]

    def create_perception(self, state):
        """
        Create a simulated perception for the given state.
        
        :Parameters:
            state : dictionary
                dictionary of object name to actual object type
        
        Return: dictionary, object name to list of confidence values ordered as in self.object_types
        
        """
        percept = {}
        for name, ob in state.items():
            i = random.randint(1, 100) / 100.0
            s = 0
            for perc in self.object_types:
                s += self.prob_perceive_given_state[perc][ob]
                if s >= i - 0.1e-10 : # account for rounding errors
                    break
            else:
                raise Exception("Overshot probs, matrix was likely incomplete. Columns "
                                "need to sum to 1")
            # labeling it as perc
            # assume p(Z|X)=p(X|Z) - ie the matrix is symetric and use
            # as the confidence values
            # TODO: think this
            percept[name] = [self.prob(obj, perc) for obj in self.object_types]
        return percept
    
    @classmethod
    def create_simple(cls, objects,  prob):
        """
        Create an uncorrelated perception model. That is the p(Z=cup | X=cup)=prob,
        p(Z=NOTcup | X=cup)=(1-prob)/(no. objects - 1)
        :Parameters:
            objects : list
                the objects to create a perception model for
            prob : float
                the probability that an object is labeled correctly
        """
        p =  cls(objects)
        assert prob <= 1.0
        for o in objects:
            for o2 in objects:
                if o == o2:
                    p.prob_perceive_given_state[o][o2] = prob
                else:
                    p.prob_perceive_given_state[o][o2] = (1.0 - prob) / (len(objects) - 1)
        isinstance(p, PerceptionProb)
        return p
    
    @staticmethod
    def is_valid_model(model_name):
        """
        Checks if the model is a known perception model
		
        :Parameters:
            model_name : string
                model name to check existence of
        """
        valid_names = [i for i, v in globals().items()
                       if type(v) == list and len(v) == 2 and type(v) == list]
        return model_name in valid_names
    
    @staticmethod
    def get_library_model_names():
        """
        Return a list of known model names
		"""
        return [i for i, v in globals().items()
                       if type(v) == list and len(v) == 2 and type(v) == list]
        
    @classmethod
    def create_from_library(cls, entry):
        """
        Create a perception model from the matrix already present in this file,
        with variable name "entry"
        
        :Parameters:
            entry : string
            	the matrix name to use
        """
        # TODO: more complete name checking
        valid_names = [i for i, v in globals().items()
                       if type(v) == list and len(v) == 2 and type(v) == list]
        if not entry in valid_names:
            raise Exception("Trying to create unknown perception model")
        return cls(globals()[entry][0], globals()[entry][1])
        


# Below are some perception matrices, see perception.gnumeric
simple_42 = [
    ["Monitor", "Cup", "Keyboard", "Pencil", "Telephone", "PC", "Mouse", "Lamp",
     "Calculator", "Headphone", "Laptop", "MobilePhone", "Glass", "Stapler", "Keys", "Book", "Bottle"], 
"""
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
]
