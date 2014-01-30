import pickle
import time
from symbolic_state import SymbolicState

def rebase(val, syms):
    base = len (syms)
    n = ""
    while val > 0:
        rem = val % base
        val -= rem
        val /= base
        n = syms[rem] + n
    return n

class Qualitator(object):
    def __init__(self,  name, dimensions):
        self.name = name
        self.dimensions = dimensions
    def __call__(self, geometry):
        raise Exception("Trying to use base class qualitator.")
    def to_str(self, *params):
        s = "("+self.name 
        for i in params:
            s = s + " " + i
        s += ")"
        return s
    def __str__(self):
        return self.name + " : " + self.dimensions
    def __hash__(self):
        return hash(self.name)
    
    
class Qualitators(object):
    def __init__(self, name=None, meta=None):
        if name is not None:
            self._name = name
        else:
            self._name = rebase(int(time.time() * 1000000), 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ')
        self._time = time.time()
        self._qualitators = set()
        self._meta = meta
    def add_qualitator(self, qualitator):
        assert isinstance(qualitator, Qualitator)
        self._qualitators.add(qualitator)
    def create_qualitative_state(self, geometric_state):
        ss =  SymbolicState(geometric_state._scene_id)
        for ob1 in geometric_state._objects.keys():
            for ob2 in geometric_state._objects.keys():
                if ob1 == ob2:
                    continue
                delta = [a - b for a, b in zip(geometric_state._objects[ob1].position,
                                               geometric_state._objects[ob2].position)]
                s1 = geometric_state._objects[ob1].bbox.get_size()
                s2 = geometric_state._objects[ob2].bbox.get_size()
                for q in self._qualitators:
                    if q.dimensions != 2:
                        continue
                    if q(delta+list(s1)+list(s2)):
                        ss.add_clause([q.name, ob1, ob2])
        for ob1 in geometric_state._objects.keys():
            for q in self._qualitators:
                if q.dimensions != 1:
                    continue
                s1 = geometric_state._objects[ob1].bbox.get_size()
                if q(geometric_state._objects[ob1].position + list(s1)):
                    ss.add_clause([q.name, ob1])
        return ss

    def save_to_disk(self, filename):
        with open(filename, "w") as f:
            pickle.dump(self, f)
            
    @classmethod
    def load_from_disk(self, filename):
        with open(filename, "r") as f:
            return pickle.load(f)
        
    def __iter__(self):
        return iter(self._qualitators)
    
    def __len__(self):
        return len(self._qualitators)


class SimpleGaussianQualitator(Qualitator):
    def __init__(self,  name, dimensions, gaussian, std_threshold=0.1):
        super(SimpleGaussianQualitator, self).__init__(name, dimensions)
        self._gaussian = gaussian
        self._threshold = std_threshold
    
    def __call__(self, geometry):
        #s = self._gaussian.calc_error(geometry)
        #if s < self._threshold:
            #return True
        #else:
            #return False
        s = self._gaussian.sample(geometry)
        if s > self._threshold:
            return True
        else:
            return False
    
    
    def __getstate__(self):
        """ 
        For pickling this class
        """
        odict = self.__dict__.copy()
        return odict
    
    def __setstate__(self, state):
        """
        To set the state when deepcopy and pickle
        """
        self.__dict__.update(state)   # update attributes
        

import qsr_calc

class TwoPointCalcAngleQualitator(Qualitator):
    def __init__(self, name, partions):
        super(TwoPointCalcAngleQualitator, self).__init__(name, 2)
        self.partitions = partions
        self.cam_pos = [1.0,-2.0,1.698]
    def __call__(self, geometry):
        rela = qsr_calc.relative_angle(self.cam_pos, [0, 0, 0], geometry[0:3])
        part = qsr_calc.argmax_partition(rela)
        if part in self.partitions:
            return True
        else:
            return False
        
        
class TwoPointCalcDistQualitator(Qualitator):
    def __init__(self, name, valid_range):
        super(TwoPointCalcDistQualitator, self).__init__(name, 2)
        self.valid_range = valid_range
        self.cam_pos = [1.0,-2.0,1.698]
    def __call__(self, geometry):
        reld = qsr_calc.relative_radius(self.cam_pos,[0, 0, 0], geometry[0:3])

        if reld >=  self.valid_range[0] and reld < self.valid_range[1]:
            return True
        else:
            return False
        
        
                
        
class SizeQualitator(Qualitator):
    def __init__(self, name, side):
        super(SizeQualitator, self).__init__(name, 2)
        self.d = side
    def __call__(self, geometry):
        if geometry[self.d+3] > geometry[self.d+6]:
            return True
        return False
    
        
                
        
class AbsoluteSizeQualitator(Qualitator):
    def __init__(self, name, rng):
        super(AbsoluteSizeQualitator, self).__init__(name, 1)
        self.range = rng
    def __call__(self, geometry):
        volume = geometry[3] * geometry[4] * geometry[5]
        if volume >  self.range[0] and volume < self.range[1]:
            return True
        return False
    
