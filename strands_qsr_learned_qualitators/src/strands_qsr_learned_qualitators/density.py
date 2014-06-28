import numpy as np
import scipy
import random
import math
from sklearn.covariance import oas, empirical_covariance

########################################################################
class GaussKernel(object):
    """A kernel with covariance and mean. Given a set of points,
    calculates mu and Sigma """

    #----------------------------------------------------------------------
    def __init__(self, data_points, weight):
        """Constructor"""
        assert isinstance(data_points, np.ndarray)
        self.raw_data =  data_points
        self.weight = weight
        self.mu = np.sum(data_points, 0) / (1.0 * data_points.shape[0])
        
        c = np.zeros((data_points.shape[1], data_points.shape[1]))
        #print data_points.shape
        #for x in data_points:
            #x_minus_mu = np.array([np.array(x) - self.mu])
            ## print x_minus_mu, x_minus_mu.transpose()
            ## print np.dot(x_minus_mu.transpose(), x_minus_mu)
            #c += np.dot(x_minus_mu.transpose(), x_minus_mu)
        #c /= data_points.shape[0]
        #self.cov = c
        
        #print data_points.shape
        self.cov =  empirical_covariance(data_points) #[0]
        #self.cov, shrink =  oas(data_points) #[0]
        #if shrink != 1:
            #self.cov /= (1 - shrink)
            
        #print self.cov
        
        self.eig_vals, self.eig_vecs = np.linalg.eig(self.cov)

       
        # precompute some parts
        self.cov_inv = np.linalg.inv(self.cov)
        self.root_2pi_d_det = math.sqrt(((2.0*math.pi)**data_points.shape[1]) *
                                        np.linalg.det(self.cov))
        #self.root_2pi_d_det = ( (2.0*math.pi) ** (-(data_points.shape[1]) / 2) *
                                        #np.linalg.det(self.cov) ** -0.5 )
    
    #----------------------------------------------------------------------
    def precompute(self):
        """used only in debugging"""
        self.cov_inv = np.linalg.inv(self.cov)
        self.root_2pi_d_det = math.sqrt((2.0*math.pi)**self.raw_data.shape[1] *
                                        np.linalg.det(self.cov))
        
    #----------------------------------------------------------------------
    def sample(self, x):
        """Sample the gaussian kernel at point x; x is np.ndarray"""
        x_minus_mu = x - self.mu
        pwr = -(np.dot(np.dot (x_minus_mu.transpose(), self.cov_inv),
                       x_minus_mu) / 2.0)
        return math.exp(pwr)  #/ self.root_2pi_d_det
    
    def calc_error(self, x):
        x_minus_mu = np.array ([x - self.mu]).T
        p =  np.dot(self.eig_vecs.T, self.eig_vecs)
        pt =  np.dot(p, x_minus_mu)
        pt /= np.sqrt(np.array([self.eig_vals]).T)
        std_dev = np.linalg.norm(pt)
        return std_dev
#        err =  
        
    
########################################################################
class DensityEstimator(object):
    """A collection of GaussKernels for modeling a distribution."""

    #----------------------------------------------------------------------
    def __init__(self):
        """Constructor"""
        self.normalisation_vector = []
        pass
    
    #----------------------------------------------------------------------
    @staticmethod
    def createFromData(data, r):
        """Create a model from a dataset, numpy array each row a data point,
        with threshold r"""
        m = DensityEstimator()
        #print "Creating denisty estimator from data..."

        m.mixture_dimensions = data.shape[1]
        m.dataset_size = data.shape[0]
        m.threshold = r
        
        # Normalisation of the data set
        m.normalisation_vector_min = np.array([0.0]*data.shape[1])
        m.normalisation_vector_max = np.array([0.0]*data.shape[1])
        for i in range(0, data.shape[1]):
            #data[:, i] -= min(data[:, i])
            #print i, ":  ", max(data[:, i])
            m.normalisation_vector_min[i] = min(data[:, i]) 
            m.normalisation_vector_max[i] = max(data[:, i])
            if m.normalisation_vector_min[i] == m.normalisation_vector_max[i]:
                m.normalisation_vector_min[i] = 0.0
                
            #data[:, i] -= m.normalisation_vector_min[i]
            #data[:, i] /= (m.normalisation_vector_max[i] - m.normalisation_vector_min[i])
            #data[:, i] /= (m.normalisation_vector_max[i] )
            #print m.normalisation_vector[i]
        #return
            
        # Split into clusters / kernels
        t=set()
        d=set(range(0,data.shape[0]))
        def get_random():
            sample = random.randint( 0, data.shape[0]-1 )
            while sample in t:
                sample = random.randint( 0, data.shape[0] - 1 )
            t.add(sample)
            return sample
        
        sets=[]
        while len(t)<data.shape[0]:
            i = get_random()
            sets.append(set([i]))
            #print "Cluster at", i,  
            for j in d-t:
                if np.sqrt(sum((data[i,:]-data[j,:])**2)) < r:
                    sets[-1].add(j)
                    t.add(j)
            #print "members = ",  len (sets[-1])
            new_set = list()
            for i in sets[-1]:
                new_set.append(data[i, :])
            sets[-1] = np.array(new_set)
        
        #print "Cluster count=",len(sets)
        #for i in sets:
            #print 'sets: ',i
            
        # Turn each cluster into a Gaussian kernel

        # Any set that has less than 3 members merge with nearests
        minimum_members = 2 #data.shape[1] # mixture dimensions to avoid singular covariance.
        #print sets
        #for i in sets:
            #if len(i) < minimum_members:
                #print "Small set "
                #for pt in i:
                    #close = [None, 1e100000]
                    #for j in sets:
                        ##print i
                        #print j
                        #if (i == j).all():
                            #print "same"
                            #continue
                        #print "diff"
                        #if len(j) < minimum_members:
                            #continue # err, bad, should allow possibile join smalls
                        #mu = np.sum(i, 0) / (1.0 * i.shape[0])
                        #dist = sum([(m - p)**2 for m, p in zip(mu, pt)])
                        #print dist
                        #if dist < close[1]:
                            #close[1] = dist
                            #close[0] = j
                    #close[0].add(pt)
        
        
        m.kernels = []
        for i in sets:
            if len(i) >= minimum_members:
                m.kernels.append(GaussKernel(i, i.shape[0]/(1. * data.shape[0])))
            else:
                print "LOST SOME"
            #else:
                #print "->Warning: dropping data point as it lies alone, singular"
        #print "ok."
        return m
    
    #----------------------------------------------------------------------
    def sample(self, x):
        """Sample the estimator"""
        #s = np.array([0.0]*len(x))
        #s = copy.deepcopy(x)
        s = x
        #print "n vec is %s" % repr(self.normalisation_vector)
        #s -= self.normalisation_vector_min
        #s  /= (self.normalisation_vector_max - self.normalisation_vector_min)
        #s /= (self.normalisation_vector_max )
        #print "maxy=%s" % repr(self.normalisation_vector_max)
        #print "miny=%s" % repr(self.normalisation_vector_min)
        #for i in range(0, len(x)):
            #s[i] = x[i] / self.normalisation_vector[i]
        ret = 0
        for i in self.kernels:
            ret += i.weight * i.sample(s)
        #ret /= len(self.kernels)
        return ret
    
    #----------------------------------------------------------------------
    def print_details(self):
        """"""
        print "Kernel weights: ", self.weights
        
        
if __name__ == '__main__':
    ''' Main Program '''
    import matplotlib
    import numpy as np
    import matplotlib.pyplot as plt
    
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib import cm
    
    
    print "Testing Gaussian"
    
    step = 0.05
    X,Y = np.meshgrid(np.arange(-0.8, 0.8, step),np.arange(-2, 2, step))
    Z = np.zeros((X.shape[0],X.shape[1]))
    
    p = np.array([0,0,0])
    
    #import numpy as np
    #points = np.random.random((6, 2))
    
    points = np.array([[ 0.12522647,  0.40287539],
                      [ 0.43694239,  0.73224165],
                      [ 0.38070038,  0.89789005],
                      [ 0.63841   ,  0.67966256],
                      [ 0.59059136,  0.2605457 ],
                      [ 0.4577748 ,  0.5437255 ],
                      [ 0.78173593,  0.51853548],
                      [ 0.18052673,  0.67760897],
                      [ 0.79310395,  0.83942354],
                      [ 0.6442909 ,  0.86481114],
                      [ 0.85338047,  0.35936989],
                      [ 0.65821848,  0.26428388]])
    points[:, 1] -=0.5
    points[:, 0] -=0.5
    
    points[:, 1] = points[:, 0] + points[:, 1]
    g = GaussKernel(points, 1)
    pts = []
    for x in range(0,X.shape[0]):
        for y in range(0,X.shape[1]):
            s = [X[x,y], Y[x,y]]
            p = np.array(s)
    
            a = g.sample(p) 
            if g.sample(p) > 0.1: #< 2:
                pts.append(s)
            Z[x, y] =   a 
            
            
    pts = np.array(pts)
    fig, ax = plt.subplots()
    
    p = ax.pcolor(X, Y, Z, cmap=cm.jet, vmin=abs(Z).min(), vmax=abs(Z).max())
    isinstance(ax, plt.Axes)
    ax.plot(pts[:, 0], pts[:, 1] , 'm.')
    
    plt.rc('text', usetex=True)
    fontsize = 20
    plt.xlabel(r'$X$',  fontsize=20)
    plt.ylabel(r'$Y$',  fontsize=20)
    plt.title(r'A test using $x$',  fontsize=20)
    ax.plot(points[:, 0], points[:, 1] , 'o')
    plt.show()
    
    