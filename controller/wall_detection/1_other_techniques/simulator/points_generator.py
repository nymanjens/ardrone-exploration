""" Note: should be deterministic, zo all algorithms are compared in the same way """
from numpy import *
from numpy.random import *
from numpy.linalg import norm
import time
from line_def import lines

def generate_points(sigma, points_per_meter=20):
    # ensure deterministic behaviour
    seed(int(sigma))
    # init
    points = []
    # generate points for each segment
    for A, B in lines:
        L = norm(B-A) # L is in mm
        N = round(points_per_meter * L/1e3)
        Delta = 1./N
        sigma_N = Delta # point placement noise
        
        """ get ideal placment of points """
        t = arange(Delta/2, 1, Delta)
        assert len(t) == N
        
        """ scramble points (still on line) """
        for i,x in enumerate(t):
            x += Delta * randn()
            x = max(0, x)
            x = min(1, x)
            t[i] = x
        
        """ transform 1D t-values to xy values """
        p = array([tt*A + (1-tt)*B for tt in t])
        
        """ add gaussian noise to scrambled points on line """
        p += sigma*randn(*p.shape)
        
        # save points
        points += p.tolist()
        
    # restore seed
    seed(int(1e3*time.time()  % 1e6))
    return array(points)

if __name__ == '__main__':
    points = generate_points(50)
    
    from plotter import *
    from pylab import *
    plot_lines()
#    plot_points(points)
    show()
    
