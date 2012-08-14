from outlierfilterlib import filter_outliers
from points import points, N
from pylab import *
import time
import argparse

""" parse args """
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--action', default='show')
args = parser.parse_args()

""" show live plot of how it works """
if args.action == 'show':
    print N
    """ plot 1: points """
    #axes().set_aspect('equal')
    #plot(points[:,0]/1e3, points[:,1]/1e3, '.b')
    #xlabel('x (m)')
    #ylabel('y (m)')
    #show()
    orig_points = points
    points = filter_outliers(points)
    rej = array([p for p in orig_points if not p in points])
    """ plot 2: accepted/rejected """
    #axes().set_aspect('equal')
    #plot(rej[:,0]/1e3, rej[:,1]/1e3, 'xk')
    #plot(points[:,0]/1e3, points[:,1]/1e3, '.b')
    #legend(['rejected', 'accepted'], loc='best')
    #xlabel('x (m)')
    #ylabel('y (m)')
    #show()
    """ plot 3: accepted """
    #axes().set_aspect('equal')
    #plot(points[:,0]/1e3, points[:,1]/1e3, '.b')
    #plot(rej[:,0]/1e3, rej[:,1]/1e3, '.w')
    #xlabel('x (m)')
    #ylabel('y (m)')
    #show()
    """ plot 4: after density filter """
    sys.path.append('..')
    import prefilterlib
    fpoints = prefilterlib.enlighten_dense_areas(points)
    plot(points[:,0]/1e3, points[:,1]/1e3, 'oy', label="rejected")
    plot(fpoints[:,0]/1e3, fpoints[:,1]/1e3, 'ob', label="accepted")
    xlabel('x (m)')
    ylabel('y (m)')
    legend()
    show()

""" check how long the algorithm takes """
if args.action == 'time':
    M = 50
    tic = time.clock()
    for i in range(M):
        filter_outliers(points)
    toc = time.clock()
    print "done in ", (toc-tic)/M, "s"
