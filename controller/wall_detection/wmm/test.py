""" wall mixture model """
from points import points, N
from wmmlib import *
from pylab import *
import pylab as pl
from copy import copy
import time
import pypr.clustering.gmm as gmm # download from http://pypr.sourceforge.net/
import argparse

""" settings """
K = 4
print N

""" plot function """
gmm_cen_lst, gmm_cov_lst, p_k, logL = gmm.em_gm(points, K = K)
def debug_output(L, gamma=None, iter_num=None):
    
    """ check final """
    if gamma != None:
        final = iter_num == None
        if final: ioff()
        #else: return
    
    """ text output """
    if gamma != None:
        print "iteration %d..." % iter_num if not final else "\nFINAL"
    
    """ init vars """
    COLORS = "bykgrcm"
    if gamma == None:
        gamma = zeros((N, K))
    
    """ plot init """
    clf()
    axes().set_aspect('equal')

    
    """ plot points """
    for n, point in enumerate(points):
        color = COLORS[gamma[n, :].argmax() % len(COLORS)]
        plot(point[0], point[1], color+'.')
    
    """ plot walls """
    for k, l in enumerate(L):
        color = COLORS[k % len(COLORS)]
        l.plot(pl, color)
    
    """ plot gaussians """
    #for i in range(len(gmm_cen_lst)):
    #    x1,x2 = gmm.gauss_ellipse_2d(gmm_cen_lst[i], gmm_cov_lst[i])
    #    plot(x1, x2, 'k')
    
    """ misc """
    vals = points[:,0]; xlim((vals.min()-100, vals.max()+100))
    vals = points[:,1]; ylim((vals.min()-100, vals.max()+100))
    show()
    gcf().canvas.draw()

""" parse args """
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--action', default='show')
parser.add_argument('-g', '--dont_init_from_gmm', action='store_true')
args = parser.parse_args()
args.init_from_gmm = not args.dont_init_from_gmm

""" show live plot of how it works """
if args.action == 'show':
    ion()
    L, logProb = EM_wall_mixture_model(points, K = K, MAX_ITER = 100, init_from_gmm=args.init_from_gmm, debug_output = debug_output)
    print Line.linesFormClosedShape(L)

if args.action == 'run':
    EM_wall_mixture_model(points, K = K, MAX_ITER = 100, init_from_gmm=args.init_from_gmm)


""" check how long the algorithm takes """
if args.action == 'time':
    #print "with init_from_gmm"
    M = 5
    tic = time.clock()
    for i in range(M):
        EM_wall_mixture_model(points, K = K, MAX_ITER = 100, init_from_gmm=args.init_from_gmm, debug_output = None)
    toc = time.clock()
    print "done in ", (toc-tic)/M, "s"
    #print "without init_from_gmm"
    #tic = time.clock()
    #for i in range(M):
    #    EM_wall_mixture_model(points, K = K, MAX_ITER = 100, init_from_gmm=False, debug_output = None)
    #toc = time.clock()
    #print "done in ", (toc-tic)/M, "s"

""" optimize K """
if args.action == 'K':
    ion()
    K_s = arange(1, 5)
    logProb_s = []
    for K in K_s:
        L, logProb = EM_wall_mixture_model(points, K = K)
        print "  K = {0} --> logProb = {1}".format(K, logProb)
        logProb_s.append(logProb)
        debug_output(L)
    logProb_s = array(logProb_s)
    
    ioff()
    figure()
    plot(K_s, logProb_s, 'b-o')
    show()






