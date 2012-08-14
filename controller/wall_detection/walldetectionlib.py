""" imports """
# std imports
import sys, os, time
import numpy as np
import pylab as pl
from numpy import array
from numpy.linalg import norm
import multiprocessing, time
# ROS imports
import roslib; roslib.load_manifest('controller')
# custom imports
import floorseparator
from outlierfilter import outlierfilterlib
from floorseparator import is_wall
from wmm.wmmlib import EM_wall_mixture_model
from wmm.Line import Line
from progressbar import ProgressBar # http://code.google.com/p/python-progressbar/
import prefilterlib
sys.path.append(os.path.dirname(__file__) + '/1_other_techniques/')
from simulator import plotter

MIN_NUM_POINTS = 20
FANCY_OUTPUT = False
final_points = None

""" preprocessor """
#def preprocess_points(points):
#    ## convert 3D points to 2D
#    if points.shape[1] == 3:
#        points = points[:, :2]
#    ## check if enough points
#    if len(points) < MIN_NUM_POINTS:
#        return None
#    ## filter outliers by clustering
#    points = outlierfilterlib.filter_outliers(points)
#    ## check if enough points
#    if len(points) < MIN_NUM_POINTS:
#        return None
#    return points

""" preprocessor """
def preprocess_points(points):
    global FANCY_OUTPUT
    
    ## divide points ##
    wallpoints = array([p for p in points if floorseparator.is_wall(p)])
    floorpoints = array([p for p in points if floorseparator.is_floor(p)])
    
    ## check if enough points
    if len(wallpoints) < MIN_NUM_POINTS: return None
    if len(floorpoints) < MIN_NUM_POINTS: return None
    
    ## convert 3D points to 2D ##
    if wallpoints.shape[1] == 3:
        wallpoints = wallpoints[:, :2]
        floorpoints = floorpoints[:, :2]
    
    ## plot points
    if FANCY_OUTPUT:
        plot_points([p for p in wallpoints if norm(p) < 10e3])
        pl.legend(['raw wall points'], 'best')
        pl.draw()
    
    ## filter outliers by clustering ##
    wallpoints = outlierfilterlib.filter_outliers(wallpoints, threshold=300, min_cluster_size=20)
    clusters = outlierfilterlib.filter_outliers(wallpoints, threshold=150, min_cluster_size=1, return_clusters=True)
    
    ## plot filtered points
    if FANCY_OUTPUT:
        pts = []
        for c in clusters: pts += c.tolist()
        pl.clf()
        plot_points(pts, '.y')
        pl.legend(['after cluster filter'], 'best')
        pl.draw()
    
    ## filter outliers by floor points ##
    definite_points = prefilterlib.get_definite_points(floorpoints)
    envelope_points = prefilterlib.envelope_from_points(definite_points)
    wallpoints = prefilterlib.filter_clusters_with_envelope(envelope_points, clusters)
    
    ## plot filtered points
    if FANCY_OUTPUT:
        plot_points(wallpoints, '.b')
        pl.legend(['after cluster filter', 'after floor filter'], 'best')
        pl.draw()
    
    ## enlighten dense areas ##
    wallpoints = prefilterlib.enlighten_dense_areas(wallpoints)
    
    ## plot filtered points
    if FANCY_OUTPUT:
        pl.clf()
        plot_points(wallpoints)
        pl.legend(['final'], 'best')
        pl.draw()
    
    ## check if enough points
    if len(wallpoints) < MIN_NUM_POINTS:
        return None
    return wallpoints
    
""" detect walls without room model """
def detect_walls(points, do_preprocessing=True):
    ## preprocess points
    if do_preprocessing:
        points = preprocess_points(points)
    if points == None:
        return []
    
    ## TMP: detect room instead of points
    return detect_room(points, do_preprocessing=False, numwalls=[4], numcalculations=3)
    
    """ detect walls from point cloud """
    ## get ideal K and consequently ideal lines
    bestL = None
    bestLogProb = float('-inf')
    for K in range(1, 20): # max 19 walls
        L, logProb = EM_wall_mixture_model(points, K = K)
        #print "  K={0} --> logProb={1}".format(K, logProb)
        if logProb > bestLogProb:
            bestLogProb = logProb
            bestL = L
        elif K > 3: # stop on decreasing prob
            break
    
    return bestL

""" detect room """
def detect_room(points, do_preprocessing=False, numwalls=None, numcalculations=10, fancy_output=False):
    """ detects room structures from walls """
    ## parse params
    global final_points, FANCY_OUTPUT; FANCY_OUTPUT = fancy_output
    if numwalls == None: numwalls = range(4, 20, 2)
    ## start plotting
    if FANCY_OUTPUT:
        pl.ion()
        #fig = pl.figure()
    ## do preprocessing
    if do_preprocessing:
        points = preprocess_points(points)
        if points == None:
            return []
    final_points = points
    ## get ideal K and consequently ideal lines
    bestL = None
    bestLogProb = float('-inf')
    for K in numwalls:
        executor = DuplicatedParallelExecution()
        L, logProb = executor.execute(detect_room_one_iteration, [points, K], M=numcalculations)
        print "  [K={0}] best: logProb={1}".format(K, logProb)
        if logProb > bestLogProb:
            bestLogProb = logProb
            bestL = L
        elif K > 7: # stop on decreasing prob
            break
    ## plot walls
    if FANCY_OUTPUT:
        pl.clf()
        plot_points(final_points)
        plotter.plot_walls(L)
        pl.legend(['points', 'final walls'], 'best')
        pl.draw()
        time.sleep(2)
        pl.close()
    return bestL

###################### UTILITY FUNCTIONS ######################
""" execute on multiple cores """
class DuplicatedParallelExecution:
    N = 2 # number of parallel executions
    
    def execute(self, function, args, M):
        """ do parallel execution """
        N = min(self.N, M)
        threads = [None] * N
        cnt = 0
        result_queue = multiprocessing.Queue()
        results = []
        if FANCY_OUTPUT: progressBar = ProgressBar(maxval=M).start()
        while cnt < M:
            time.sleep(0.005)
            for n, thread in enumerate(threads):
                if not thread or not thread.is_alive():
                    while not result_queue.empty():
                        results.append(result_queue.get_nowait())
                    threads[n] = multiprocessing.Process(target=function, args=args+[result_queue, cnt])
                    threads[n].start()
                    cnt += 1
                    if FANCY_OUTPUT: progressBar.update(cnt - self.N if cnt > self.N else 0)
                    if FANCY_OUTPUT: plot_results(results)
                    if cnt == M:
                        break
                    time.sleep(0.01)
        for thread in threads:
            thread.join()
        
        if FANCY_OUTPUT: progressBar.finish()
        
        """ calculate minimum from returnvals """
        while not result_queue.empty():
            results.append(result_queue.get_nowait())
        assert len(results) == M
        logProbs = array([x[1] for x in results])
        return results[logProbs.argmax()]

""" one iteration of room detection """
def detect_room_one_iteration(points, K, result_queue, cnt):
    np.random.seed(int(1e3*time.time()  % 1e6) + cnt)
    L, logProb = EM_wall_mixture_model(points, K = K, MAX_ITER=10)
    # extra check: all lines connected with eachother
    if not Line.linesFormClosedShape(L):
        logProb *= 2 # this is the same as sqrt(prob)
    if not FANCY_OUTPUT:
        print "    (K={0}) score={2:.1f} logProb={1}".format(K, logProb, -logProb/1e4)
    result_queue.put([L, logProb])


""" utility plot functions """
def plot_points(pts, arg='.', **kwargs):
    pts = array(pts)
    pl.axes().set_aspect('equal'); x,y = pts.T/1e3; pl.plot(x, y, arg, **kwargs)
    pl.xlabel('x (m)')
    pl.ylabel('y (m)')
def plot_results(results):
    global final_points
    if not len(results):
        return
    logProbs = array([x[1] for x in results])
    bestind = logProbs.argmax()
    L = results[bestind][0]
    pl.clf()
    plot_points(final_points)
    plotter.plot_walls(L)
    pl.legend(['points', 'best walls (%d)' % len(results)], 'best')
    pl.draw()
        


