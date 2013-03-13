from __future__ import division
from pylab import *
from numpy.testing import assert_almost_equal
from line_def import lines
import time, argparse
from points_generator import *
import line_def

### utility functions ###
def abc_dist((a,b,c), points):
    u = array([a, b])
    u /= norm(u)
    P0 = array([0, -c/b]) if abs(b)>abs(a) else array([-c/a, 0])
    vectors = points - P0
    dists = abs(dot(vectors, u)).squeeze()
    return dists
def line_dist(l, points):
    return array([l.diff(p) for p in points])

### main functional class ###
class Evaluator:
    timestamp = None
    duration = None
    experiment_name = None
    sigma = "N/A"
    dataset = line_def.selection
    num_lines = len(line_def.lines)
    detected_walls = None
    
    def __init__(self, experiment_name=None, sigma=None, dataset=None, num_lines=None, detected_walls=None, parse_args=True):
        if parse_args:
            parser = argparse.ArgumentParser()
            parser.add_argument('-s', '--sigma', type=int, default=None)
            args = parser.parse_args()
            sigma = args.sigma
        if experiment_name: self.experiment_name = experiment_name
        if sigma: self.sigma = sigma
        if dataset: self.dataset = dataset
        if num_lines: self.num_lines = num_lines
        if detected_walls: self.detected_walls = detected_walls
        self.tic()
    
    def set_sigma(self, sigma):
        self.sigma = sigma
    
    def tic(self):
        self.timestamp = time.time()
    
    def toc(self):
        self.duration = time.time() - self.timestamp
    
    def evaluate_lines(self, lines, linetype, dataset_points):
        self.duration or self.toc()
        points = generate_noiseless_points()
        K = len(lines); N = len(points)
        dists = zeros((K, N))
        for k, l in enumerate(lines):
            if linetype == 'abc':
                d = abc_dist(l, points)
            elif linetype == 'line':
                d = line_dist(l, points)
            dists[k, :] = d
        dists = dists.min(0)
        meandist = dists.mean()
        rmsdist = sqrt((dists ** 2).mean())
        print "{}\t{}\t{}\t{}\t{}\t{:.1f}\t{:.2f}\t{}".format(
            self.experiment_name,
            self.dataset,
            self.sigma,
            self.num_lines,
            len(dataset_points),
            self.duration,
            rmsdist,
            K if not self.detected_walls else self.detected_walls
        )




