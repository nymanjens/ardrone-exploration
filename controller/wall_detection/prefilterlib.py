from numpy import *
from sets import Set

""" globals """
M = 50 # num steps
R = lambda(alpha): array([
    [cos(alpha),-sin(alpha)],
    [sin(alpha), cos(alpha)]
])

class ScoreBoard:
    score = None
    min_num_neighbours = None
    
    def __init__(self, N, min_num_neighbours):
        self.score = array([0] * N)
        self.min_num_neighbours = min_num_neighbours
    
    def append(self, matches):
        self.score[0] = sum(matches)-1
        for n, match in enumerate(matches):
            if n and match:
                self.score[n] += 1
        winners = self.score >= self.min_num_neighbours
        undefined = self.score < self.min_num_neighbours
        undefined[0] = False
        self.score = self.score[undefined]
        return winners, undefined
    
def get_definite_points(points, threshold=500, min_num_neighbours=20):
    """ these points are definitely not outliers
        (definite if enough neighbours) """
    N = len(points)
    threshold2 = threshold**2
    scoreBoard = ScoreBoard(N, min_num_neighbours)
    winners_list = []
    for n in range(N):
        neigh = ((points - points[0,:])**2).sum(1) < threshold2
        winners, undefined = scoreBoard.append(neigh)
        winners_list += points[winners].tolist()
        points = points[undefined]
        if not sum(undefined):
            break
    return array(winners_list)

def envelope_from_points(points):
    global R, M
    envelope_set = Set()
    for alpha in arange(0, pi/2, pi/2/M):
        tfpoints = dot(points, R(alpha).T)
        for i in range(2):
            envelope_set.add(tfpoints[:,i].argmin())
            envelope_set.add(tfpoints[:,i].argmax())
    return points[list(envelope_set)]

def filter_clusters_with_envelope(envelope, clusters):
    global R, M
    accepted = []
    for c in clusters:
        inside = array([True]*len(c))
        for alpha in arange(0, pi/2, pi/2/M):
            tfc = dot(c, R(alpha).T)
            tfenv = dot(envelope, R(alpha).T)
            for i in range(2):
                inside = logical_and(inside, tfc[:,i] > tfenv[:,i].min())
                inside = logical_and(inside, tfc[:,i] < tfenv[:,i].max())
        if inside.sum() * 1. / len(c) < .9:
            accepted += c.tolist()
    return array(accepted)

def enlighten_dense_areas(points, thres=50):
    """ enlighten dense areas """
    thres2 = thres**2
    pending = array([True] * len(points))
    rejected = array([False] * len(points))
    for n, p in enumerate(points):
        if pending[n]:
            diffs = ((points[pending] - points[n])**2).sum(1)
            neighbours = diffs < thres2
            rejected[pending] = logical_or(rejected[pending], neighbours)
            rejected[n] = False
            pending[n] = False
            pending[rejected] = False
    return points[logical_not(rejected)]
    






