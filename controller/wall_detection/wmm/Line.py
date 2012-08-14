import random, itertools
from numpy import array, ones, zeros, arange, exp, log, pi, sqrt, hstack, vstack, diag, matrix, cos, sin, tan, arctan, arccos, dot, isnan, atleast_2d
from numpy.linalg import norm, eigh
from numpy.random import rand, randn
from numpy.testing import assert_almost_equal
import numpy as np

""" settings """
THRESHOLD_E = 3500. #mm
THRESHOLD_ALMOST_CONNECTED = 400 #mm

""" line: {point, alpha, e}
          (alpha elem [0, pi[)
"""
class Line:
    M = None
    alpha = None
    sigma = None
    e = None
    
    def __init__(self, M, alpha, sigma, e):
        self.M = array(M)
        self.alpha = alpha
        self.sigma = sigma
        self.e = e
        self.initOptimCache()

    def __str__(self):
        x,y = self.M
        return "<M=({0},{1}), alpha={2}, sigma={3}, e={4}>".format(
            x, y, self.alpha, self.sigma, self.e
        )
        
    def __eq__(self, L):
        return norm(self.E() - L.E()) < .001
    
    optimCache = None
    def initOptimCache(self):
        M = self.M if len(self.M.shape) else [None, None]
        self.optimCache = {
            '_': [M[0], M[1], self.alpha, self.sigma, self.e],
        }
    def checkOptimizer(self):
        """ check if up to date """
        if not self.optimCache['_'] == [self.M[0], self.M[1], self.alpha, self.sigma, self.e]:
            self.initOptimCache()
    def fromOpimizer(self, var):
        self.checkOptimizer()
        if var in self.optimCache:
            return self.optimCache[var]
        return None
    def toOpimizer(self, var, val):
        self.optimCache[var] = val
        
    
    @classmethod
    def getRandom(Self, points):
        # get M (from random point)
        M = array(random.choice(points))
        M += 50.0*randn(2)
        # get alpha
        alpha = pi*rand()
        # get sigma
        sigma = 100.0
        # get e
        e = 200
        return Self(M, alpha, sigma, e)
    
    @classmethod
    def fromEllipse(Self, cen, cov):
        l = Self(M=None, alpha=None, sigma=None, e=None)
        sigma_x = l.updateFromEllipse(cen, cov)
        l.e = sigma_x * 1.732
        return l
    
    def updateFromEllipse(self, cen, cov):
        # sanity check
        if norm(cov) < .001:
            cov = np.identity(2)
        # set center
        self.M = cen
        # get eigenvals of cov
        vals, eigs = eigh(cov)
        eigs = [eigs[:, 0], eigs[:, 1]]
        # sort according to eigenvalues
        if vals[1] > vals[0]:
            vals = vals[::-1]
            eigs.reverse()
        # get alpha
        self.updateAlphaFromVector(eigs[0])
        # get sigma
        self.sigma = sqrt(vals[1])
        # return largest eigenvalue (= sigma_x)
        sigma_x = sqrt(vals[0])
        #print "sigma_x*1.732 = ", sigma_x * 1.732
        return sigma_x
    
    def updateFromE(self, (E0, E1)):
        self.M = (E0 + E1) / 2
        self.updateAlphaFromVector(E0 - E1)
        self.e = norm(E0 - E1) / 2
        # assertion:
        try:
            assert_almost_equal(self.E(), vstack((E0, E1)))
        except AssertionError:
            assert_almost_equal(self.E(), vstack((E1, E0)))
    
    def updateAlphaFromVector(self, v):
        if abs(v[0]) < .00001:
            alpha = pi/2
        else:
            alpha = arctan(v[1]/v[0])
            if alpha < 0: alpha += pi
        self.alpha = alpha
    
    def x(self, t):
        x0, y0 = self.M; alpha = self.alpha
        return cos(alpha)*t + x0
    
    def y(self, t):
        x0, y0 = self.M; alpha = self.alpha
        return sin(alpha)*t + y0
    
    def P(self, t):
        return array([self.x(t), self.y(t)])
    
    def E(self, i=-1):
        E = self.fromOpimizer('E')
        if E == None:
            e = self.e
            E = array([self.P(-e), self.P(e)])
            self.toOpimizer('E', E)
        return E if i == -1 else E[i]
    
    def ab(self):
        x0, y0 = self.M; alpha = self.alpha
        a = tan(alpha)
        b = y0 - a*x0
        return a,b
    
    def diff(self, P):
        e = self.e
        x, y = self.normalizeCoords(P)
        if abs(x) > e:
            return norm([abs(x)-e, y])
        else:
            return abs(y)
    
    def hasPoint(self, P):
        return self.diff(P) < .001
    
    
    def OLDnormalizeCoords(self, P):
        """ transform coords from xy to axises on line """
        M, E1 = self.M, self.E(1)
        v = P - M
        w = E1 - M
        ux = w / norm(w)
        uy = array([-ux[1], ux[0]])
        A = array([ux, uy]).T
        return dot(v, A)
    
    
    def normalizeCoords(self, P):
        """ transform coords from xy to axises on line """
        M = self.M
        v = P - M
        A = self.fromOpimizer('A')
        if A == None:
            w = self.E(1) - M
            ux = w / norm(w)
            uy = array([-ux[1], ux[0]])
            A = array([ux, uy]).T
            self.toOpimizer('A', A)
        return dot(v, A)
    
    def Prob(self, P, normalized=False):
        return self.conditionalProb(P, normalized)
    
    def conditionalProb(self, P, normalized=False):
        sigma, e = self.sigma, self.e
        # constants
        probparams = self.fromOpimizer('probparams')
        if probparams != None:
            alpha_2pisigma2, inv2sigma2 = probparams
        else:
            # alpha (scaling param)
            alpha = 1 / (1 + sqrt(2/pi)*e/sigma)
            alpha_2pisigma2 = alpha / (2*pi*sigma**2)
            inv2sigma2 = 1./(2*sigma**2)
            self.toOpimizer('probparams', [alpha_2pisigma2, inv2sigma2])
        # get transformed point
        x, y = self.normalizeCoords(P) if not normalized else P
        # probablility
        if abs(x) < e: # inside point
            return alpha_2pisigma2 * exp(-inv2sigma2 * y*y)
        else: # edge point
            dx = abs(x)-e
            return alpha_2pisigma2 * exp(-inv2sigma2 * (dx*dx + y*y))
    
    def realProb(self, P, normalized=False):
        # constants
        sigma, e = self.sigma, self.e
        normPdf = lambda x: 1/(sqrt(2*pi)*sigma) * exp(-x**2 / (2*sigma**2))
        # get transformed point
        x, y = self.normalizeCoords(P) if not normalized else P
        from scipy.special import erf
        # get prob
        erf1 = erf( (e+x) / (sqrt(2)*sigma) )
        erf2 = erf( (e-x) / (sqrt(2)*sigma) )
        return (erf1 + erf2) / (4*e) * normPdf(y)
    
    def inBadCondition(self):
        return isnan(self.sigma) or self.sigma < .01
    
    def almostEqualTo(self, L):
        # get dalpha
        dalpha = abs(self.alpha - L.alpha)
        dalpha = abs(dalpha-pi) if dalpha > pi/2 else dalpha
        # get dM
        dM = norm(self.M - L.M)
        # dist
        dist = 100/pi*dalpha + norm(dM)
        return dist < 1

    def plot(self, plt, color='r'):
        alpha, sigma, e = self.alpha, self.sigma, self.e
        t = arange(-e, e, 2*e/300)
        x = self.x(t)
        y = self.y(t)
        a,b = self.ab()
        # plot wall
        plt.plot(x, y, color+'-')
        # plot sigma lines
        dx = self.sigma / sqrt(1+1./a**2)
        dy = -dx/a
        plt.plot(x + dx, y + dy, color+':')
        plt.plot(x - dx, y - dy, color+':')
        # plot edge sigma lines
        E1 = self.P(t=-e)
        E2 = self.P(t=e)
        theta = arange(0, pi, .01)
        for E, angle in (
            (E2, -pi/2 + alpha + theta),
            (E1, pi/2 + alpha + theta),
        ):
            Px, Py = E[0] + sigma*cos(angle), E[1] + sigma*sin(angle)
            plt.plot(Px, Py, color+':')
    
    def getNearestEdgeToEdge(self, l):
        distToEdge = {}
        for selfE, lE in itertools.product(self.E(), l.E()):
            dist = norm(selfE - lE)
            distToEdge[dist] = (selfE, lE)
        minDist = min(distToEdge.keys())
        return minDist, distToEdge[minDist]

    def isConnectedTo(self, l):
        diff, edges = self.getNearestEdgeToEdge(l)
        return diff < .001
    
    def isAlmostConnectedTo(self, l, L):
        # get edges that should be connected
        dist, (selfE, lE) = self.getNearestEdgeToEdge(l)
        if dist > THRESHOLD_ALMOST_CONNECTED:
            return False
        # get index of edge of l
        k_l = 0 if norm(l.E(0) - lE) < .001 else 1
        # check if the nearest edge for self is also nearest for l, else do nothing
        nearestLine_l = l.getNearestLinesWithAdjacentEdges(L)[k_l]
        return self == nearestLine_l

    def getAllLinesWithAdjacentEdges(self, L):
        """ get all lines with adjacent edges:
                it could be that one edge results in multiple adjacent lines
        """
        # filter out the line in the list that is identical to self
        L = [l for l in L if not self == l]
        # get adjacent edges for each edge
        adjL = [[], []]; dists = [[], []]
        for l in L:
            dist, (selfE, lE) = self.getNearestEdgeToEdge(l)
            if dist < THRESHOLD_E:
                index = 0 if norm(selfE - self.E(0)) < .001 else 1
                assert_almost_equal(selfE, self.E(index))
                dists[index].append(dist)
                adjL[index].append(l)
        return dists, adjL
    
    def getNearestLinesWithAdjacentEdges(self, L):
        dists, cands = self.getAllLinesWithAdjacentEdges(L)
        nearestLines = []
        for distsPerEdge, candsPerEdge in zip(dists, cands):
            nearestCand = None
            if candsPerEdge:
                distsPerEdge, candsPerEdge = zip(*sorted(zip(distsPerEdge, candsPerEdge)))
                nearestCand = candsPerEdge[0]
            nearestLines.append(nearestCand)
        return nearestLines

    def connectToNearestLines(self, L):
        nearestLines = self.getNearestLinesWithAdjacentEdges(L)
        for k, (selfE, l) in enumerate(zip(self.E(), nearestLines)):
            """ checks: should self and l be connected? """
            # continue if no matching edge found
            if not l:
                continue
            
            # get edges that should be connected
            dist, (selfE, lE) = self.getNearestEdgeToEdge(l)
            if dist < .001: continue # continue if already connected
            # get index of edge of l
            k_l = 0 if norm(l.E(0) - lE) < .001 else 1
            
            # check if the nearest edge for self is also nearest for l, else do nothing
            nearestLine_l = l.getNearestLinesWithAdjacentEdges(L)[k_l]
            if not self == nearestLine_l:
                continue
            
            """ do connect operation """
            E_new = self.getIntersectionWith(l)
            # only proceed when displacements are small
            if norm(E_new - selfE) > THRESHOLD_E or norm(E_new - lE) > THRESHOLD_E:
                continue
            # update self with new E
            self.updateFromE((E_new, self.E(1-k)))
            # update l with new E
            l.updateFromE((E_new, l.E(1-k_l)))

    def getAllConnectedLinesFrom(self, L):
        nearestLines = self.getNearestLinesWithAdjacentEdges(L)
        connectedLines = []
        for l in nearestLines:
            # continue if no matching edge found
            if not l:
                continue
            # check distance to line
            dist, (selfE, lE) = self.getNearestEdgeToEdge(l)
            if dist < .001:
                connectedLines.append(l)
        return connectedLines
    
    def getAngleWith(self, l):
        dist, (selfE, lE) = self.getNearestEdgeToEdge(l)
        w1 = self.M - selfE
        w2 = l.M - lE
        w1w2 = dot(w1, w2)
        norms = norm(w1) * norm(w2)
        return arccos(w1w2 / norms)

    def getIntersectionWith(self, l):
        Delta = l.M - self.M
        alpha1, alpha2 = self.alpha, l.alpha
        # if parallel --> no solution
        if abs(alpha1-alpha2) < .01:
            return array([float('inf')] * 2)
        # help matrix to solve system
        A = matrix([
            [cos(alpha1), -cos(alpha2)],
            [sin(alpha1), -sin(alpha2)],
        ])
        T = Delta * A.I.T
        T = array(T)[0]
        assert_almost_equal(self.P(T[0]), l.P(T[1]))
        return self.P(T[0])

    @staticmethod
    def linesFormClosedShape(L):
        """ checks if all lines are connected with eachother and they form a closed shape """
        K = len(L)
        l = l0 = L[0]
        seen = []
        while True:
            connectedLines = l.getAllConnectedLinesFrom(L)
            connectedLines = [ll for ll in connectedLines if not ll in seen]
            if l0 in connectedLines:
                if len(seen) == 1:
                    connectedLines = [ll for ll in connectedLines if not ll == l0]
                else:
                    seen.append(l0)
                    break
            if not connectedLines:
                return False
            l = connectedLines[0]
            seen.append(l)
        return len(seen) == K
        




