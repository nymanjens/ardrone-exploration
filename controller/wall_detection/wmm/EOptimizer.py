import numpy as np
from numpy import array, ones, zeros, arange, exp, log, pi, sqrt, hstack, vstack, diag, matrix, cos, sin, tan, arctan, isnan, atleast_2d
from numpy.linalg import norm, eigh

#counter = 0

class EOptimizer:
    """ optimizes e for one value of k """
    # external vars
    L = gamma = pi_k = N_k = points = pi_times_prob = None
    # state vars
    k = None
    # calculated vars
    trPoints = None
    
    def __init__(self, L, gamma, pi_k, N_k, points, pi_times_prob):
        self.L, self.gamma, self.pi_k, self.N_k, self.points, self.pi_times_prob = L, gamma, pi_k, N_k, points, pi_times_prob
    
    def setK(self, k):
        self.k = k
        self.trPoints = self.L[k].normalizeCoords(self.points)
    
    def optimize(self):
        k = self.k; l = self.L[k]; x,y = self.trPoints.T; gamma_k = self.gamma[:, k];
        sigma = l.sigma; N = self.N_k[k]
        # set e start value
        e = l.e
        
        """ tmp """
        #prev_e = e
        """ iterative algorithm (not necessary) """
        #prev_e = float('nan')
        #while prev_e != e:
        #    prev_e = e
        
        # calculate edge points
        S_E = arange(len(x))[abs(x) > e]
        x_E = x[S_E]
        gamma_E = gamma_k[S_E]
        """ get ideal e from given set of edge points """
        mu_E = sum(gamma_E * abs(x_E))
        N_E = sum(gamma_E)
        # quadratic equation: Ax^2 + Bx + C
        A = N_E / sigma**2
        B = sqrt(pi/2)*N_E/sigma - mu_E/sigma**2
        C = N - sqrt(pi/2)*mu_E/sigma
        Delta = B**2 - 4*A*C
        if Delta > 0:
            e = (-B + sqrt(Delta)) / (2*A)

        """ working debug plots """
        #global counter
        #if k == 0:
        #    counter += 1
        #if k == 0 and counter > 0 and not np.isnan(l.sigma):
        #    from mdp.utils import progressinfo
        #    import pylab as pl
        #    from pprint import pprint
        #    ees = np.arange(-sqrt(2/pi)*sigma, 2*e, 3*e/1000.)
        #    lnps = []
        #    gammalnps = []
        #    gammalnps_hat = []
        #    for ee in progressinfo(ees):
        #        l.e = ee
        #        #lnps.append(sum(np.log(sum(
        #        #    pi_kk * ll.Prob(p, normalized=True) for pi_kk in self.pi_k
        #        #)) for p, ll in zip(self.trPoints, self.L)))
        #        ##Pr = sum(np.log(self.pi_k[k] * l.Prob(p, normalized=True)) for p in self.trPoints)
        #        #gammalnps.append(
        #        #    sum(g*np.log(l.Prob(p, normalized=True)) for p,g in zip(self.trPoints,gamma_k))
        #        #)
        #        gammalnps_hat.append(
        #            -1/(ee+sqrt(pi/2)*sigma)*N + mu_E/sigma**2 - ee*N_E/sigma**2
        #        )
        #    gammalnps_hat = array(gammalnps_hat)
        #    pl.ioff(); pl.figure()
        #    # plot here
        #    #pl.plot(ees/1e3, lnps, '.-')
        #    #pl.plot(ees/1e3, gammalnps, '.-k')
        #    pl.axhline(0, ls='-', color='k')
        #    p1 = pl.plot(ees/1e3, gammalnps_hat*1e3, '-r')
        #    p2 = pl.axvline(e/1e3, ls='--')
        #    pl.axvline(((-B - sqrt(Delta)) / (2*A))/1e3, ls='--')
        #    p4 = pl.axvline(prev_e/1e3, ls=':')
        #    #p5 = pointplot = pl.plot(abs(x)/1e3, 0*np.ones(len(x)), '.g')
        #    # calc lims
        #    vals = gammalnps_hat[ees>0]*1e3
        #    pl.ylim((vals.min(),vals.max()+100))
        #    # other plot things
        #    pl.xlabel('e (m)')
        #    pl.ylabel('[d/de ln Pr(X|L)]_simplified   (1/m)')
        #    pl.legend([p1,p2,p4], ['[d/de ln Pr(X|L)]_simpl', 'solutions for e', 'old e'], loc='best')
        #    # plot end
        #    pl.show()
        #    l.e = e
        #    #exit()
        
        """ debug output """
        #print "            e = ", e

        """ old (not working) debug plots """
        ## get Prob
        #from mdp.utils import progressinfo
        #sigmax_times_1_732 = L.e # (only in first iteration)
        #bivariateNormPdf = lambda x: 1/(2*pi*sigma**2) * exp(-x**2 / (2*sigma**2))
        #k = self.k; l = self.L[k]; pi_ = self.pi_k[k]; residualProbTerm = self.residualProbTerm
        #e_s = arange(0, e2 + sigmax_times_1_732*4, sigmax_times_1_732/5)
        #logProbs = []
        #for e in progressinfo(e_s):
        #    totalLogProb = 0
        #    for n, point in enumerate(self.trPoints):
        #        # alpha (scaling param)
        #        alpha = 1 / (1 + sqrt(2/pi)*e/sigma)
        #        # probablility
        #        if n in S_E:
        #            prob = alpha * bivariateNormPdf(norm([abs(x[n])-e, y[n]]))
        #        else:
        #            prob = alpha * bivariateNormPdf(y[n])
        #        totalLogProb += log(residualProbTerm[n] + pi_ * prob)
        #    logProbs.append(totalLogProb)
        #logProbs = array(logProbs)
        ## get diff Prob
        #dProb = -1/(e_s + sqrt(pi/2)*sigma)*N
        #for gamma_ik, x_ix in zip(gamma_E, x_E):
        #    dProb += gamma_ik * (abs(x_ix) - e_s) / sigma**2
        ## get vkv
        #vkv = 1/(e_s + sqrt(pi/2)*sigma)*N - mu_E/sigma**2 + e_s*N_E/sigma**2
        #vkv2 = A*e_s**2 + B*e_s + C
        ## plots - minimization
        #ioff()
        #figure()
        #plot(e_s/sigmax_times_1_732, (logProbs - logProbs.mean())/logProbs.std())
        ##plot(e_s/sigmax_times_1_732, dProb/dProb.std())
        ##plot(e_s/sigmax_times_1_732, vkv/vkv.std())
        #plot(e_s/sigmax_times_1_732, vkv2/vkv2.std())
        #axvline(e1/sigmax_times_1_732)
        #axvline(e2/sigmax_times_1_732)
        #axhline(0)
        #show()
        ## plots - points
        #ioff()
        ##figure()
        ##l.plot(pl)
        ##points = self.points
        ##for i, point in enumerate(points):
        ##    if i in S_E:
        ##        plot(point[0], point[1], 'gx')
        ##    else:
        ##        plot(point[0], point[1], 'b.')
        ##show()
        #exit()
        
        return e

    """ tmp debugfunction """
    #def totalLogProb(self, e):
    #    k = self.k; l = self.L[k]; pi_ = self.pi_k[k]
    #    # calc residualProbTerm, constant terms in calc of total probablility
    #    residualProbTerm = np.delete(self.pi_times_prob, k, 1).sum(1)
    #    l.e = e
    #    totalLogProb = 0
    #    for n, point in enumerate(self.trPoints):
    #        totalLogProb += log(residualProbTerm[n] + pi_ * l.Prob(point, normalized=True))
    #    return totalLogProb
    



