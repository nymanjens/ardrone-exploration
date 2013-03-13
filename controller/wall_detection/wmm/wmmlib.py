""" wall mixture model library """
""" std imports """
from numpy import array, ones, zeros, arange, exp, log, pi, sqrt, hstack, vstack, diag, matrix, cos, sin, tan, arctan, isnan, atleast_2d
from numpy.linalg import norm, eigh
"""  custom imports """
import pypr.clustering.gmm as gmm # download from http://pypr.sourceforge.net/
from Line import Line, THRESHOLD_E
from EOptimizer import EOptimizer

def EM_wall_mixture_model(points, K = 3, MAX_ITER = 30, init_from_gmm=True, force_connected=True, debug_output = None):
    """ initialization """
    N = points.shape[0]
    # init lines
    L = []
    if init_from_gmm:
        cen_lst, cov_lst, p_k, logL = gmm.em_gm(points, K = K)
        for cen, cov in zip(cen_lst, cov_lst):
            L.append(Line.fromEllipse(cen, cov))
    else:
        for k in range(K):
            L.append(Line.getRandom(points))
    # init pi
    pi_k = 1./K * ones(K)
    
    """ run algorithm """
    oldgamma = zeros((N, K))
    for iter_num in range(MAX_ITER):
        """ E-step """
        pi_times_prob = zeros((N, K))
        gamma = zeros((N, K))
        for n in range(N):
            for k in range(K):
                pi_times_prob[n,k] = pi_k[k] * L[k].Prob(points[n])
            # normalized version of pi_times_prob is gamma
            gamma[n,:] = pi_times_prob[n,:] / pi_times_prob[n,:].sum()

        """ debug output """
        if debug_output:
            debug_output(L, gamma, iter_num)
        
        """ M-step (general) """
        N_k = gamma.sum(0)
        pi_k = N_k / N_k.sum()
        
        """ M-step (gaussian mixture with inf primary eigenval) """
        eOptimizer = EOptimizer(L, gamma, pi_k, N_k, points, pi_times_prob)
        for k in range(K):
            """ get mean """
            mu = 1/N_k[k] * sum(gamma[n,k]*points[n] for n in range(N))
            """ get cov matrix """
            x = [array([points[n]-mu]).T for n in range(N)]
            cov = 1/N_k[k] * sum(gamma[n,k]*x[n]*x[n].T for n in range(N))
            # re-initilize M, alpha and sigma from ellipse
            L[k].updateFromEllipse(mu, cov)
            """ get e """
            eOptimizer.setK(k)
            L[k].e = eOptimizer.optimize()
        """ force that all lines are connected """
        for l in L:
            l.connectToNearestLines(L)
        
        """ check sanity of lines """
        for k in range(K):
            if L[k].inBadCondition():
                #print "L[%d] in bad condition" % k
                L[k] = Line.getRandom(points)
            for kk in range(k):
                if L[k].almostEqualTo(L[kk]):
                    #print "L[%d] almost equal to L[%d]" % (k, kk)
                    L[k] = Line.getRandom(points)
                    break
        
        """ remove 2 lines that are on same real line """
        DTHETA = 20 * pi/180 # margin
        for k, l in enumerate(L):
            for kk, line in ([kk, ll] for kk, ll in enumerate(L) if k != kk):
                if l.isConnectedTo(line) or l.isAlmostConnectedTo(line, L):
                    theta = l.getAngleWith(line)
                    if pi/2 - abs(theta - pi/2) < DTHETA:
                        # remove line with least responsibility
                        del_k = k if N_k[kk] > N_k[k] else kk
                        L[del_k] = Line.getRandom(points)
        
        """ remove crossing lines """
        for k, l in enumerate(L):
            for kk, line in ([kk, ll] for kk, ll in enumerate(L) if k != kk):
                # if l and line are connected, they can't cross
                if l.isConnectedTo(line):
                    continue
                X = l.getIntersectionWith(line)
                # if X lies on line and l, this is an intersection
                if line.hasPoint(X) and l.hasPoint(X):
                    # remove line with least responsibility
                    del_k = k if N_k[kk] > N_k[k] else kk
                    L[del_k] = Line.getRandom(points)
        
        """ check stop cond """
        if norm(oldgamma - gamma) < .05:
            break
        oldgamma = gamma
    
    """ debug output """
    if debug_output:
        debug_output(L, gamma)
    
    """ calc probablility P[{Lk} | X] ~ P[X | {Lk}] * P[{Lk}] """
    totalLogProb = sum(log(pi_times_prob.sum(1)))
    logProbLk = 0
    anglecounter = 0
    ## add prob of theta ~ N(pi/2, sigma_theta)
    sigma_theta = .01 * pi/2
    for k, l in enumerate(L):
        lines = l.getAllConnectedLinesFrom(L[k+1:])
        for line in lines:
            theta = l.getAngleWith(line)
            dtheta = abs(theta - pi/2)
            logProbLk += - dtheta**2 / (2*sigma_theta**2) - log(sqrt(2*pi)*sigma_theta)
            anglecounter += 1
    logProbLk /= anglecounter if anglecounter else 1
    ## in case of crossing: Prob = 0
    for k, l in enumerate(L):
        for line in (ll for kk, ll in enumerate(L) if k != kk):
            X = l.getIntersectionWith(line)
            # if l and line are connected, no extra prob is needed
            if l.isConnectedTo(line):
                continue
            # if X lies on line and l, this is an intersection
            if line.hasPoint(X) and l.hasPoint(X):
                logProbLk += float('-inf')
                break
    ## add prob for unattached but near line (extension has to cross)
    ##         ~ N(THRESHOLD_E/d | 0, sigma_unatt) * N(theta | pi/2, sigma_theta)
    sigma_unatt = 0.05
    for k, l in enumerate(L):
        for line in (ll for kk, ll in enumerate(L) if k != kk):
            # if l and line are connected, no extra prob is needed
            if l.isConnectedTo(line):
                continue
            for E in line.E():
                # if E is near l, add probabilities as described above
                d = l.diff(E) + .001 # d should never be zero
                if d < THRESHOLD_E:
                    logProbLk += - (THRESHOLD_E / d)**2 / (2 * sigma_unatt**2)
                    theta = l.getAngleWith(line)
                    logProbLk += - (theta - pi/2)**2 / (2*sigma_theta**2)
                    break
    
    logProbLkX = totalLogProb + logProbLk
    
    return L, logProbLkX





