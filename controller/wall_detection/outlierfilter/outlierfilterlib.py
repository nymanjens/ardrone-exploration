from numpy import array, zeros, pi, unique, sqrt
import numpy as np

def filter_outliers(points, threshold=300, min_cluster_size=30, return_clusters=False):
    N = len(points)
    threshold2 = threshold**2
    
    """ assign cluster id's to every point """
    clusts = zeros(N)
    dists = zeros(N)
    for clust_id in range(N):
        # pick a point P_n
        #n = array([dist if clust == 0 else float('inf') \
        #    for dist, clust in zip(dists, clusts)]).argmin() # doesn't work correctly, but is a lot faster
        n = clust_id
        # get dists from every point to P_n
        dists = ((points - points[n,:])**2).sum(1)
        # for all points already in a cluster: assign found cluster points to this cluster
        for k in unique(clusts[dists < threshold2]):
            if k != 0:
                clusts[clusts == k] = clust_id
        # assign all near points to this cluster
        clusts[dists < threshold2] = clust_id
        
        """ debug: live plot to show how algo works """
        #from pylab import plot, show, Circle, gca, pi, sin, cos, arange, ion, ioff, gcf, clf, axes
        ## divide points in list (mainly for debugging)
        #clusters = []
        #for clust_id in unique(clusts):
        #    if clust_id == 0: continue
        #    clusters.append(points[clusts == clust_id])
        ## do plotting
        #ion()
        #clf()
        #axes().set_aspect('equal')
        #COLORS = "rbkym"
        #plot(points[:, 0], points[:, 1], 'kx')
        #for k, cluster in enumerate(clusters):
        #    color = COLORS[k % len(COLORS)]
        #    plot(cluster[:, 0], cluster[:, 1], color+'o')
        ## circle
        #theta = arange(0,2*pi, .01)
        #xy = (threshold * array([cos(theta), sin(theta)]))
        #plot(points[n,0] + xy[0], points[n,1] + xy[1], 'k-')
        #show()
        #gcf().canvas.draw()
        #ioff()

        
    """ divide points in list (mainly for debugging) """
    clusters = []
    for clust_id in unique(clusts):
        clusters.append(points[clusts == clust_id])
    
    """ debug: output """
    #print "#points:   ", len(points)
    #print "#clusters: ", len(unique(clusts))
    
    """ debug: plots """
    #from pylab import plot, show, axes, legend, xlabel, ylabel
    #COLORS = "rybmk"
    #COLORS2 = "kyrg"
    #axes().set_aspect('equal')
    #for k, cluster in enumerate([c for c in clusters if len(c) > min_cluster_size]):
    #    color = COLORS[k % len(COLORS)]
    #    p1 = plot(cluster[:, 0]/1e3, cluster[:, 1]/1e3, color+'.')
    #for k, cluster in enumerate([c for c in clusters if len(c) <= min_cluster_size]):
    #    color = COLORS2[k % len(COLORS2)]
    #    p2 = plot(cluster[:, 0]/1e3, cluster[:, 1]/1e3, color+'x')
    #legend([p1, p2], ['accepted', 'rejected'], loc='upper left')
    #xlabel('x (m)')
    #ylabel('y (m)')
    #show()
    
    """ do actual filtering (with min_cluster_size) """
    accepted = []
    for cluster in clusters:
        if len(cluster) > min_cluster_size:
            accepted += cluster.tolist()
    accepted = array(accepted)
    
    return accepted if not return_clusters else [c for c in clusters if len(c) > min_cluster_size]









