#!/usr/bin/python
# std imports
import numpy as np
from math import cos, sin, acos
from numpy import array, pi, matrix
from numpy.linalg import eigh, norm
import pickle
from time import time
# ros imports
if __name__ == "__main__":
    import roslib; roslib.load_manifest('py_shared')
import rospy
from std_msgs.msg import String
from  py_shared.srv import *
# custom imports
from utility import *
from floorseparator import is_floor

def rotation_from_angle_axis(angle, axis, point=None):
    """Return matrix to rotate about axis defined by point and axis."""
    sina = sin(angle)
    cosa = cos(angle)
    axis /= norm(axis)
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(axis, axis) * (1.0 - cosa)
    axis *= sina
    R += array([
        [ 0.0,    -axis[2], axis[1]],
        [ axis[2], 0.0,    -axis[0]],
        [-axis[1], axis[0], 0.0    ]
    ])
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = array(point[:3], dtype=np.float64, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return matrix(M)

class XYTransformation:
    T = None # transformation matrix
    
    def __init__(self, T = None):
        self.T = T if T else matrix(np.identity(4))
        #self.T = T if T else self.getMatrix(array([-400,0,1500]), array([ # TMP ad hoc
        #    [1./3,1,0],
        #    [0,0,1],
        #]))
    
    def updateFromPointCloud(self, points):
        """ update current state from new point cloud """
        # filter away non-floor based on previous transform
        old_principal_transformed_points = points = self.doTransform(points)
        points = array([pt for i, pt in enumerate(points) if is_floor(points[i,:])])
        # filter away outliers
        outlier_filtered_points = points = array([pt for pt in points if norm(pt) < 5e3])
        # take only points near mean
        mean = points.mean(0)
        points = array([pt for pt in points if norm(pt - mean) < 1e3])
        
        # sanity checks
        if len(old_principal_transformed_points) < 20: # = num points before outlier filter
            rospy.logwarn("Warning: too little #points")
            return
        elif len(outlier_filtered_points) < 20: # = num points after outlier filter
            rospy.logwarn("Warning: mean of points too large")
            return
        elif len(points) < 20: # = num points near mean
            rospy.logwarn("Warning: too little #points near mean")
            return
        # perform PCA
        mean, eigs = self.performPCA(points)
        if not eigs: return
        # get transform-matrix from eigenvalues
        self.T = self.getMatrix(mean, eigs) * self.T
    
    @staticmethod
    def performPCA(A):
        """ perform PCA """
        # set zero mean
        mean = A.mean(0)
        A = A - np.tile(mean, (A.shape[0], 1)) # not anymore
        # convert to point list
        X = (np.atleast_2d(A[i,:].T) for i in range(A.shape[0]))
        # PCA analysis
        E = np.sum(x*x.T for x in X)
        w, v = eigh(E)
        v = array(v)
        eigs = v[:, 0], v[:, 1], v[:, 2]
        # sort according to eigenvalues
        eigs = zip(*sorted(zip(w, eigs), reverse=True))[1]
        return mean, eigs
    
    @staticmethod
    def getMatrix(mean, eigs):
        """ get transformation to XY from eigenvectors """
        # get normal pointing out of plane
        n = np.cross(eigs[0], eigs[1])
        n /= norm(n)
        n = -n if n[2] < 0 else n # ensure normal points up
        # get rotation axis to get n -> z
        z = array([0,0,1])
        r = np.cross(n, z)
        r /= norm(r)
        # calculate required rotation
        theta = acos(np.dot(n, z))
        # get rotation matrix around mean
        T = rotation_from_angle_axis(theta, r, mean)
        # translate plane to z=0
        T[2, 3] -= mean[2]
        return T
    
    def toString(self):
        return pickle.dumps(self.T.tolist())
    
    def setFromString(self, s):
        self.T = matrix(pickle.loads(s))
    
    def doTransform(self, A):
        """ perform this transform (A = list of points) """
        # convert to matrix (each row is a point)
        A = array(A)
        A = np.vstack([A.T, np.ones((1,A.shape[0]))])
        # do actual transformation
        return array(self.T * A).T[:,:3]

class Principalplane:
    transformation = None
    lastupdate = None
    
    def __init__(self, transformation=None):
        self.transformation = transformation if transformation else XYTransformation()
    
    """ listener """
    def updateParams(self, msg):
        return # TMP
        points = decode_string_to_array(msg)
        self.transformation.updateFromPointCloud(points)
    
    """ server method """
    def getParams(self, req=None):
        """ [remote method] """
        return self.transformation.toString()
    
    """ client methods """
    def updateTransformation(self):
        # check if last update was not too near now
        if self.lastupdate and time() - self.lastupdate < 2:
            return
        # do update
        try:
            get_principalplane_params = rospy.ServiceProxy('get_principalplane_params', getPrincipalplaneParams)
            s = get_principalplane_params().params
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: {0}".format(e))
        else:
            self.transformation.setFromString(s)
            self.lastupdate = time()

    def updateAndTransform(self, *pointslist, **kwargs):
        # update
        if not 'update' in kwargs or kwargs['update']:
            self.updateTransformation()
        # transform
        results = []
        for points in pointslist:
            if not isinstance(points, dict):
                points = np.atleast_2d(array(points))
                results.append(self.transformation.doTransform(points).squeeze())
            else: # special case: hybriddata
                hybriddata = points
                point = [hybriddata[key] for key in ('x', 'y', 'h')]
                point = self.updateAndTransform(point, update=False)
                for i, key in enumerate(('x', 'y', 'h')):
                    hybriddata[key] = point[i]
                results.append(hybriddata)
        return results if len(results) > 1 else results[0]


if __name__ == "__main__":
    # initialization
    rospy.init_node('principalplane_server')
    principalplane = Principalplane()
    # services
    s = rospy.Service('get_principalplane_params', getPrincipalplaneParams, principalplane.getParams)
    # subscriptions
    rospy.Subscriber("/map/points", String, principalplane.updateParams)
    # keep alive
    rospy.spin()
    print "\n"

