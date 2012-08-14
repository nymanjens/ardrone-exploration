""" imports """
# std imports
from numpy import array, cross, dot
from numpy.linalg import norm
import numpy as np
# ROS imports
import roslib; roslib.load_manifest('wallviz')
# custom imports
from utility import Minimizer

class MultimapTransform:
    def __init__(self, wall_width, positions, walls_list):
        # project door positions to walls
        relevant_walls = []
        for i, (P, walls) in enumerate(zip(positions, walls_list)):
            Pa, wall = self.projectPosToWalls(P, walls)
            positions[i] = Pa
            relevant_walls.append(wall)
        # get transform parameters
        P0, P1 = positions
        n_s = []
        for E0,E1 in relevant_walls:
            e = E1 - E0
            e /= norm(e)
            n_s.append([-e[1], e[0]])
        n0, n1 = array(n_s)
        n1a = -n0
        # get transform
        costheta = dot(n1, n1a)
        sintheta = cross(n1, n1a)
        R = array([
            [costheta, -sintheta],
            [sintheta, costheta],
        ])
        Q1 = np.identity(3); Q2 = np.identity(3); Q3 = np.identity(3)
        Q1[:2,2] = -P1
        Q2[:2,:2] = R
        Q3[:2,2] = P0 + (wall_width+.001)*n0
        self.Q = dot(dot(Q3, Q2), Q1)
    
    def transform(self, P):
        """ transform point or list of points """
        if len(P.shape) == 2:
            Pa = np.zeros(P.shape)
            for i, p in enumerate(P):
                Pa[i] = self.transform(p)
            return Pa
        elif P.shape[0] == 3:
            x, y, z = P
            x, y = self.transform(array([x,y]))
            return array([x,y,z])
        else:
            P = np.hstack((P,1))
            Pa = dot(self.Q, P)
            return Pa[:2]
    
    @classmethod
    def projectPosToWalls(Self, P, walls):
        minimizer = Minimizer()
        for wall in walls:
            Pa = Self.projectPosToWall(P, wall)
            dist = norm(Pa - P)
            minimizer.add((Pa,wall), dist)
        return minimizer.argmin
    
    @staticmethod
    def projectPosToWall(P, (E0, E1)):
        e = E1-E0
        e /= norm(e)
        v = P - E0
        Pa = E0 + dot(e, v) * e
        return Pa
