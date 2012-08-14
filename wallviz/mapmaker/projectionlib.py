from __future__ import division
import Image, sys, random
from numpy import arange, array, dot, pi, average, logical_and, sqrt, tanh
from numpy.linalg import norm, inv
import numpy as np
try: from mdp.utils import progressinfo
except: progressinfo = lambda(x): x; print "  Note: mdp.utils.progressinfo not loaded (only for nicer output)"
import concave

#WALL_HEIGHT = 2.5 #m # 3rooms_fish
WALL_HEIGHT = 1.55 #m # 3rooms
PIXELS_PER_M = 50
DEBUGPLOT = not '-d' in sys.argv

""" debugger """
class DebuggerPlot:
    implot = None
    ax = None
    @classmethod
    def plot(Self, projectedImage):
        if DEBUGPLOT:
            import pylab as pl
            img = array(projectedImage.image, dtype=np.float32) / 255.
            if not Self.ax:
                pl.ion()
                Self.ax = pl.figure().gca()
            if Self.implot and Self.implot.get_size() == img.shape:
                Self.implot.set_data(img)
                pl.draw()
            else:
                pl.clf()
                Self.implot = pl.imshow(img, cmap=pl.cm.gray)
                pl.draw()
    
    @classmethod
    def close(Self):
        if DEBUGPLOT:
            import pylab as pl
            pl.close()

""" TMP """
def dot_on_img(img, P):
    (i,j) = np.int_(P)
    for ii in range(i-2, i+3):
        for jj in range(j-2, j+3):
            if 0 <= ii < len(img) and 0 <= jj < len(img[0]):
                img[ii,jj] = 0
    for ii in range(i-1, i+2):
        for jj in range(j-1, j+2):
            if 0 <= ii < len(img) and 0 <= jj < len(img[0]):
                img[ii,jj] = 255
    img[i,j] = 0
def cross_on_img(img, P):
    (i,j) = np.int_(P)
    N = 6
    for ii in range(i-N, i+N+1):
        for jj in range(j-N, j+N+1):
            if 0 <= ii < len(img) and 0 <= jj < len(img[0]):
                img[ii,jj] = 255
    for ii in range(i-N, i+N+1):
        for jj in (-(ii-i)+j, ii-i+j):
            if 0 <= ii < len(img) and 0 <= jj < len(img[0]):
                img[ii,jj] = 0

""" projected image (base class) """
class ProjectedImage:
    WH = None # width and height in meter
    MN = None # M = height, N = width (in pixels)
    image = None
    fillableArea = None
    lowestCostPerPixel = None
    
    def __init__(self, W, H, do_precaching=True):
        self.WH = [W,H]
        self.MN = M,N = [int(round(x)) for x in PIXELS_PER_M * array([H,W])]
        self.image = np.zeros((M,N,3), dtype=np.uint8)
        if do_precaching:
            self._precache()
    
    def _precache(self):
        self.createFillableArea()
        self.lowestCostPerPixel = float('inf') * np.ones(len(self.fillableArea))
    
    def projectKeyframe(self, kfimg, projections):
        K = len(projections)
        if K < 6:
            return
        
        ### get weight matrix ###
        #R = np.ones((K,1)) # unweighted linear regression
        R = array([[tanh(0.1/projerr)] for (_,_,projerr,_) in projections]) # weighted linear regression
        
        ### perform linear regression ###
        X = array([[Pw[0], Pw[1], 1] for Pw, _, _, _ in projections])
        T = array([Pkf for _, Pkf, _, _ in projections])
        XTR = (R*X).T
        W = dot(inv(dot(XTR, X)), dot(XTR, T))
        
        ### get train error ###
        sqerr = ( R * (dot(X, W) - T)**2 ).sum()
        
        ### frame cost ###
        projcost = sqerr / (array(kfimg.shape).prod() * sum(R))
        anglecost = 1/pi*abs(average([angle for (_,_,_,angle) in projections]))
        framecost = 5000*projcost + 32*anglecost
        
        ### projection pre-calculations ###
        kfM, kfN = kfimg.shape[:2]
        # get area to fill (based on lowest cost)
        areaToFillBools = self.lowestCostPerPixel > framecost
        areaToFill = self.fillableArea[areaToFillBools]
        arrinds = arange(len(self.lowestCostPerPixel))[areaToFillBools]
        # unpack W
        Transf = W.T[:, :2]
        Transl = W.T[:,  2]
        
        ### TMP verify train error ###
        #errs = Transl + dot([Pw for Pw, _, _, _ in projections], Transf.T) - [Pkf for _, Pkf, _, _ in projections]
        #sqerr2 = ( R * errs**2 ).sum()
        #np.testing.assert_almost_equal(sqerr, sqerr2)
        
        ### calculatios for every point ###
        # transform indices
        kfindices = Transl + dot(areaToFill, Transf.T)
        # round
        kfindices = np.int_(kfindices.round())
        # check index bounds
        i_s, j_s = kfindices.T
        i_valid = logical_and(0 <= i_s, i_s < kfM)
        j_valid = logical_and(0 <= j_s, j_s < kfN)
        valid = logical_and(i_valid, j_valid)
        # zero check to avoid erors
        if sum(valid) == 0: return
        # transform pixels
        self.image[areaToFill[valid].T.tolist()] = kfimg[kfindices[valid].T.tolist()]
        # save framecost
        self.lowestCostPerPixel[arrinds[valid]] = framecost
        
        ### debug output ###
        DebuggerPlot.plot(self)
    
    def toImage(self):
        """ from 2D array to Image """
        return Image.fromarray(self.image)
    
    def createFillableArea(self):
        raise NotImplementedError()

class WallImage(ProjectedImage):
    E = None
    
    def __init__(self, E1, E2):
        self.E = [E1, E2]
        W = norm(E2-E1)
        ProjectedImage.__init__(self, W, WALL_HEIGHT)
    
    def createFillableArea(self):
        M,N = self.MN
        self.fillableArea = []
        for i in progressinfo(range(M)):
            for j in range(N):
                self.fillableArea.append(array([i,j]))
        self.fillableArea = array(self.fillableArea, dtype=np.int32)
    
    def indicesFromLambdaZ(self, lambd, z):
        M,N = self.MN
        W,H = self.WH
        i = (1 - z/H) * M
        j = N * lambd
        return array([i,j])
    
    def angleWithNormal(self, Pc, Pp):
        E0, E1 = self.E
        e = E1 - E0
        e /= norm(e)
        n = [e[1], -e[0], 0]
        v = Pc - Pp
        v /= norm(v)
        return np.arccos(dot(v,n))

    def projectKeyframes(self, keyframes):
        E0, E1 = self.E
        for kf in progressinfo(keyframes.values()):
            Pc = kf['pos']
            # get 2 projected points on wall from this keyframe
            projections = []
            for pt in kf['points']:
                if int(pt['level']) > 0:
                    continue
                Pm = pt['pos']
                Pp, props = self.projectPointToWall(Pc, Pm)
                if not (props['forward_proj'] and props['on_wall']):
                    continue
                # get indices on wallimg and kfimg
                wallind = self.indicesFromLambdaZ(props['lambda_wall'], Pp[2])
                kfind = pt['imgpos']
                # get extra parameters that can serve as cost
                projcost = abs(1-props['lambda_line'])
                angle = self.angleWithNormal(Pc, Pp)
                if abs(angle) > np.pi/2:
                    #print "warning: keyframe behind wall"
                    continue
                projections.append([wallind, kfind, projcost, angle])
            # get keyframe data
            kfimg = Image.open(kf['img'])
            kfimg = array(kfimg) # from Image to 2D array
            # project keyframe on wall
            self.projectKeyframe(kfimg, projections)
    
    def projectPointToWall(self, Pc, Pm):
        """ get crossing point [x,y] between E1E2 and PcPm """
        E1, E2 = self.E
        a1, m1 = self.a_and_m(Pc, Pm)
        a2, m2 = self.a_and_m(E1, E2)
        x = (a1-a2)/(m2-m1)
        lambda_line = (x-Pc[0])/(Pm[0]-Pc[0])
        lambda_wall = (x-E1[0])/(E2[0]-E1[0])
        Pp = Pc + lambda_line * (Pm - Pc)
        # return projected point and properties
        return Pp, {
            'lambda_line': lambda_line,
            'lambda_wall': lambda_wall,
            'forward_proj': lambda_line > 0,
            'on_wall': 0 < lambda_wall < 1 and 0 < Pp[2] < WALL_HEIGHT,
        }
    
    @staticmethod
    def a_and_m(P1, P2):
        """ helpfunciton for projectPointToWall """
        x1, y1 = P1[:2]
        x2, y2 = P2[:2]
        m = (y2-y1)/(x2-x1)
        a = y1 - m*x1
        return a,m


class FloorImage(ProjectedImage):
    F = None
    walls = None
    walls_normals = None
    walls_indices = None
    walls_normalindices = None
    
    def __init__(self, walls):
        self.walls = walls
        ### calculate F from walls ###
        self.F = F0,F1 = self.getFFromWalls(walls)
        # get W an H
        W, H = F1 - F0
        ### call superclass ###
        ProjectedImage.__init__(self, W, H, do_precaching=False)
        ### caching of wall normals ###
        self.walls_normals = []
        self.walls_indices = []
        self.walls_normalindices = []
        for E0, E1 in walls:
            # normals in 2D point space
            e = E1 - E0
            e /= norm(e)
            n = array([e[1], -e[0]])
            self.walls_normals.append(n)
            # normals in index space
            E0i = self.indicesFromP(E0)
            E1i = self.indicesFromP(E1)
            e = E1i - E0i
            e /= norm(e)
            n = array([e[1], -e[0]])
            self.walls_indices.append([E0i, E1i])
            self.walls_normalindices.append(n)
        ### call precaching ###
        self._precache()
        ### set white floor background ###
        for Pw in self.fillableArea:
            self.image[Pw[0], Pw[1]] = [255, 255, 255]
    
    @staticmethod
    def getFFromWalls(walls):
        # get all wall edges
        edges = []; [[edges.append(xx) for xx in x] for x in walls]; edges = array(edges)
        # get F
        F0 = edges.min(0)
        F1 = edges.max(0)
        return array([F0, F1])
    
    def createFillableArea(self):
        fillableArea = concave.create_fillable_area(self.walls_indices, self.MN)
        self.fillableArea = array(fillableArea, dtype=np.int32)
    
    def indicesFromP(self, P):
        M,N = self.MN
        W,H = self.WH
        F0,F1 = self.F
        j,Mmini = array([N/W, M/H]) * (P[:2] - F0[:2])
        return array([M - Mmini, j])
    
    def pointBetweenWalls(self, P):
        P = P[:2]
        return concave.point_inside_area(self.walls, P)
    
    def indicesBetweenWalls(self, ij):
        return concave.point_inside_area(self.walls_indices, ij)
    
    def angleWithNormal(self, Pc, Pp):
        n = [0, 0, 1]
        v = Pc - Pp
        v /= norm(v)
        return np.arccos(dot(v,n))

    def projectKeyframes(self, keyframes):
        for kf in progressinfo(keyframes.values()):
            Pc = kf['pos']
            # get 2 projected points on floor from this keyframe
            projections = []
            for pt in kf['points']:
                if int(pt['level']) > 0:
                    continue
                Pm = pt['pos']
                Pp, props = self.projectPointToFloor(Pc, Pm)
                if not (props['forward_proj'] and props['between_walls']):
                    continue
                floorind = self.indicesFromP(Pp)
                kfind = pt['imgpos']
                angle = self.angleWithNormal(Pc, Pp)
                #assert abs(angle) < np.pi/2, "keyframe under floor"
                projections.append([floorind, kfind, abs(1-props['lambda_line']), angle])
            # get keyframe data
            kfimg = Image.open(kf['img'])
            kfimg = array(kfimg) # from Image to 2D array
            # project keyframe on floor
            self.projectKeyframe(kfimg, projections)
    
    def projectPointToFloor(self, Pc, Pm):
        lambda_line = (Pc[2])/(Pc[2]-Pm[2])
        Pp = Pc + lambda_line * (Pm - Pc)
        # return projected point and properties
        return Pp, {
            'lambda_line': lambda_line,
            'forward_proj': lambda_line > 0,
            'between_walls': self.pointBetweenWalls(Pp),
        }
    
    def toImage(self):
        """ from 2D array to Image """
        h,w = self.image.shape[:2]
        alpha_img = np.zeros((h,w,4), dtype=np.uint8)
        for c, (alpha_line, line) in enumerate(zip(alpha_img, self.image)):
            for r, pix in enumerate(line):
                alpha_line[r,:3] = pix
                alpha_line[r,3] = 255 if self.indicesBetweenWalls((c,r)) else 0
        return Image.fromarray(alpha_img, 'RGBA')
        

