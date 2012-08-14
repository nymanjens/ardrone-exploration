""" imports """
# std imports
from __future__ import division
import os, pickle, sys
import xml.parsers.expat
from numpy import array, zeros, matrix, sqrt, sin, cos, cross, dot
from numpy.linalg import norm
from numpy.testing import assert_almost_equal
# ROS imports
import roslib; roslib.load_manifest('wallviz')
from std_msgs.msg import String
# custom imports
from principalplane import Principalplane
from utility import *
import concave
sys.path.append('..')
from undistorter import undistortionlib

""" settings """
MAPDIR = (os.path.dirname(__file__) or '.') + '/../map'
def mapdir(room_num=None):
    return MAPDIR + '/wallvizmap{}'.format(room_num) if room_num != None else MAPDIR

""" PointParser """
class PointParser:
    slamScaleToMetricScale = None
    principalplane = None
    
    def __init__(self, room_num):
        """ get slam-to-metric scale """
        self.slamScaleToMetricScale = float(open(mapdir(room_num)+'/slamtometric').readline())
        self.slamScaleToMetricScale /= Scale.get(room_num) # apply scaling
        """ recover principalplane """
        tfstring = open(mapdir(room_num)+'/principalplane').read()
        self.principalplane = Principalplane()
        self.principalplane.transformation.setFromString(tfstring)
    
    def parse(self, pt, fromstring=False, frompose=False):
        """ step 0: convert to point """
        if fromstring:
            pt = self.stringToArray(pt)
        if frompose:
            pt = self.lnToTranslation(pt)
        """ step 1: convert from PTAM to ROS axes """
        pt = array([
            pt[1] / self.slamScaleToMetricScale,
            -pt[0] / self.slamScaleToMetricScale,
            pt[2] / self.slamScaleToMetricScale,
        ])
        """ step 2: transform via principalplane """
        pt = self.principalplane.updateAndTransform(pt, update=False)
        pt[2] += 200 # TMP (ad hoc)
        """ step 3: convert mm to m """
        pt /= 1e3
        """ step 3: remove if outlier """
        if norm(pt) > 20:
            return None
        return pt
    
    @staticmethod
    def stringToArray(string):
        return array([float(x) for x in string.strip().split(' ')])
    
    @classmethod
    def lnToTranslation(Self, mu):
        mu = array(mu)
        w = mu[3:]
        theta_sq = sum(w*w)
        theta = sqrt(theta_sq)
        w_x_mu = cross(w, mu[:3])
        if theta_sq < 1e-8:
            A = 1.0 - theta_sq/6
            B = 0.5
            translation = mu[:3] + 0.5 * w_x_mu
        else:
            if theta_sq < 1e-6:
                C = (1.0 - theta_sq/20) / 6
                A = 1.0 - theta_sq * C
                B = 0.5 - 0.25 * one_6th * theta_sq
            else:
                inv_theta = 1.0/theta
                A = sin(theta) * inv_theta
                B = (1 - cos(theta)) * (inv_theta * inv_theta)
                C = (1 - A) * (inv_theta * inv_theta)
            translation = mu[:3] + B * w_x_mu + C * cross(w, w_x_mu)
        rotation = Self.rodrigues_so3_exp(w, A, B)
        # invert translation
        rotinv = rotation.I
        translinv = -(translation * rotinv.T)
        translinv = array(translinv).squeeze()
        return translinv
    
    @staticmethod
    def rodrigues_so3_exp(w, A, B):
        R = zeros((3,3))
        wx2 = w[0]*w[0]
        wy2 = w[1]*w[1]
        wz2 = w[2]*w[2]
        R[0][0] = 1.0 - B*(wy2 + wz2)
        R[1][1] = 1.0 - B*(wx2 + wz2)
        R[2][2] = 1.0 - B*(wx2 + wy2)
        a = A*w[2]
        b = B*(w[0]*w[1])
        R[0][1] = b - a
        R[1][0] = b + a
        a = A*w[1]
        b = B*(w[0]*w[2])
        R[0][2] = b + a
        R[2][0] = b - a
        a = A*w[0]
        b = B*(w[1]*w[2])
        R[1][2] = b - a
        R[2][1] = b + a
        return matrix(R)

""" index parser """
class IndexParser:
    room_num = None
    indexUndistorter = None
    necessary = False
    
    def __init__(self, room_num):
        self.room_num = room_num
        #self.indexUndistorter = undistortionlib.IndexUndistorter(mapdir(room_num))
        #self.necessary = self.indexUndistorter.isNecessary()
    
    def parse(self, p):
        if self.necessary:
            return self.indexUndistorter.parse(p)
        else:
            return p

""" map.xml parser """
class MapXmlParser:
    current_mappoint_id = -1
    current_mappoint_pos = None
    keyframes = None
    pointparser = None
    room_num = None
    
    def __init__(self, room_num):
        self.room_num = room_num
        self.keyframes = {}
        self.pointparser = PointParser(room_num)
        self.indexparser = IndexParser(room_num)
    
    def parse(self):
        """ allow caching """
        if not self.parseIsNecessary():
            return self.loadParsedMap()
        """ parse xml """
        p = xml.parsers.expat.ParserCreate()
        p.StartElementHandler = self.start_element
        p.EndElementHandler = self.end_element
        p.CharacterDataHandler = self.char_data
        p.Parse(open(mapdir(self.room_num) + '/map.xml').read(), 1)
        """ sanity check """
        def unique(seq): 
           # order preserving
           noDupes = []
           [noDupes.append(i) for i in seq if not noDupes.count(i)]
           return noDupes
        for kfid, kf in self.keyframes.items():
            #assert len(kf['points']), (kfid, kf)
            point_ids = [p['id'] for p in kf['points']]
            assert point_ids == unique(point_ids), (kfid, kf)
        """ return """
        self.saveParsedMap()
        return self.keyframes
    
    ### 3 xmlparse handler functions ###
    def start_element(self, name, attrs):
        if name == 'KeyFrame':
            kfid = attrs['id']
            self.current_kfid = kfid
            assert not kfid in self.keyframes
            self.keyframes[kfid] = {
                'points': [],
                'pos': self.pointparser.parse(attrs['pose'], fromstring=True, frompose=True),
            }
        elif name == 'Image':
            kfid = self.current_kfid
            self.keyframes[kfid]['img'] = os.path.join(mapdir(self.room_num), attrs['file'])
        elif name == 'MapPoint':
            self.current_mappoint_id = attrs['id']
            self.current_mappoint_pos = self.pointparser.parse(attrs['position'], fromstring=True)
        elif name == 'SourceKF' and self.current_mappoint_pos != None:# and attrs['level'] == '0':
                    # note: current_mappoint_pos is None when outlier
            parsedindex = self.indexparser.parse(array([int(attrs['y']), int(attrs['x'])]))
            if parsedindex != None:
                kf = self.keyframes[attrs['id']]
                kf['points'].append({
                    'id': self.current_mappoint_id,
                    'pos': self.current_mappoint_pos,
                    'imgpos': parsedindex,
                    'level': attrs['level'],
                })
    def end_element(self, name):
        pass
    def char_data(self, data):
        pass
    
    def parseIsNecessary(self):
        mapfname = mapdir(self.room_num) + '/map.xml'
        backupfname = mapdir(self.room_num) + '/KeyFrames/undistorted_originals'
        cachefname = self.cachedFilePath()
        try:
            return os.stat(mapfname).st_mtime > os.stat(cachefname).st_mtime \
                or (os.path.isdir(cachefname) and os.stat(backupfname).st_mtime > os.stat(cachefname).st_mtime)
        except OSError:
            return True
    
    def saveParsedMap(self):
        f = open(self.cachedFilePath(), 'w')
        pickle.dump(self.keyframes, f)
    
    def loadParsedMap(self):
        f = open(self.cachedFilePath(), 'r')
        return pickle.load(f)
    
    def cachedFilePath(self):
        return "{}/../cache/map{}.pkl".format(mapdir(), self.room_num)

""" WallParser """
class WallParser:
    room_num = None
    def __init__(self, room_num):
        self.room_num = room_num
        
    def parse(self):
        strdata = open(mapdir(self.room_num)+'/walls').read()
        walls = decode_string_to_tuplearray(String(strdata))
        # convert from mm to m
        walls = array(walls) / 1e3
        # apply scaling
        walls *= Scale.get(self.room_num)
        return walls
    
    @staticmethod
    def ensureRightWallDirectedToRoom(walls):
        for i, (E0, E1) in enumerate(walls):
            e = E1 - E0
            e /= norm(e)
            n = [e[1], -e[0]]
            M = .5*(E1 + E0)
            point_on_right = concave.point_inside_area(walls, M + n)
            if not point_on_right:
                walls[i] = [E1, E0]
        return array(walls)

""" DoorParser """
class DoorParser:
    @staticmethod
    def parse():
        door_pos_arr = []
        room_nums_arr = []
        for strdata in open(mapdir()+'/doors').readlines():
            doors = [door.split('::') for door in strdata.split(' | ')]
            door_pos = [[float(x) for x in pt.split(';')]  for pt, room_num in doors]
            room_nums = [int(room_num)  for pt, room_num in doors]
            # convert from mm to m
            door_pos = array(door_pos) / 1e3
            door_pos_arr.append(door_pos)
            room_nums_arr.append(room_nums)
        # check if room_nums are logically ordered (0,1  1,2  2,3  ...)
        for i, (n1, n2) in enumerate(room_nums_arr):
            assert i == n1, 'ilogically ordered room nums ({})'.format(room_nums_arr)
            assert i+1 == n2, 'ilogically ordered room nums ({})'.format(room_nums_arr)
        # get num rooms
        num_rooms = array(room_nums_arr).max() + 1 if room_nums_arr else 1
        return array(door_pos_arr), num_rooms

class Scale:
    scales = None
    @classmethod
    def get(Self, room_num):
        if Self.scales == None:
            try:
                Self.scales = []
                for s in open(mapdir()+'/scales').readlines():
                    Self.scales.append(float(s))
            except:
                Self.scales = 1
            return Self.get(room_num)
        elif Self.scales == 1:
            return 1
        else:
            return Self.scales[room_num]
                
   
""" test code """
if __name__ == '__main__':
    """ test lnToTranslation """
    testdata_rows = open('testdata').read().strip().split('**************')
    for row in testdata_rows:
        row = row.strip().split('----------')
        ln, transl, rot, invtransl = [array([[float(x) for x in xx.strip().split(' ')] \
                                                       for xx in xxx.strip().split('\n')]).squeeze() \
                                                       for xxx in row]
        #translhat, rothat = PointParser.lnToTranslation(ln)
        #assert_almost_equal(translhat, transl, decimal=6)
        #assert_almost_equal(rothat, rot, decimal=5)
        invtranslhat = PointParser.lnToTranslation(ln)
        assert_almost_equal(invtranslhat, invtransl, decimal=5)
    print "lnToTranslation tested on %d samples with success" % len(testdata_rows)
    pass






