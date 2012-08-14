""" imports """
# std imports
import Image, time, os
from numpy import array, cos, sin
import numpy as np
from pprint import pprint
# custom imports
from mapparser import MapXmlParser, WallParser, DoorParser
import projectionlib
from multimaplib import MultimapTransform

class MapMaker:
    WALL_HEIGHT = projectionlib.WALL_HEIGHT
    _walls_list = None
    _keyframes_list = None

    def makeMap(self, wall_width, build_texture=True):
        ### first parse doors file ###
        door_pos, num_rooms = DoorParser.parse()
        self.num_rooms = num_rooms
        ### get all maps (map.xml) ###
        self.getMaps()
        
        ### transform the rooms so they fit naturally on the door positions ###
        for i, (pos0, pos1) in enumerate(door_pos):
            # get transform from pos0 to pos1
            multimapTransform = MultimapTransform(wall_width, [pos0, pos1], (self._walls_list[i], self._walls_list[i+1]))
            # transform rooms[i+1]
            self._walls_list[i+1] = array([multimapTransform.transform(wall) for wall in self._walls_list[i+1]])
            # transform keyframes[i+1]
            for kf in self._keyframes_list[i+1].values():
                kf['pos'] = multimapTransform.transform(kf['pos'])
                for pt in kf['points']:
                    pt['pos'] = multimapTransform.transform(pt['pos'])
            # transform pos0 of next
            if len(door_pos) > i+1:
                door_pos[i+1][0] = multimapTransform.transform(door_pos[i+1][0])
        
        ### make texture ###
        if build_texture:
            self.createTexture()
        
        ### return all necessary data foor wallviz ###
        floorF_list = [projectionlib.FloorImage.getFFromWalls(walls) for walls in self._walls_list]
        rooms = zip(self._walls_list, floorF_list)
        return rooms
    
    def getMaps(self):
        """ parse maps """
        print "[*] parsing maps..."
        tic = time.time()
        self._walls_list = []
        self._keyframes_list = []
        for room_num in range(self.num_rooms):
            keyframes = MapXmlParser(room_num).parse()
            walls = WallParser(room_num).parse()
            walls = WallParser.ensureRightWallDirectedToRoom(walls)
            self._keyframes_list.append(keyframes)
            self._walls_list.append(walls)
        toc = time.time()
        print "[*] done in %.2f mins" % ((toc-tic)/60)
    
    def createTexture(self):
        """ create texture for all rooms """
        tic = time.time()
        for room_num, (walls, keyframes) in enumerate(zip(self._walls_list, self._keyframes_list)):
            print "[*] ===== parsing room {} OF {} =====".format(room_num+1, self.num_rooms)
            N = len(walls)
            ### walls ###
            for n, (E0, E1) in enumerate(walls):
                print "[*] creating wall texture {0}/{1}...".format(n+1, N)
                wallimg = projectionlib.WallImage(E0, E1)
                wallimg.projectKeyframes(keyframes)
                wallimg.toImage().save(self.wallPathFromIndex(room_num, n))
            ### floor ###
            print "[*] creating floor texture..."
            floorimg = projectionlib.FloorImage(walls)
            floorimg.projectKeyframes(keyframes)
            floorimg.toImage().save(self.floorPath(room_num))
            toc = time.time()
        projectionlib.DebuggerPlot.close()
        print "[*] done in %.2f mins\n" % ((toc-tic)/60)
    
    @staticmethod
    def wallPathFromIndex(room_num, i):
        selfpath = os.path.dirname(__file__)
        return "{}/../cache/wall_{}_{}.png".format(selfpath, room_num, i)

    @staticmethod
    def floorPath(room_num):
        selfpath = os.path.dirname(__file__)
        return "{}/../cache/floor_{}.png".format(selfpath, room_num)



