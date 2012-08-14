#!/usr/bin/python
""" imports """
# std imports
import os
from numpy import *
# ROS imports
import roslib; roslib.load_manifest('controller'); import rospy
from std_msgs.msg import String
# custom imports
from utility import *
from principalplane import Principalplane
from floorseparator import is_wall
from wall_detection.walldetectionlib import detect_room

""" settings """
WALLVIZ_MAP_PATH = roslib.packages.get_pkg_dir('wallviz') + '/map'
DOORS_PATH = WALLVIZ_MAP_PATH + '/doors'
PTAMM_MAP_PATH = roslib.packages.get_pkg_dir('ptamm') + '/wallvizmap{map_id}'
PRINCIPALPLANE_PATH = PTAMM_MAP_PATH + "/principalplane"
WALLS_PATH = PTAMM_MAP_PATH + "/walls"

""" listeners: last position & points """
principalplane = Principalplane()
class LastPosition:
    pos = None
    @classmethod
    def update(self, pos_str):
        pos = decode_string_to_dict(pos_str)
        pos = principalplane.updateAndTransform(pos)
        self.pos = array([pos['x'], pos['y']])
class LastPoints:
    points = None
    @classmethod
    def update(self, string):
        if self.points == None:
            print " * map updated (first time)\n"
        points_arr = decode_string_to_array(string)
        self.points = principalplane.updateAndTransform(points_arr)

""" map is ready to start wallviz """
wallpublisher = rospy.Publisher('/map/walls', String)
wallvizpublisher = rospy.Publisher('/wallviz/update', String)
def wallviz_ready_signal(map_id):
    ### SANITY CHECK ###
    if LastPoints.points == None:
        print "Warning: did not yet gathered a map"
        return
    
    ### SAVE PRINCIPAL PLANE ###
    principalplane.updateTransformation()
    params = principalplane.getParams()
    open(PRINCIPALPLANE_PATH.format(map_id=map_id), 'w').write(params)
    
    ### WALL DETECTION ###
    print " * Detecting walls ..."
    # apply wall detection
    L = detect_room(LastPoints.points, do_preprocessing=True, numwalls=[4], numcalculations=12, fancy_output=True)
    # get walls list
    walls = [ l.E() for l in L ]
    # convert walls to string, publish and save
    wallstr = encode_tuplearray_to_string(walls)
    wallpublisher.publish(wallstr)
    open(WALLS_PATH.format(map_id=map_id), 'w').write(wallstr)
    print "   ... done"
    print ""
    
    ### WALLVIZ UPDATE SIGNAL ###
    wallvizpublisher.publish("")


""" flying through door signal """
door1 = None
doorString = ""
def door_signal(map_id):
    global door1, doorString
    if not door1:
        door1 = (LastPosition.pos, map_id)
        print " * door1 recorded for map", map_id
    else:
        door2 = (LastPosition.pos, map_id)
        doorString += "\n"
        doorString += ' | '.join(
            "{};{}::{}".format(pos[0], pos[1], mpid)
            for pos, mpid in [door1, door2]
        )
        doorString = doorString.strip()
        open(DOORS_PATH, 'w').write(doorString)
        print " * doors saved for map {} <-> map {}\n".format(door1[1], map_id)
        door1 = None

""" listeners: signal """
wallviz_ready_signal_process = MaxOneProcess(wallviz_ready_signal)
def signal_handler(String):
    signal, map_id = String.data.split('\n')
    if signal == 'w':
        wallviz_ready_signal_process.startProcess(map_id)
    elif signal == 'd':
        door_signal(map_id)
    else:
        print "Warning: unknown signal:", signal

""" ros init """
rospy.init_node('multimap_controller')
rospy.Subscriber("/slam/signals", String, signal_handler)
rospy.Subscriber("/ardrone/hybriddata", String, LastPosition.update, queue_size=1)
rospy.Subscriber("/map/points", String, LastPoints.update, queue_size=1)
rospy.spin()
print ""



