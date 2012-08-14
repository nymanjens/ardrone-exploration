#!/usr/bin/python

""" imports """
# std imports
from numpy.random import randn
# ROS imports
import roslib; roslib.load_manifest('controller'); import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
# custom imports
from utility import *
from floorseparator import is_wall
from principalplane import Principalplane
from wall_detection.walldetectionlib import detect_walls

# globals
publisher = rospy.Publisher('/map/walls', String)
principalplane = Principalplane()

# subscribing function
def process_map_points(string):
    """ get points """
    rospy.loginfo("Starting calculation...")
    points_arr = decode_string_to_array(string)
    points_arr = principalplane.updateAndTransform(points_arr)
    
    """ apply wall detection """
    L = detect_walls(points_arr)
    
    """ get walls """
    walls = [ l.E() for l in L ]
    
    """ publish walls """
    publisher.publish(encode_tuplearray_to_string(walls))
    rospy.loginfo("published {0} lines".format(len(L)))
    
# ros init
rospy.init_node('wall_detector')
# subscribe and spin
rospy.Subscriber("/map/points", String, MaxOneProcess(process_map_points).startProcess, queue_size=1)
rospy.spin()
print ""



