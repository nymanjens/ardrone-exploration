#!/usr/bin/python
""" imports """
# std imports
# ...
# ROS imports
import roslib; roslib.load_manifest('wallviz'); import rospy
from std_msgs.msg import String
# custom imports
from utility import *
from principalplane import Principalplane

""" ros init """
rospy.init_node('point_visualizer')
principalplane = Principalplane()

""" get principalplane params """
principalplane.updateTransformation()
params = principalplane.getParams()

""" save params """
print "params = ", params
open('map/principalplane', 'w').write(params)

""" done """
print "done"
