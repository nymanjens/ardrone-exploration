#!/usr/bin/python
""" imports """
# std imports
from math import pi, sin, cos
from numpy import array
from numpy.linalg import norm
# ROS imports
import roslib; roslib.load_manifest('controller'); import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
# custom imports
from utility import *
from principalplane import Principalplane
from floorseparator import is_floor, is_wall

# def's
class FlyToWallAction:
    ctrlr = None
    angle = None
    target = None
    
    def __init__(self, ctrlr, angle):
        self.ctrlr = ctrlr
        self.angle = angle # in degrees, defined like psi
    
    def __str__(self):
        return "flying to wall ({} deg)".format(self.angle)
    
    def update(self, pos, psi):
        if not self.target:
            angle = self.angle * pi / 180
            angle_vec = array([cos(angle), sin(angle)])
            self.target = (pos + 20e3 * angle_vec, self.angle)
            self.ctrlr.publishTarget(self.target)
        # check if there are enough points in the forbidden area to conclude that there is a wall
        if sum((self.inForbiddenArea(pos, pt) for pt in self.ctrlr.wall_points)) > 4:
            return True
        return False
    
    def inForbiddenArea(self, pos, pt):
        RADIUS = 300 #mm
        pt = array([pt[0], pt[1]])
        in_circle = norm(pos - pt) < RADIUS
        #TODO: create half circle
        return in_circle

class CycleAction:
    ctrlr = None
    pos = None
    angle = None
    
    def __init__(self, ctrlr):
        self.ctrlr = ctrlr
    
    def __str__(self):
        return "cycling"
    
    @staticmethod
    def anglediff(a, b):
        diff = a - b
        return (diff + pi/2) % pi - pi/2
    
    def update(self, pos, psi_deg):
        psi = psi_deg * pi / 180
        if self.pos == None:
            self.pos = pos
            self.startangle = psi
            self.angle = psi + pi/2
            print "  [ExploringController] cycling action: startpos={}, startangle={}".format(self.pos, self.startangle)
        elif 0 < self.anglediff(psi, self.angle) < pi/3:
            self.angle = self.angle + pi/2
            print "  [ExploringController] cycling action: angle={}".format(self.angle)
        else:
            return False
        target = (self.pos, self.angle)
        self.ctrlr.publishTarget(target)
        done = self.angle > self.startangle + 2*pi
        return done

#class CycleAction:
#    ctrlr = None
#    pos = None
#    angle = None
#    
#    def __init__(self, ctrlr):
#        self.ctrlr = ctrlr
#    
#    def __str__(self):
#        return "cycling"
#    
#    def update(self, pos, psi_deg):
#        psi = psi_deg * pi / 180
#        if self.pos == None:
#            self.pos = pos
#            self.startangle = psi
#            self.angle = psi
#            print "  [ExploringController] cycling action: startpos={}, startangle={}".format(self.pos, self.startangle)
#        else:
#            self.angle = self.angle + .1
#            print "  [ExploringController] cycling action: angle={}".format(self.angle)
#        target = (self.pos, self.angle)
#        self.ctrlr.publishTarget(target)
#        done = self.angle > self.startangle + 2*pi
#        return done

class ExploringController:
    # utility vars (static)
    targetPublisher = rospy.Publisher('/goal', PoseStamped)
    statePublisher = StatePublisher('/explorer/state')
    principalplane = Principalplane()
    # regular vars
    wall_points = None
    walls = None
    action = None
    
    def __init__(self):
        self.walls = []
        self.wall_points = []
        #self.setAction(FlyToWallAction(self, 45))
        self.setAction(CycleAction(self))
    
    def setAction(self, action_object):
        self.action = action_object
        # output
        action_descr = str(action_object) if action_object else "no action"
        self.statePublisher.publish(action_descr)
        print "  [ExploringController] new action:", action_descr
    
    def setNextAction(self):
        #TODO
        #x.__class__.__name__
        self.setAction(None)
        rospy.signal_shutdown("done")
    
    def updatePoints(self, points_str):
        points_arr = decode_string_to_array(points_str)
        points_arr = self.principalplane.updateAndTransform(points_arr)
        self.wall_points = [x for x in points_arr if is_wall(x)]
    
    def updatePos(self, pos_str):
        pos = decode_string_to_dict(pos_str)
        pos = self.principalplane.updateAndTransform(pos)
        P = array([pos['x'], pos['y']])
        psi = pos['psi']
        done = self.action.update(P, psi) if self.action else False
        if done:
            self.resetTarget()
            self.setNextAction()
    
    def publishTarget(self, (xy, psi)):
        if xy != None:
            p = Point(xy[0], xy[1], 0)
            o = Quaternion()
            o.x, o.y, o.z, o.w = quaternion_from_euler(0, 0, psi)
            target = PoseStamped(pose=Pose(position=p, orientation=o))
        else:
            target = PoseStamped()
            target.header.frame_id = '-1'
        self.targetPublisher.publish(target)
    
    def resetTarget(self):
        self.publishTarget((None, None))

# init
rospy.init_node('exploring_controller')
ctrlr = ExploringController()


rospy.Subscriber("/ardrone/hybriddata", String, ctrlr.updatePos, queue_size=1)
rospy.Subscriber("/map/points", String, ctrlr.updatePoints, queue_size=1)
rospy.spin()
print ""



