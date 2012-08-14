#!/usr/bin/python
""" imports """
# std imports
from math import pi, sin, cos, acos
from numpy import array, arange
from numpy.linalg import norm
from copy import deepcopy
# ROS imports
import roslib; roslib.load_manifest('analysis'); import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
# custom imports
from utility import *
from floorseparator import is_floor, is_wall
from principalplane import Principalplane

""" BASH COMMANDS """
# to run: rosrun rviz rviz

""" ros init """
rospy.init_node('point_visualizer')
publisher = rospy.Publisher('/point_visualizer', Marker)
transformBroadcaster = tf.TransformBroadcaster()
principalplane = Principalplane()
ardrone_target_state = StateSubscriber('/ardrone/state', 'stopped')
exploring_controller_state = StateSubscriber('/explorer/state', '')

""" utility """
def get_marker(namespace, Type):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.ns = namespace
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.ADD
    marker.id = 0
    marker.type = Type
    return marker

""" listeners """
def plot_pos(string):
    """ get and transform hybriddata """
    hybriddata = decode_string_to_dict(string)
    hybriddata = principalplane.updateAndTransform(hybriddata)
    phi, theta, psi = pi/180 * array([hybriddata['phi'], hybriddata['theta'], hybriddata['psi']])
    
    """ get mother marker """
    mmarker = get_marker('position', Marker.MESH_RESOURCE)
    # scale
    mmarker.scale.x = mmarker.scale.y = mmarker.scale.z = 1000
    # position
    mmarker.pose.position.x = hybriddata['x']
    mmarker.pose.position.y = hybriddata['y']
    mmarker.pose.position.z = hybriddata['h']
    # orientation
    o = mmarker.pose.orientation
    o.x, o.y, o.z, o.w = quaternion_from_euler(-phi, -theta, psi)
    # mesh parameters
    mmarker.mesh_resource = "package://gazebo/gazebo/share/gazebo/Media/models/kinect/kinect.dae"
    
    """ publish stopped pos marker """
    marker = deepcopy(mmarker)
    # color
    marker.color.r = 1.0
    marker.color.a = 1.0
    # enable/disable
    if not ardrone_target_state.state == 'stopped':
        marker.pose.position.x = -1e20
    # publish
    publisher.publish(marker)
    
    """ publish flying pos marker (to solve color-change bug in rviz) """
    marker = deepcopy(mmarker)
    marker.ns = 'position_flying'
    # color
    marker.color.r = .1
    marker.color.g = 1.0
    marker.color.b = .1
    marker.color.a = 1.0
    # enable/disable
    if ardrone_target_state.state == 'stopped':
        marker.pose.position.x = -1e20
    # publish
    publisher.publish(marker)
    
    """ publish exploring_controller_state """
    marker = get_marker('exploring_controller_state', Marker.TEXT_VIEW_FACING)
    # text
    text = exploring_controller_state.state
    marker.text = text if text else "--"
    # scale
    marker.scale.x = marker.scale.y = marker.scale.z = 60
    # position
    marker.pose.position.x = hybriddata['x']
    marker.pose.position.y = hybriddata['y']
    marker.pose.position.z = hybriddata['h'] + 100
    if not text:
        marker.pose.position.x = -1e20
    # color
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1
    # orientation
    o = marker.pose.orientation
    o.x, o.y, o.z, o.w = quaternion_from_euler(-phi, -theta, psi)
    # publish
    publisher.publish(marker)
    
    """ broadcast target frame transform """
    transformBroadcaster.sendTransform(
        (hybriddata['x'], hybriddata['y'], hybriddata['h']),
        quaternion_from_euler(-phi, -theta, psi),
        rospy.Time.now(),
        "/quadcopter",
        "/map",
    )
    #rospy.loginfo("published pos")

def plot_target(target):
    """ (no transformation needed) """
    
    """ publish target marker """
    marker = get_marker('target', Marker.MESH_RESOURCE)
    # scale
    marker.scale.x = marker.scale.y = marker.scale.z = 1000
    # color
    marker.color.b = 1.0
    marker.color.a = 1.0
    # position
    marker.pose = target.pose
    marker.pose.position.z = 550
    # mesh parameters
    marker.mesh_resource = "package://gazebo/gazebo/share/gazebo/Media/models/kinect/kinect.dae"
    # publish
    publisher.publish(marker)
    
    """ publish cross shadow """
    #marker = get_marker('target_shadow', Marker.LINE_LIST)
    ## scale
    #marker.scale.x = 30
    ## color
    #marker.color.b = .4
    #marker.color.a = 1.0
    ## position
    #pos = target.pose.position
    #x, y, z = pos.x, pos.y, 10
    #W = 100
    #marker.points.append(Point(x-W, y-W, z))
    #marker.points.append(Point(x+W, y+W, z))
    #marker.points.append(Point(x+W, y-W, z))
    #marker.points.append(Point(x-W, y+W, z))
    ## publish
    #publisher.publish(marker)
    #rospy.loginfo("published target")
    
    """ publish arrow shadow """
    marker = get_marker('target_arrow_shadow', Marker.ARROW)
    L = 200.
    # scale
    marker.scale.x = marker.scale.y = 900
    marker.scale.z = L
    # color
    marker.color.b = 1.0
    marker.color.a = 1.0
    # position
    marker.pose = deepcopy(target.pose)
    marker.pose.position.z = 10
    o = marker.pose.orientation
    psi = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    marker.pose.position.x -= L/2. * cos(psi)
    marker.pose.position.y -= L/2. * sin(psi)
    # publish
    publisher.publish(marker)
    
    """ publish circle """
    marker = get_marker('target_circle_shadow', Marker.LINE_STRIP)
    # scale
    marker.scale.x = 40
    # color
    marker.color.b = 1.0
    marker.color.a = 1.0
    # position
    pos = target.pose.position
    x, y, z = pos.x, pos.y, 10
    R = 200
    D = .2
    for theta in arange(0, 2*pi+D, D):
        dx = R*cos(theta)
        dy = R*sin(theta)
        marker.points.append(Point(x+dx, y+dy, z))
    # publish
    publisher.publish(marker)
    rospy.loginfo("published target")

def plot_points(string):
    """ get points """
    points_arr = decode_string_to_array(string)
    points_arr = [p for p in points_arr if norm(p) < 100e3]

    """ publish old, untransformed points """
    #marker = get_marker('old_points', Marker.SPHERE_LIST)
    #marker.pose.orientation.w = 1.0
    ## POINTS markers use x and y scale for width/height respectively
    #marker.scale.x = marker.scale.y = marker.scale.z = 50
    ## Points are green
    #marker.color.r = .2
    #marker.color.g = .2
    #marker.color.a = 1.0
    ## Create the vertices for the points and lines
    #for pt in points_arr:
    #    p = Point()
    #    p.x = pt[0]
    #    p.y = pt[1]
    #    p.z = pt[2]
    #    marker.points.append(p)
    ## publish
    #publisher.publish(marker)

    """ transform points """
    points_arr = principalplane.updateAndTransform(points_arr)
    
    """ publish floor """
    marker = get_marker('floor_points', Marker.SPHERE_LIST)
    marker.pose.orientation.w = 1.0
    # POINTS markers use x and y scale for width/height respectively
    marker.scale.x = marker.scale.y = marker.scale.z = 50
    # set color
    marker.color.r = .8
    marker.color.g = .8
    marker.color.a = 1.0
    # Create the vertices for the points and lines
    for pt in (x for x in points_arr if is_floor(x)):
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = pt[2]
        marker.points.append(p)
    # publish
    publisher.publish(marker)
    
    """ publish walls """
    marker = get_marker('wall_points', Marker.SPHERE_LIST)
    marker.pose.orientation.w = 1.0
    # POINTS markers use x and y scale for width/height respectively
    marker.scale.x = marker.scale.y = marker.scale.z = 50
    # set color
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    # Create the vertices for the points and lines
    for pt in (x for x in points_arr if is_wall(x)):
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = pt[2]
        marker.points.append(p)
    # publish
    publisher.publish(marker)
    
    rospy.loginfo("published {} points".format(len(points_arr)))

class WallPlotter:
    prev_num_plotted_walls = 0
    
    def plot(self, string):
        walls = decode_string_to_tuplearray(string)
        """ publish walls with cubes """
        for i, (p1, p2) in enumerate(walls):
            marker = get_marker('wall_%d' % i, Marker.CUBE)
            # set scale
            marker.scale.x = norm(p1-p2)
            marker.scale.y = 20
            marker.scale.z = 1e3
            # set color
            marker.color.r = 1.0
            marker.color.a = 1.0
            # set position
            p = marker.pose.position
            p.x, p.y = .5 * (p1 + p2)
            p.z = .5e3
            # set orientation
            dp = p2 - p1
            psi = acos(dp[0]/norm(dp))
            o = marker.pose.orientation
            o.x, o.y, o.z, o.w = quaternion_from_euler(0, 0, psi)
            # publish
            publisher.publish(marker)
        for i in range(len(walls), self.prev_num_plotted_walls):
            marker = get_marker('wall_%d' % i, Marker.CUBE)
            p = marker.pose.position
            p.x = -1e20
            publisher.publish(marker)
        self.prev_num_plotted_walls = len(walls)

""" subscribe """
rospy.Subscriber("/ardrone/hybriddata", String, plot_pos, queue_size=1)
rospy.Subscriber("/goal", PoseStamped, plot_target, queue_size=1)
rospy.Subscriber("/map/points", String, plot_points, queue_size=1)
rospy.Subscriber("/map/walls", String, WallPlotter().plot, queue_size=1)
rospy.spin()
    
print "\n\n<done>"




