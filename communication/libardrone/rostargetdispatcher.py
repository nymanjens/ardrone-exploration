""" imports """
# std imports
from numpy import array, pi, dot
from numpy.linalg import norm
from math import atan2, sin, cos, ceil, sqrt
# ROS imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
# custom imports
from utility import *
from principalplane import Principalplane

class State:
    IDLE, SET_ANGLE, FLY_TO_TARGET = range(3)
    state = None
    startpos = None
    def __init__(self):
        self.state = self.IDLE
        self.startpos = None

class CommandOrganiser:
    # settings
    MAX_SPEED = {
        'moveforward': 0.15, # max speed per mm
        'moveright':   0.15, # max speed per mm
        'turnright':   0.5, # max speed per deg
    }
    WEIGHT_FACTORS = {
        'moveforward': 1./2e3, # max speed = 1
        'moveright':   1./2e3, # max speed = 1
        'turnright':   1./20, # max speed = 1
    }
    # variables
    drone = None
    commanded = None
    stopped = None
    statePublisher = None
    
    def __init__(self, drone):
        self.drone = drone
        self.stopped = True
        self.commanded = {
            'moveforward': 0,
            'moveright': 0,
            'turnright': 0,
        }
        self.statePublisher = StatePublisher('/ardrone/state')

    def _reset_commanded(self):
        for k in self.commanded:
            self.commanded[k] = 0
    
    def schedule(self, command, distance):
        speed = distance * self.WEIGHT_FACTORS[command]
        if abs(speed) > 1:
            speed /= abs(speed)
        speed *= self.MAX_SPEED[command]
        self.commanded[command] = speed

    def stop(self):
        if not self.stopped:
            print "  [targetdispatcher] drone.stop()"
            self.drone.hover()
            self.stopped = True
            self._reset_commanded()
        self.statePublisher.publish('stopped')

    def flush(self):
        # check if any of the commands is non zero
        if not norm(array(self.commanded.values())):
            return self.stop()
        
        moveright, moveforward, turnright = self.commanded['moveright'], self.commanded['moveforward'], self.commanded['turnright']
        print "  [targetdispatcher] move: {0:4s}{1:4s}{2:4s}  {3: .2f} {4: .2f} {5: .2f}".format(
                self._cmd2str(moveright, '-->', '<--', reversecutpos=True),
                self._cmd2str(moveforward, '^', 'v'),
                self._cmd2str(turnright, '//>', '<\\\\', reversecutpos=True),
                moveright, moveforward, turnright)
        self.drone.do_combined_actions(moveright=moveright, moveforward=moveforward, turnright=turnright)
        self._reset_commanded()
        self.stopped = False
        self.statePublisher.publish('flying_to_target')
    
    def _cmd2str(self, value, posstr, negstr, reversecutpos=False):
        MAX_LEN = 3
        if value == 0:
            return ""
        s = posstr if value > 0 else negstr
        while len(s) < MAX_LEN: s *= 2
        strlen = int(ceil(abs(value) * MAX_LEN / 0.2))
        strlen = MAX_LEN if strlen > MAX_LEN else strlen
        return s[:strlen] if not (reversecutpos and value>0) else s[-strlen:]

class RosTargetDispatcher:
    """ translates ros commands into drone function calls """
    # settings
    TARGET_RADIUS = 50 #mm
    # variables
    target = None
    state = None
    principalplane = None
    comorg = None
    
    def __init__(self, drone, allow_ros_control):
        # init vars
        self.allow_ros_control = allow_ros_control
        self.reset()
        self.principalplane = Principalplane()
        self.comorg = CommandOrganiser(drone)
        # ros subscriptions
        rospy.Subscriber("/goal", PoseStamped, self.setTarget)
        rospy.Subscriber("/ardrone/hybriddata", String, self.updateDirection)
    
    def reset(self, target=None):
        self.state = State()
        self.target = target
    
    def setTarget(self, poseStamped):
        if poseStamped.header.frame_id == '-1':
            self.reset()
            print "  [targetdispatcher] done"
        elif self.target != poseStamped.pose:
            self.reset(poseStamped.pose)
            self.comorg.stop()
    
    def updateDirection(self, string_obj):
        """ init & checks """
        pos = decode_string_to_dict(string_obj)
        if not self.target:
            return self.comorg.stop()
        if not self.allow_ros_control():
            self.reset(self.target)
            return self.comorg.stop()
        # transform pos to principalplane
        pos = self.principalplane.updateAndTransform(pos)
        P_quad = array([pos['x'], pos['y']])
        psi_q = pos['psi']
        
        """ check confidence & num points """
        if pos['confidence'] < 0.6:
            print "  [targetdispatcher] no confidence"
            return self.comorg.stop()
        elif pos['num_found_points'] < 100:
            print "  [targetdispatcher] not enough SLAM points"
            return self.comorg.stop()

        """ get local target """
        # <disable state system>
        #if self.state.state == State.IDLE:
        #    self.state.startpos = P_quad
        #    self.state.state = State.SET_ANGLE
        #    P_target = P_quad
        #elif self.state.state == State.SET_ANGLE:
        #    P_target = self.state.startpos
        #elif self.state.state == State.FLY_TO_TARGET:
        #    P_target = array([self.target.position.x, self.target.position.y])
        P_target = array([self.target.position.x, self.target.position.y])
        dP = P_target - P_quad

        """ get target psi """
        o = self.target.orientation
        target_psi = 180/pi * euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        angle_diff = ((target_psi - psi_q + 180) % 360) - 180

        """ check if reached target """
        reached_target_pos = norm(dP) < self.TARGET_RADIUS
        reached_target_angle = abs(angle_diff) < 5 #deg
        
        """ update state """
        if reached_target_angle and reached_target_pos:
            # <disable state system>
            #if self.state.state == State.SET_ANGLE:
            #    print "angle ok, proceeding"
            #    self.state.state = State.FLY_TO_TARGET
            #elif self.state.state == State.FLY_TO_TARGET:
            #    if not self.comorg.stopped: print "target reached"
            #    self.comorg.stop()
            if not self.comorg.stopped: print "target reached"
            return self.comorg.stop()
        
        """ update commands """
        if not reached_target_pos:
            psi = psi_q / 180.0 * pi
            u_qx = array([cos(psi), sin(psi)])
            u_qy = array([-sin(psi), cos(psi)])
            dFront = dot(dP, u_qx)
            dRight = -dot(dP, u_qy)
            for diff, action in ((dFront, 'moveforward'), (dRight, 'moveright')):
                if abs(diff) < self.TARGET_RADIUS/sqrt(2):
                    continue
                else:
                    self.comorg.schedule(action, diff)
        if not reached_target_angle:
            self.comorg.schedule('turnright', -angle_diff)
        self.comorg.flush()
                
                
        
        





