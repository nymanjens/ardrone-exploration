#!/usr/bin/python
""" imports """
# std imports
import sys
from time import time
from numpy import array, pi, matrix
from math import sin, cos
# ROS imports
import roslib; roslib.load_manifest('communication'); import rospy
# ROS imports
from libardrone.libardrone import ARDrone
from libardrone.arcommandergui import ARCommanderGUI
from libardrone.rostargetdispatcher import RosTargetDispatcher
from libcom.publishlib import Publisher

""" BASH COMMANDS """
# to view: rosrun image_view image_view image:=/ardrone_video0/image
# to get refreshrate: rostopic hz /ardrone_video0/image

""" initialization """
rospy.init_node("ardrone_data_elf")
publisher = Publisher(True)
drone = ARDrone()
x = y = z = 0
SIMPLEDATA_VARS = ('num_frames', 'battery', 'altitude', 'vx', 'vy', 'vz', 'theta', 'phi', 'psi', 'x', 'y', 'z')
commander = None

""" start commander (optional) """
commander = ARCommanderGUI(drone, draw_vid=True)

""" start ros controller (optional) """
rosTargetDispatcher = RosTargetDispatcher(drone, allow_ros_control = lambda: not commander or commander.allow_ros_control)

""" main loop """
timestamp = None
try:
    while commander == None or commander.is_alive():
        """ calculate time since last iteration """
        prev_timestamp = timestamp
        timestamp = time()
        time_delta = timestamp - prev_timestamp if prev_timestamp else 0
        """  get data """
        image = drone.wait_for_data(keep_alive = lambda: commander == None or commander.is_alive())
        if not image: # timeout
            continue
        simpledata = {}
        for var in SIMPLEDATA_VARS:
            simpledata[var] = drone.getFromNavdata(var)
        # transform vx and vy
        vnav = array([simpledata['vx'], simpledata['vy']])
        psi = simpledata['psi'] / 180.0 * pi
        R = matrix([
            [cos(psi), -sin(psi)],
            [sin(psi),  cos(psi)]
        ])
        v = matrix(vnav) * R.T
        simpledata['vx'], simpledata['vy'] = array(v)[0]
        # add v to pos-estimation
        x += simpledata['vx'] * time_delta # (conversion from mm/s to mm)
        y += simpledata['vy'] * time_delta
        z += simpledata['vz'] * time_delta
        simpledata['x'] = x
        simpledata['y'] = y
        simpledata['z'] = z
        simpledata['timestamp'] = timestamp
        simpledata['spacebar_pressed'] = 1 if commander and commander.getSpacebarPressed() else 0
        
        """ publish data """
        publisher.publish_elf(simpledata, image)
        
        """ print data """
        #s = "\n" * 40
        #s += "*********************************\n"
        #for var in SIMPLEDATA_VARS: s += "%s = %s\n" % (var, simpledata[var])
        #s += "*********************************\n"
        #print "%s" % s
    
except KeyboardInterrupt:
    print "\n<done>\n"
except:
    if str(sys.exc_info()).startswith("(<class 'select.error'>, error(4, 'Interrupted system call')"):
        print "\n<done>\n"
    else:
        print "\n<Unexpected error>\n"
        # land and halt
        drone.land()
        drone.halt()
        print "\n\n* Error message:"
        raise

# land and halt
if commander: commander.stop()
drone.land()
drone.halt()
if commander and commander.is_alive():
    commander.join()


