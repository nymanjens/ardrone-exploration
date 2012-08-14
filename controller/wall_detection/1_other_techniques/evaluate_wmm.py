import sys, time
sys.path.append('..')
from walldetectionlib import detect_room
import pylab as pl
from simulator.plotter import *
from simulator.points_generator import *
import roslib; roslib.load_manifest('controller'); import rospy
from utility import *

""" generate points """
points = generate_points(1500)

""" non-generated points """
#from simulator.points_dataset_... import points
#from outlierfilter import outlierfilterlib
#points = outlierfilterlib.filter_outliers(points)

""" pre-filtered points """
#from simulator.points_dataset_fish_robot_room import points

""" detect room """
#tic = time.time()
#L = detect_room(points, do_preprocessing=False, numwalls=[4], numcalculations=5, fancy_output=True)
#toc = time.time()
#print "  done in ", (toc-tic), "s"

""" plot """
plot_lines()
plot_points(points)
#plot_walls(L)
show()

""" print """
#print encode_tuplearray_to_string([ l.E() for l in L ])




