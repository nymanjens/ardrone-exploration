import sys, time
sys.path.append('..')
from walldetectionlib import detect_room
import pylab as pl
from simulator.plotter import *
from simulator.points_generator import *
import roslib; roslib.load_manifest('controller'); import rospy
from utility import *
from simulator.evaluator import Evaluator

""" setup evaluator """
evaluator = Evaluator("wmm")

""" generate points """
points = generate_points(evaluator.sigma)

""" non-generated points """
#from simulator.points_dataset_... import points
#from outlierfilter import outlierfilterlib
#points = outlierfilterlib.filter_outliers(points)

""" pre-filtered points """
#from simulator.points_dataset_fish_robot_room import points

""" detect room """
#L = detect_room(points, do_preprocessing=False, numwalls=[4, 6, 8, 10], numcalculations=4, fancy_output=True)
#L = detect_room(points, do_preprocessing=False, numwalls=[4, 6], numcalculations=2, fancy_output=True)
L = detect_room(points, do_preprocessing=False, numwalls=[4], numcalculations=12, fancy_output=True)
#L = detect_room(points, do_preprocessing=False, numwalls=[4], numcalculations=1, fancy_output=True)

""" evaluate """
evaluator.evaluate_lines(L, 'line', points)

""" plot """
ioff()
plot_lines()
plot_points(points)
plot_walls(L)
show()

""" print """
#print encode_tuplearray_to_string([ l.E() for l in L ])




