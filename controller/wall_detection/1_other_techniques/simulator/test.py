import sys, time
import pylab as pl
from plotter import *
from points_generator import *

""" generate points """
points = generate_points(50)

""" non-generated points """
#from simulator.points_dataset_real2 import points
#from outlierfilter import outlierfilterlib
#points = outlierfilterlib.filter_outliers(points)

""" plot """
plot_lines()
plot_points(points)
show()
