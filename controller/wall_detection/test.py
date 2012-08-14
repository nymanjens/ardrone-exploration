from walldetectionlib import detect_walls
from wmm.points import *
from pylab import *
import pylab as pl

L = detect_walls(points, do_preprocessing=False)


""" plots """
plot(points[:,0], points[:,1], 'b.')
for l in L:
    l.plot(pl)
show()
