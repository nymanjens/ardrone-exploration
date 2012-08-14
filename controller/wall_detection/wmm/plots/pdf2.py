#!/opt/epd/bin/python
import sys; sys.path.append('..')
from points import points, N
from wmmlib import *
from numpy import *
try:
    from mdp.utils import progressinfo
except:
    progressinfo = lambda(x): x
    print "  Note: mdp.utils.progressinfo not loaded (only for nicer output)"

""" init data """
M = [0,0]
alpha = 3*pi/4
sigma = .5
e = 5*sigma
L = Line(M, alpha, sigma, e)
E1 = L.P(t=-e)
E2 = L.P(t=e)

""" get lims """
E = vstack((E1, E2))
margin = 4*L.sigma
vals = E[:,0]; xmin=vals.min()-margin; xmax=vals.max()+margin
vals = E[:,1]; ymin=vals.min()-margin; ymax=vals.max()+margin

""" get X and Y """
N = 300
#X = arange(xmin, xmax, (xmax-xmin)/N)
#Y = arange(ymin, ymax, (ymax-ymin)/N)
#X, Y = meshgrid(X, Y)
X, Y = mgrid[xmin:xmax:(xmax-xmin)/N, ymin:ymax:(ymax-ymin)/N]

""" get prob functions """
def gridfill(X, Y, f):
    Z = zeros(X.shape)
    for row, (xrow, yrow) in enumerate(progressinfo(zip(X, Y))):
        for col, (x,y) in enumerate(zip(xrow, yrow)):
            Z[row, col] = f([x,y])
    return Z*(xmax-xmin)

def conditionalProb(X, Y):
    return gridfill(X, Y, L.conditionalProb)

def realProb(x,y):
    return gridfill(X, Y, L.realProb)

""" plot """
from mayavi import mlab
colormap='Blues'
#colormap=None

#mlab.surf(X, Y, realProb, colormap=colormap)
mlab.surf(X, Y, conditionalProb, colormap=colormap)

#mlab.surf(X, Y, realProb, transparent=True, opacity=1.0)
#mlab.surf(X, Y, conditionalProb, representation='wireframe')
mlab.show()

