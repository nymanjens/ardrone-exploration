import sys; sys.path.append('..')
from points import points, N
from wmmlib import *
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt
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

# 3D plot init
fig = plt.figure()
ax = fig.gca(projection='3d')
# lims
E = vstack((E1, E2))
margin = 4*L.sigma
vals = E[:,0]; xmin=vals.min()-margin; xmax=vals.max()+margin
vals = E[:,1]; ymin=vals.min()-margin; ymax=vals.max()+margin
# XY vals
N = 30
X = arange(xmin, xmax, (xmax-xmin)/N)
Y = arange(ymin, ymax, (ymax-ymin)/N)
X, Y = meshgrid(X, Y)
Z = zeros(X.shape)
print "calculating probabilities..."
for row, (xrow, yrow) in enumerate(progressinfo(zip(X, Y))):
    for col, (x,y) in enumerate(zip(xrow, yrow)):
        Z[row, col] = L.Prob([x,y])
#cmap = cm.gray_r # black&white
cmap = cm.jet # color
surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cmap, linewidth=0, antialiased=False)
# unknown
ax.w_zaxis.set_major_locator(LinearLocator(6))
# colorbar
#fig.colorbar(surf, shrink=0.5, aspect=5)
# set view
ax.view_init(elev=47, azim=-69)
xlabel('x (m)', fontsize=20)
ylabel('y (m)', fontsize=20)
fig.suptitle('Prob[(x,y)]', fontsize=30)
plt.show()




