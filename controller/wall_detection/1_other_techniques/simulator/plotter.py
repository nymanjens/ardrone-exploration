from copy import copy
from pylab import *
from line_def import lines

def plot_lines():
    lines_m = 1e-3 * lines
    for AB in lines_m:
        # convert to meter
        x,y = AB.T
        plot(x, y, '-k')
    # set lims
    set_lims(array([A for A,B in lines]), D=1)
    draw_labels()

def plot_points(points):
    x,y = 1e-3 * points.T
    plot(x, y, '.b')
    # set lims
    set_lims(points, D=.5)
    axes().set_aspect('equal')
    draw_labels()

def plot_walls(L):
    plot_edge_list([l.E() for l in L])

def plot_edge_list(L):
    L = array(L)
    for E in L:
        x,y = E.T / 1e3
        plot(x, y, '.-r', linewidth=2.5, ms=15)

def plot_abc_line((a, b, c), points):
    set_lims(points, D=.5)
    xmin, xmax = xlim(); dx = (xmax-xmin)/8e2
    x = 1e3*arange(xmin, xmax, dx)
    y = (-a*x - c) / b
    plot(x/1e3, y/1e3, '-r', linewidth=2.5)

def plot_hough(thetas, rhos, hpoints, peaks=None):
    H = copy(hpoints)
    if peaks != None:
        # alter H, so peaks are shown
        D = 7 #px
        white = H.max()
        for x0,y0 in peaks:
            for x in x0 + arange(-D, D+1):
                for y in y0 + array([-D, -D+1, D-1, D]):
                    if x > 0 and y > 0:
                        try: H[x,y] = white
                        except: pass
            for y in y0 + arange(-D, D+1):
                for x in x0 + array([-D, -D+1, D-1, D]):
                    if x > 0 and y > 0:
                        try: H[x,y] = white
                        except: pass
    H = H.max() - H
    rhos_m = rhos / 1e3
    pcolormesh(thetas, rhos_m, H.T, cmap=cm.gray_r)
    # set labels
    xlabel('theta (rad)')
    ylabel('rho (m)')
    # set lims
    vals = thetas; xlim(min(vals), max(vals))
    vals = rhos_m; ylim(min(vals), max(vals))

# help functions
def set_lims(points, D):
    points_m = points / 1e3
    vals = points_m.T[0]; xlim(min(vals)-D, max(vals)+D)
    vals = points_m.T[1]; ylim(min(vals)-D, max(vals)+D)
def draw_labels():
    xlabel('x (m)')
    ylabel('y (m)')


