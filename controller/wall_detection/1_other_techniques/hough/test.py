import sys, subprocess, os, shlex
from scipy.io import loadmat, savemat
from pylab import *
sys.path.append('..')
from simulator.plotter import *
from simulator.points_generator import *
import hough
from simulator.evaluator import Evaluator

""" setup evaluator """
evaluator = Evaluator("hough", detected_walls="N/A")

""" generate points """
points = generate_points(evaluator.sigma)

""" non-generated points """
#sys.path.append('../..')
#from simulator.points_dataset_real2 import points
#from outlierfilter import outlierfilterlib
#points = outlierfilterlib.filter_outliers(points)

""" hough transform """
rhos, thetas, hgrid = hough.hough_transform(points, n_theta=460, n_rho=360)
#print "  hough transform complete"

""" get lines from hough transform (matlab) """
MATLAB_HOUGH_FILE = 'matlab_io/input_hough.mat'
MATLAB_PEAKS_FILE = 'matlab_io/output_peaks.mat'
# check if matlab can be skipped
skip_matlab = False
if os.path.isfile(MATLAB_HOUGH_FILE):
    data = loadmat(MATLAB_HOUGH_FILE)
    if data['H'].shape == hgrid.T.shape and (data['H'] == hgrid.T).all():
        skip_matlab = True
#if not skip_matlab:
if True:
    # create input file
    savemat(MATLAB_HOUGH_FILE, {'H': hgrid.T})
    # run script
    matlab_command = "sudo /usr/local/MATLAB/R2010b/bin/matlab" # edit this!
    command_line = matlab_command + " -nodesktop -nosplash -nojvm -r 'from_hough_to_lines;exit' | grep -i warning"
    subprocess.call(command_line, shell=True, close_fds=True)
# read output file
peaks = loadmat(MATLAB_PEAKS_FILE)['P']
evaluator.toc()

""" convert from matlab to python """
for i, (rho_i, theta_i) in enumerate(peaks):
    peaks[i] = theta_i-1, rho_i-1

""" convert peaks to lines """
lines = []
peaks_theta_rho = []
for p in peaks:
    theta = thetas[p[0]]
    rho = rhos[p[1]]
    a = cos(theta)
    b = sin(theta)
    c = -rho
    peaks_theta_rho.append([theta, rho])
    lines.append([a,b,c])

""" evaluate """
evaluator.evaluate_lines(lines, 'abc', points)

""" hough plot """
## plot 1: points space
plot_lines()
plot_points(points)
for l in lines:
    plot_abc_line(l, points)
show()

## plot 2: parameter space
#plot_hough(thetas, rhos, hgrid, peaks=peaks)
#plot_hough(thetas, rhos, hgrid)
#show()


