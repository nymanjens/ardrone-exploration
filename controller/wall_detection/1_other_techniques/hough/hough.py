from numpy import pi, cos, sin, array, arange, zeros
from numpy.linalg import norm

 
def hough_transform(points, n_theta=460, n_rho=360):
    """ Calculate Hough transform """
    n_rho2 = int(n_rho/2)
    n_rho = n_rho2*2 + 1
    rho_max = max(norm(x) for x in points)
    delta_rho = rho_max / n_rho2
    delta_theta = pi / n_theta
    theta_offset = -pi/4
#    theta_offset = 0
    thetas = arange(theta_offset, pi+theta_offset, delta_theta)
    rhos = arange(-rho_max, rho_max+delta_rho/2, delta_rho)
    assert len(thetas) == n_theta
    assert len(rhos) == n_rho
 
    houghmx = zeros((n_theta, n_rho))
    for x,y in points:
        for i_theta, theta in enumerate(thetas):
            rho = x*cos(theta) + y*sin(theta)
            i_rho = n_rho2 + round(rho/delta_rho)
            houghmx[i_theta, i_rho] += 1
    return rhos, thetas, houghmx
 
 
 
 
 
 
 
