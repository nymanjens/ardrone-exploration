import sys; sys.path.append('..')
from points import points, N
from wmmlib import *
from pylab import *

""" init data """
M = [0,0]
alpha = 3*pi/4
sigma = .5
e = 5*sigma
L = Line(M, alpha, sigma, e)
E1 = L.P(t=-e)
E2 = L.P(t=e)

""" get both pdf's along x """
D = e+3.5*sigma
t = arange(-D, D, .01)
xy = L.P(t)
y = lambda(f): array([f(p) for p in xy.T])
y_cond = y(L.conditionalProb)
y_real = y(L.realProb)

""" plot pdf's """
plot(t, y_real, '--b')
plot(t, y_cond, '-r')
xlabel('x')
ylabel('Q((x,0) | L)')
xlim((-D,D))
ylim((0, y_real.max() * 1.1))
legend(('Ideal pdf', 'Simplified pdf'))
show()
