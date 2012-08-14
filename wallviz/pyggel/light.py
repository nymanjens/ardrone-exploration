"""
pyggle.light
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The light module contains a basic light class that interfaces with the OpenGL Light(s).
"""

from include import *
import math3d, camera

all_lights = []
for i in xrange(8):
    exec "all_lights.append(GL_LIGHT%s)"%i

class Light(object):
    """A simple 3d light"""
    def __init__(self, pos=(0,0,0), ambient=(0,0,0,0),
                 diffuse=(1,1,1,1), specular=(1,1,1,1),
                 spot_direction=(0,0,0), directional=True):
        """Create the light
           pos it the position of the light
           ambient is the ambient color of the light
           diffuse is the diffuse color of the light
           specular is how much objects mirror the light, ie how shiny they are
           spot_direction is the 3d direction the light will be facing if it is directional
           directional is whether the light is directional or global"""
        self.pos = pos
        self.directional = directional
        self.ambient = ambient
        self.diffuse = diffuse
        self.specular = specular
        self.spot_direction = spot_direction
        self.gl_light = GL_LIGHT0

    def shine(self):
        """Resets the position and enables the light, called after a camera is pushed to ensure it remains in the right place"""
        if not self.gl_light == None:
            gl_light = self.gl_light
            glLightfv(gl_light, GL_AMBIENT, self.ambient)
            glLightfv(gl_light, GL_DIFFUSE, self.diffuse)
            glLightfv(gl_light, GL_SPECULAR, self.specular)
            glLightfv(gl_light, GL_POSITION, (self.pos[0], self.pos[1], -self.pos[2], int(not self.directional)))
            glLightfv(gl_light, GL_SPOT_DIRECTION, self.spot_direction+(0,))
            glEnable(gl_light)

    def hide(self):
        """Disables the light"""
        if self.gl_light:
            glDisable(self.gl_light)
