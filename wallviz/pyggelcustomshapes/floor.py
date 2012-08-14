from pyggel.include import *
from pyggel.geometry import *
import numpy as np
from numpy import pi
from numpy.linalg import norm

FLOOR_COLOR = (0,0,0,1)

class Floor(Plane):
    def __init__(self, F1, F2, texture=None):
        # parse args
        F1 = np.array(F1)
        F2 = np.array(F2)
        x,y = .5 * (F1 + F2)
        dF = abs(F2-F1)
        self.a = dF[0]
        self.b = -dF[1]
        color = FLOOR_COLOR if not texture else (1,1,1,1)
        Plane.__init__(self, size=1, pos=(x,0,y), rotation=(0,0,0), colorize=color, texture=texture, tile=1)

    def _compile(self):
        """Compile Plane into a data.DisplayList"""
        self.display_list.begin()

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_REPEAT)

        glBegin(GL_QUADS)
        glNormal3f(0,1,0)
        glTexCoord2f(self.tile,0)
        glVertex3f(self.a, 0, -self.b)
        glTexCoord2f(0,0)
        glVertex3f(-self.a, 0, -self.b)
        glTexCoord2f(0,self.tile)
        glVertex3f(-self.a, 0, self.b)
        glTexCoord2f(self.tile,self.tile)
        glVertex3f(self.a, 0, self.b)

        glTexCoord2f(self.tile,self.tile)
        glVertex3f(self.a, 0, self.b)
        glTexCoord2f(0, self.tile)
        glVertex3f(-self.a, 0, self.b)
        glTexCoord2f(0,0)
        glVertex3f(-self.a, 0, -self.b)
        glTexCoord2f(self.tile, 0)
        glVertex3f(self.a, 0, -self.b)
        glEnd()
        self.display_list.end()


