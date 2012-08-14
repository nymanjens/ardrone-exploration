from pyggel.include import *
from pyggel.geometry import *
import numpy as np
from numpy import pi
from numpy.linalg import norm

WALL_COLOR = (1,0,0,1)

class Wall:
    visible = True
    pickable = True
    def __init__(self, E1, E2, texture, height=2.5, width=.7):
        # parse args
        E1 = np.array(E1)
        E2 = np.array(E2)
        dx, dy = dE = E2-E1
        n = np.array([dy, -dx]) / norm(dE)
        dn = width * n
        # create pieces
        self.pieces = []
        self.pieces.append(WallPiece(E1, E2, height, texture))
        self.pieces.append(WallPiece(E1-dn, E2-dn, height))
        self.pieces.append(WallPiece(E1-dn, E1, height))
        self.pieces.append(WallPiece(E2-dn, E2, height))
        self.pieces.append(Ceiling(E1-dn/2., E2-dn/2., height, width))
    
    def render(self, camera=None):
        for p in self.pieces:
            p.render(camera)



class WallPiece(Plane):
    def __init__(self, E1, E2, height, texture=None):
        self.width = norm(E2-E1)
        self.height = height
        x,y = .5 * (E1 + E2)
        dE = E2-E1
        theta = 180/pi * np.arctan2(dE[1], dE[0])
        color = WALL_COLOR if not texture else (1,1,1,1)
        Plane.__init__(self, size=1, pos=(x,height/2.,y), rotation=(0,theta,0), colorize=color, texture=texture, tile=1)

    def _compile(self):
        """Compile Plane into a data.DisplayList"""
        self.display_list.begin()

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_REPEAT)

        glBegin(GL_QUADS)
        glNormal3f(0,1,0)
        glTexCoord2f(self.tile,0)
        glVertex3f(self.width, -self.height, 0)
        glTexCoord2f(0,0)
        glVertex3f(-self.width, -self.height, 0)
        glTexCoord2f(0,self.tile)
        glVertex3f(-self.width, self.height, 0)
        glTexCoord2f(self.tile,self.tile)
        glVertex3f(self.width, self.height, 0)

        glTexCoord2f(self.tile,self.tile)
        glVertex3f(self.width, self.height, 0)
        glTexCoord2f(0, self.tile)
        glVertex3f(-self.width, self.height, 0)
        glTexCoord2f(0,0)
        glVertex3f(-self.width, -self.height, 0)
        glTexCoord2f(self.tile, 0)
        glVertex3f(self.width, -self.height, 0)
        glEnd()
        self.display_list.end()


class Ceiling(Plane):
    def __init__(self, E1, E2, height, width):
        # parse args
        x,y = .5 * (E1 + E2)
        dx, dy = dE = E2-E1
        n = np.array([dy, -dx]) / norm(dE)
        theta = 180/pi * np.arctan2(dE[1], dE[0])
        self.a = norm(dE)
        self.b = width
        Plane.__init__(self, size=1, pos=(x,height,y), rotation=(0,theta,0), colorize=WALL_COLOR, texture=None, tile=1)

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


