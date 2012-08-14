"""
pyggle.include
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The include module imports all necessary libraries,
as well as creates a blank, white texture for general use on non-textured objects.
"""

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import numpy

try:
    from OpenGL.GL.EXT.framebuffer_object import *
    FBO_AVAILABLE = True
except:
    FBO_AVAILABLE = False
