"""
pyggle.picker
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The picker module contains functions and classes to assist in selecting which 3d object(s) the mouse is over.
"""

from include import *

def Pick512Objects(x, y, objs, camera):
    """Figure out which object in a list of up to 512 objects are selected.
       x, y are the mouse coords on screen
       objs is a list of renderable objects
       camera must be None or the camera the scene is using"""

    viewport = glGetIntegerv(GL_VIEWPORT)
    glSelectBuffer(512)
    glRenderMode(GL_SELECT)

    glInitNames()
    glMatrixMode(GL_PROJECTION)
    previousviewmatrix = glGetDoublev(GL_PROJECTION_MATRIX)
    glLoadIdentity()
    gluPickMatrix(x, viewport[3] - y, 1, 1, viewport)
    glMultMatrixd(previousviewmatrix)
    if camera:
        camera.push()
    for i in objs:
        glPushName(i[1])
        if i[0].visible:
            i[0].render(camera)
        glPopName()
    if camera:
        camera.pop()
    glFlush()
    glMatrixMode(GL_PROJECTION)
    glLoadMatrixd(previousviewmatrix)

    return glRenderMode(GL_RENDER)


class Storage(object):
    """A simple class to store unique integer pick names."""
    def __init__(self):
        """Create the Storage object."""
        self.number = 0

_s = Storage()

def getPickName():
    """Return a unique integer pick name."""
    _s.number += 1
    return _s.number - 1

class Group(object):
    """A simple class to store any number of renderable objects."""
    def __init__(self):
        """Create the group."""
        self.objects = [[]] #cur obj = -1
        self.obj_dict = {}
        self.all_objs = []
        self.all_names = []

    def add_obj(self, obj):
        """Add a new object to the group."""
        name = getPickName()
        self.objects[-1].append((obj, name))
        if len(self.objects[-1]) >= 512:
            self.objects.append([])
        self.obj_dict[name] = obj
        self.all_objs.append(obj)
        self.all_names.append(name)

    def rem_obj(self, obj):
        """Remove obj from group."""
        try:
            o = self.all_objs.index(obj)
            name = self.all_names[o]

            del self.obj_dict[name]
            for i in self.objects:
                for x in i:
                    if x[0] == obj and x[1] == name:
                        i.remove(x)
            self.all_objs.remove(obj)
            self.all_names.remove(name)
        except:
            print "!@#!@#@#"

    def pick(self, mouse_pos, camera=None):
        """Run Pick512Objects(mouse_pos[0], mouse_pos[1], self.objects, camera) and return results."""
        x, y = mouse_pos

        near = []
        far = []
        names = []
        for objgroup in self.objects:
            if not objgroup and len(self.objects)>1:
                self.objects.remove(objgroup)
                continue
            _n = Pick512Objects(x, y, objgroup, camera)
            for i in _n:
                near.append(i.near)
                far.append(i.far)
                names.append(i.names)

        if near:
            best = names[near.index(min(near))][0]
            return self.obj_dict[best], min(near)
        return None
