"""
pyggle.scene
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The scene module contains classes used to represent an entire group of renderable objects.
"""

from include import *
import camera, view, misc
from light import all_lights

class Tree(object):
    """A simple class used to keep track of all objects in a scene."""
    def __init__(self):#, hs, ps):
        """Create the Tree."""
        self.render_3d = []
        self.render_3d_blend = []
        self.render_2d = []
        self.render_3d_always = []
        self.skybox = None
        self.lights = []

class Scene(object):
    """A simple scene class used to store, render, pick and manipulate objects."""
    def __init__(self):
        """Create the scene."""
        self.graph = Tree()

        self.render2d = True
        self.render3d = True

        self.render_buffer = None

        self.pick = False #can be true or false

    def render(self, camera=None, pick_pos=None):
        """Render all objects.
           camera must no or the camera object used to render the scene
           Returns None or picked object if Scene.pick is True and an object is actually touching the mouse."""
        if self.render_buffer:
            self.render_buffer.enable()
        else:
            view.set3d()
        pick = None

        if pick_pos == None:
            mpx, mpy = view.screen.get_mouse_pos()
            mpy = view.screen.screen_size[1] - mpy
        else:
            mpx, mpy = pick_pos
            mpy = view.screen.screen_size[1] - mpy
        last_depth = 1

        my_lights = list(all_lights)
        if self.graph.skybox and camera:
            self.graph.skybox.render(camera)
        if self.render3d:
            if camera:
                camera.push()
            for i in self.graph.lights:
                i.gl_light = my_lights.pop()
                i.shine()
            glEnable(GL_ALPHA_TEST)
            for i in self.graph.render_3d:
                if i.visible:
                    i.render(camera)
                    if self.pick and i.pickable:
                        dep = glReadPixelsf(mpx, mpy, 1, 1, GL_DEPTH_COMPONENT)[0][0]
                        if dep < last_depth:
                            last_depth = dep
                            pick = i

            glDisable(GL_ALPHA_TEST)
            r, g, b, a = glReadPixelsf(mpx, mpy, 1, 1, GL_RGBA)[0][0]
            last_color = r,g,b,a
            glDepthMask(GL_FALSE)
            for i in self.graph.render_3d_blend:
                if i.visible:
                    i.render(camera)
                    if self.pick and i.pickable:
                        r, g, b, a = glReadPixelsf(mpx, mpy, 1, 1, GL_RGBA)[0][0]
                        col = r,g,b,a
                        if col != last_color:
                            last_color = col
                            pick = i
            glDepthMask(GL_TRUE)
            glDisable(GL_DEPTH_TEST)
            for i in self.graph.render_3d_always:
                if i.visible:
                    i.render(camera)
                    if self.pick and i.pickable:
                        r, g, b, a = glReadPixelsf(mpx, mpy, 1, 1, GL_RGBA)[0][0]
                        col = r,g,b,a
                        if col != last_color:
                            last_color = col
                            pick = i
            glEnable(GL_DEPTH_TEST)

            for i in self.graph.lights:
                i.hide()
            if camera:
                camera.pop()

        if self.render2d:
            view.set2d()
            glPushMatrix()
            rx = 1.0 * view.screen.screen_size[0] / view.screen.screen_size_2d[0]
            ry = 1.0 * view.screen.screen_size[1] / view.screen.screen_size_2d[1]
            glScalef(rx, ry, 1)
            glDisable(GL_LIGHTING)
            for i in self.graph.render_2d:
                if i.visible: i.render()
            if view.screen.lighting:
                glEnable(GL_LIGHTING)
            glPopMatrix()

        if self.render_buffer:
            self.render_buffer.disable()

        return pick

    def add_2d(self, ele):
        """Add a 2d object or list of objects to the scene."""
        if not hasattr(ele, "__iter__"):
            ele = [ele]
        for i in ele:
            self.graph.render_2d.append(i)
            i.scene = self

    def remove_2d(self, ele):
        """Remove a 2d object from the scene."""
        self.graph.render_2d.remove(ele)

    def add_3d(self, ele):
        """Add a 3d, non-blended, depth-tested object or list of objects to the scene."""
        if not hasattr(ele, "__iter__"):
            ele = [ele]
        for i in ele:
            self.graph.render_3d.append(i)

    def remove_3d(self, ele):
        """Remove a 3d object from the scene."""
        self.graph.render_3d.remove(ele)

    def add_3d_blend(self, ele):
        """Add a 3d, blended, depth-tested object or list of objects to the scene."""
        if not hasattr(ele, "__iter__"):
            ele = [ele]
        for i in ele:
            self.graph.render_3d_blend.append(i)

    def remove_3d_blend(self, ele):
        """Remove a 3d blended object from the scene."""
        self.graph.render_3d_blend.remove(ele)

    def add_3d_always(self, ele):
        """Add a 3d, blended, non-depth-tested (always visible) object or list of objects to the scene."""
        if not hasattr(ele, "__iter__"):
            ele = [ele]
        for i in ele:
            self.graph.render_3d_always.append(i)

    def remove_3d_always(self, ele):
        """Remove a 3d always visible obejct from the scene."""
        self.graph.render_3d_always.remove(ele)

    def add_skybox(self, ele=None):
        """Add a Skybox or Skyball object to the scene.
           If None is given, disables skybox."""
        self.graph.skybox = ele

    def add_light(self, light):
        """Add a light to the scene."""
        if len(self.graph.lights) < 8:
            self.graph.lights.append(light)
        else:
            raise ValueError("Too many Lights - max 8")

    def remove_light(self, light):
        """Remove a light from the scene."""
        if light in self.graph.lights:
            self.graph.lights.remove(light)
