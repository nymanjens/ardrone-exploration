"""
pyggle.particle
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The particle module contains classes for creating and rendering particle effects.
A simple fire effect is included.
"""

from include import *
import data, image, misc, data

import random
import numpy

from misc import randfloat

class Particle3D(object):
    """A simple 3d particle."""
    def __init__(self, parent, behavior):
        """Create the particle.
           parent must be the emitter class creating the particle
           behavior must be the behavior class that will handle how the particle behaves"""
        self.parent = parent
        self.parent.particles.append(self)

        self.extra_data = {}

        self.behavior = behavior
        self.image = self.behavior.image.copy()
        self.behavior.register_particle(self)

        self.age = 0

    def update(self):
        """Update the particle."""
        self.behavior.particle_update(self)

    def render(self, camera):
        """Render the particle.
           camera must be None or the camera object the scene is using"""
        self.update()
        self.image.render(camera)

    def kill(self):
        """Destroy the particle."""
        self.parent.particles.remove(self)

class Emitter3D(object):
    """A simple Particle3D emitter."""
    def __init__(self, behavior, pos=(0,0,0)):
        """Create the emitter.
           behavior must be the behavior class (not instance) that will control how the emitter and particles will behave
           pos must be a three-part tuple of the position of the emitter"""
        self.pos = pos
        self.behavior = behavior(self)
        self.particles = []
        self.particle_type = Particle3D

        self.visible = True
        self.pickable = False
        self.outline = False
        self.outline_size = 4
        self.outline_color=(1,0,0)

    def get_dimensions(self):
        """Return the maximum dimensions (width/height/depth) of the emitter and particles."""
        return self.behavior.get_dimensions()

    def get_pos(self):
        """Return the emitter position."""
        return self.pos

    def get_scale(self):
        """Return the scale of the object."""
        return 1,1,1

    def update(self):
        """Update the emitter."""
        self.behavior.emitter_update()

    def render(self, camera):
        """Render and update all particles.
           camera must be None of the camera the scene is using"""
        self.update()
        for i in self.particles:
            i.render(camera)


class Behavior3D(object):
    """A simple behavior class to control an emitter and particles."""
    def __init__(self, emitter):
        """Create the emitter.
           emitter must be the emitter object that is using this behavior.
           NOTE: this should never be called, the emitter object will do that!"""
        self.emitter = emitter

        self.particle_lifespan = 1
        self.image = image.create_empty_image3d((8,8))
        self.image.pos = self.emitter.pos

    def get_dimensions(self):
        """Calculate and return the maximum dimensions (width/height/depth) of the emitter and particles."""
        #calculate max width, height and depth of particles...
        return 1, 1, 1

    def emitter_update(self):
        """Update the emitter."""
        pass

    def particle_update(self, part):
        """Update a particle."""
        part.age += 1
        if part.age >= self.particle_lifespan:
            part.kill()

    def register_particle(self, part):
        """Register a particle."""
        pass

class Fire3D(Behavior3D):
    """A simple fire behavior for an Emitter3D."""
    def __init__(self, emitter):
        Behavior3D.__init__(self, emitter)

        self.image = image.create_empty_image3d((8,8), (1,.5,0,1))
        self.image.scale = .25
        self.image.pos = self.emitter.pos
        self.particle_lifespan = 20
    __init__.__doc__ = Behavior3D.__init__.__doc__

    def get_dimensions(self):
        return 2, 6, 2 #max/abs(min) directions(x,y,z) * particle_lifespan
    get_dimensions.__doc__ = Behavior3D.get_dimensions.__doc__

    def emitter_update(self):
        for i in xrange(5):
            self.emitter.particle_type(self.emitter, self)
    emitter_update.__doc__ = Behavior3D.emitter_update.__doc__

    def register_particle(self, part):
        dx = randfloat(-.1, .1)
        dy = randfloat(.15, .3)
        dz = randfloat(-.1, .1)

        rot = random.randint(-25, 25)

        part.extra_data["dir"] = (dx, dy, dz)
        part.extra_data["rot"] = rot

        x, y, z = self.emitter.pos

        part.image.pos = x+dx*randfloat(1, 2), y, z+dz*randfloat(1, 2)
    register_particle.__doc__ = Behavior3D.register_particle.__doc__

    def particle_update(self, part):
        Behavior3D.particle_update(self, part)
        x, y, z = part.image.pos
        a, b, c = part.extra_data["dir"]
        x += a
        y += b
        z += c

        b -= .025
        part.extra_data["dir"] = a, b, c
        part.image.pos = x, y, z

        x, y, z = part.image.rotation
        z -= part.extra_data["rot"]
        part.image.rotation = x, y, z

        r, g, b, a = part.image.colorize
        a -= .075
        part.image.colorize = r, g, b, a

        part.image.scale -= .025
    particle_update.__doc__ = Behavior3D.particle_update.__doc__


class ParticlePoint(object):
    """A more complex particle that can be used in a VertexArray powered emitter."""
    def __init__(self, parent, behavior):
        """Create the particle.
           parent must be the emitter class creating the particle
           behavior must be the behavior class that will handle how the particle behaves"""
        self.parent = parent
        self.pos = self.parent.pos
        self.colorize = (1,1,1,1)

        self.index = self.parent.add_particle(self)

        self.extra_data = {}

        self.behavior = behavior
        self.behavior.register_particle(self)

        self.age = 0

    def get_vertex_index(self):
        """Return our unique index from our emitter's vertex array."""
        return self.parent.particles.index(self)

    def kill(self):
        """Kill the particle."""
        self.parent.remove_particle(self)

    def update(self):
        """Update the particle."""
        self.behavior.particle_update(self)
        x, y, z = self.pos
        r, g, b, a = self.colorize

        self.parent.vertex_array.verts[self.index][0] = x
        self.parent.vertex_array.verts[self.index][1] = y
        self.parent.vertex_array.verts[self.index][2] = z

        self.parent.vertex_array.colors[self.index][0] = r
        self.parent.vertex_array.colors[self.index][1] = g
        self.parent.vertex_array.colors[self.index][2] = b
        self.parent.vertex_array.colors[self.index][3] = a

class EmitterPoint(object):
    """A more complex particle emitter, that stores all particles in a vertex array."""
    def __init__(self, behavior, pos=(0,0,0)):
        """Create the emitter.
           behavior must be the behavior class (not instance) that will control how the emitter and particles will behave
           pos must be a three-part tuple of the position of the emitter"""
        self.pos = pos
        self.behavior = behavior(self)
        self.particles = numpy.empty(self.behavior.max_particles, dtype=object)
        self.empty_spaces = []
        self.last_number = 0

        self.vertex_array = data.VertexArray(GL_POINTS, self.behavior.max_particles)

        self.visible = True
        self.pickable = False
        self.outline = False
        self.outline_size = 4
        self.outline_color=(1,0,0)
        self.particle_type = ParticlePoint

    def get_dimensions(self):
        """Return the maximum dimensions (width/height/depth) of the emitter and particles."""
        return self.behavior.get_dimensions()

    def get_pos(self):
        """Return the emitter position."""
        return self.pos

    def get_scale(self):
        """Return the scale of the object."""
        return 1,1,1

    def add_particle(self, part):
        """Add the particle to the vertex array and assign it it's own index."""
        if self.empty_spaces:
            x = self.empty_spaces.pop(0)
            self.particles[x] = part
            return x
        else:
            self.particles[self.last_number] = part
            self.last_number += 1
            return self.last_number - 1

    def remove_particle(self, part):
        """Remove the particle."""
        if part.index+1 == self.last_number:
            self.last_number -= 1
        else:
            self.empty_spaces.append(part.index)
        self.particles[part.index] = None

    def update(self):
        """Update the emitter."""
        self.behavior.emitter_update()

    def render(self, camera):
        """Render and update all particles.
           camera must be None of the camera the scene is using"""
        self.update()
        glPointSize(self.behavior.point_size)
        for i in self.particles:
            if i:
                i.update()
        self.vertex_array.render()


class BehaviorPoint(object):
    """Almost the same as Behavior3D, except also has a max_particles attribute for the size of the vertex array."""
    def __init__(self, emitter):
        """Create the emitter.
           emitter must be the emitter object that is using this behavior.
           NOTE: this should never be called, the emitter object will do that!"""
        self.emitter = emitter

        self.particle_lifespan = 1
        self.max_particles = 2

    def get_dimensions(self):
        """Calculate and return the maximum dimensions (width/height/depth) of the emitter and particles."""
        return 1,1,1

    def emitter_update(self):
        """Update the emitter."""
        pass

    def particle_update(self, part):
        """Update a particle."""
        part.age += 1
        if part.age >= self.particle_lifespan:
            part.kill()

    def register_particle(self, part):
        """Register a particle for us to control."""
        pass

class FirePoint(BehaviorPoint):
    """A more complex fire behavior for an EmitterPoint."""
    def __init__(self, emitter):
        BehaviorPoint.__init__(self, emitter)

        self.particle_lifespan = 20
        self.point_size = 15
        self.max_particles = 105 #self.particle_lifespan * emit rate (5) + 1 cycle of give space - as the emitter runs before the particles die...
    __init__.__doc__ = BehaviorPoint.__init__.__doc__

    def get_dimensions(self):
        return 2, 6, 2 #max/abs(min) directions (x,y,z) of particles * particle_lifespan
    get_dimensions.__doc__ = BehaviorPoint.get_dimensions.__doc__

    def emitter_update(self):
        for i in xrange(5):
            self.emitter.particle_type(self.emitter, self)
    emitter_update.__doc__ = BehaviorPoint.emitter_update.__doc__

    def register_particle(self, part):
        dx = randfloat(-.1, .1)
        dy = randfloat(.15, .3)
        dz = randfloat(-.1, .1)

        part.extra_data["dir"] = (dx, dy, dz)
        part.colorize = (1, 0, 0, 1)

        x, y, z = self.emitter.pos

        part.pos = x + dx * randfloat(1, 1.2), y, z + dz * randfloat(1, 1.2)

        part.colorize = random.choice(((1, 0, 0, 1),
                                       (1, .25, 0, 1),
                                       (1, 1, 0, 1)))
    register_particle.__doc__ = BehaviorPoint.register_particle.__doc__

    def particle_update(self, part):
        BehaviorPoint.particle_update(self, part)

        r, g, b, a = part.colorize
        g += .01
        a -= 1.0/20
        part.colorize = r, g, b, a

        x, y, z = part.pos

        a, b, c = part.extra_data["dir"]
        x += a
        y += b
        z += c

        b -= .01
        part.extra_data["dir"] = a, b, c
        part.pos = x, y, z
    particle_update.__doc__ = BehaviorPoint.particle_update.__doc__
