"""
pyggle.math3d
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The math3d module contains general 3d math functions, as well as collision detection primitives.
"""

import math

def move_with_rotation(pos, rot, amount):
    """Returns a new position that is calculated based on
       the old pos, moved by amount according to rot facing.
       pos is the original position we are moving from,
       rot is the 3d rotation of the object - can be one value or a three part tuple
       amount is how much to move by"""
    try:
        a1, a2, a3 = amount
    except:
        a1 = a2 = a3 = amount
    p=[pos[0],pos[1],pos[2]]
    po=0.0174532925
    p[0] -= math.sin(rot[1]*po)*a1
    p[1] += math.sin(rot[0]*po)*a2
    p[2] += math.cos(rot[1]*po)*a3
    return p

def get_distance(a, b):
    """Return the distance between two points"""
    return Vector(a).distance(Vector(b))

class Vector(object):
    """A simple, 3d Vector class"""
    ctype = "Vector"
    def __init__(self, pos):
        """Create the Vector
           pos must be a three part tuple of the position of the Vector"""
        self.x, self.y, self.z = pos

    def copy(self):
        """Return a copy of the Vector"""
        return Vector((self.x, self.y, self.z))

    def distance(self, other):
        """Return the distance between this Vector and other Vector"""
        n = self - other
        return math.sqrt(n.x**2 + n.y**2 + n.z**2)

    def magnitude(self):
        """Return the magnititude of the Vector"""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def perpendicular(self):
        """Return a perpendicular Vector"""
        return self.cross(Vector((-self.y, self.x, self.z)))

    def fast_distance(self, other):
        """Return the distance between this Vector and other Vector.
           This method is the same as distance, except it does not sqrt the result,
           so the comparison must be squared to be accurate, but it is faster."""
        n = self - other
        return n.x**2 + n.y**2 + n.z**2

    def rotate(self, vec, amount):
        """Return a new Vector that represents this Vector rotated around Vector vec by amount.
           vec must be another Vector
           amount must be a three part tuple representing the rotation amount in 3d space
           Warning - this method seems to be a little buggy, and does not match the other rotations in PYGGEL!"""
        a, b, c = amount

        vec2 = self - vec

        Sin, Cos, Rad = math.sin, math.cos, math.radians

        if a:
            rad = Rad(-a)
            cos = Cos(rad)
            sin = Sin(rad)

            op = self.copy()

            vec2.y = cos * op.y - sin * op.z
            vec2.z = sin * op.y + cos * op.z

        if b:
            rad = Rad(-b)
            cos = Cos(rad)
            sin = Sin(rad)

            op = self.copy()

            vec2.x = cos * op.x - sin * op.z
            vec2.z = sin * op.x + cos * op.z

        if c:
            rad = Rad(-c)
            cos = Cos(rad)
            sin = Sin(rad)

            op = self.copy()

            vec2.x = cos * op.x - sin * op.y
            vec2.y = sin * op.x + cos * op.y

        return vec2 + vec

    def invert(self):
        """Return an inverted Vector"""
        return Vector((-self.x, -self.y, -self.z))

    def length(self):
        """Return the distance of this Vector from (0,0,0)"""
        return self.distance(Vector((0,0,0)))

    def fast_length(self):
        """Return the fast_distance of this Vector from (0,0,0)"""
        return self.fast_distance(Vector((0,0,0)))

    def normalize(self):
        """Return a normalized Vector"""
        L = self.length()
        return self / Vector((L, L, L))

    def dot(self, other):
        """Return the dot product between this Vector and other Vector"""
        x = self * other
        return x.x + x.y + x.z

    def get_pos(self):
        """Return the position of this Vector as a tuple"""
        return self.x, self.y, self.z

    def set_pos(self, pos):
        """Set the position of this Vector from a tuple"""
        self.x, self.y, self.z = pos

    def angle(self, other):
        """Return the angle between this vector and another"""
        return math.acos(self.dot(other))

    def __sub__(self, other):
        """Return a Vector representing this Vector subtracting other Vector"""
        return Vector((self.x-other.x, self.y-other.y, self.z-other.z))

    def __add__(self, other):
        """Return a Vector representing this Vector adding other Vector"""
        return Vector((self.x+other.x, self.y+other.y, self.z+other.z))

    __radd__ = __add__

    def __mul__(self, other):
        """Return a Vector representing this Vector multiplying other Vector"""
        return Vector((self.x*other.x, self.y*other.y, self.z*other.z))
    __rmul__ = __mul__

    def __div__(self, other):
        """Return a Vector representing this Vector divided by other Vector"""
        x = self.x/other.x if (self.x and other.x) else 0
        y = self.y/other.y if (self.y and other.y) else 0
        z = self.z/other.z if (self.z and other.z) else 0
        return Vector((x, y, z))

    def __eq__(self, other):
        """Return whether this Vector is at the same position as other Vector"""
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __ne__(self, other):
        """Return whether this Vector is not at the same position as other Vector"""
        return not self == other

    def __nonzero__(self):
        """Return whether this Vector is not at (0,0,0)"""
        return self != Vector((0,0,0))

    def __iadd__(self, other):
        """Adds other Vector to this Vector"""
        self.set_pos((self + other).get_pos())

    def __rsub__(self, other):
        """Returns a Vector representing other Vector subtracting this Vector"""
        return Vector(*(other - self).get_pos())

    def __isub__(self, other):
        """Subtracts other Vector from this Vector"""
        self.set_pos((self - other).get_pos())

    def __imul__(self, other):
        """Multiplies this Vector by other Vector"""
        self.set_pos((self * other).get_pos())

    def __rdiv__(self, other):
        """Returns a Vector representing other Vector divided by this Vector"""
        return Vector((other / self).get_pos())
    def __idiv__(self, other):
        """Divides this Vector by other Vector"""
        self.set_pos((self / other).get_pos())

    __neg__ = invert

    def __abs__(self):
        """Returns a Vector representing this Vector's absolute position"""
        return Vector((abs(self.x), abs(self.y), abs(self.z)))

    def __pow__(self, other):
        """Return a Vector representing this Vector raised to other Vector"""
        return Vector((self.x**other.x, self.y**other.y, self.z**other.z))

    def __rpow__(self, other):
        """Return other Vector raised to this Vector"""
        return other ** self

    def cross(self, other):
        """Return the cross product between this Vector and other Vector"""
        return Vector((self.y * other.z - self.z * other.y,
                      self.z * other.x - self.x * other.z,
                      self.x * other.y - self.y * other.x))

    def collide(self, other):
        """Return whether this Vector collides with another object"""
        if other.ctype == "Vector":
            return self == other
        else:
            return other.collide(self)

    def get_plane_distance(self, plane):
        return plane[0] * self.x + plane[1] * self.y +\
               plane[2] * self.z + plane[3]

    def in_frustum(self, frustum):
        for plane in frustum:
            if self.get_plane_distance(plane) < 0:
                return False
        return True

class Sphere(Vector):
    """A simple Sphere object - same as Vector except has a radius"""
    ctype = "Sphere"
    def __init__(self, pos, radius):
        """Create the Sphere
           pos must be a three part tuple of the position of the Sphere
           radius must be a positive number"""
        Vector.__init__(self, pos)
        self.radius = radius
        self.scale = 1

    def collide(self, other):
        """Return whether this Sphere is colliding with another object"""
        r = self.radius * self.scale
        if other.ctype == "Vector":
            return other.fast_distance(self) <= r ** 2 #this so we avoid the sqrt call ;)
        elif other.ctype == "Sphere":
            return other.fast_distance(self) <= (r + other.radius * other.scale) ** 2
        else:
            return other.collide(self)

    def in_frustum(self, frustum):
        for plane in frustum:
            if self.get_plane_distance(plane) < -self.radius:
                return False
        return True

class AABox(Vector):
    """A simple, axis-aligned Cube object - same as Vector except has a size(width/height/depth)"""
    ctype = "AABox"
    def __init__(self, pos, size):
        """Create the AABox
           pos must be a three part tuple of the position of the AABox
           size must be either a number representing the size of the AABox - if all sides are equal,
               otherwise, must be a three-part tuple representing the size of each direction of the AABox"""
        Vector.__init__(self, pos)

        try:
            self.width, self.height, self.depth = size
        except:
            self.width = self.height = self.depth = size

        self.scale = 1,1,1

    def collide(self, other):
        """Return whether this AABox is colliding with another object"""

        width = self.width / 2 * self.scale[0]
        height = self.height / 2 * self.scale[1]
        depth = self.depth / 2 * self.scale[2]

        left = self.x - width
        right = self.x + width
        bottom = self.y - height
        top = self.y + height
        front = self.z - depth
        back = self.z + depth

        if other.ctype == "Vector":
            return left <= other.x <= right and\
                   bottom <= other.y <= top and\
                   front <= other.z <= back
        elif other.ctype == "Sphere":
            r = other.radius
            return left -r <= other.x <= right + r and\
                   bottom -r <= other.y <= top + r and\
                   front - r <= other.z <= back + r
        elif other.ctype == "AABox":
            points = ((left, bottom, front),
                      (right, bottom, front),
                      (right, top, front),
                      (left, top, front),
                      (left, bottom, back),
                      (right, bottom, back),
                      (right, top, back),
                      (left, top, back))
            for i in points:
                if other.collide(Vector(i)):
                    return True

            #test them against us now...
            width = other.width / 2 * other.scale[0]
            height = other.height / 2 * other.scale[1]
            depth = other.depth / 2 * other.scale[2]

            left = other.x - width
            right = other.x + width
            bottom = other.y - height
            top = other.y + height
            front = other.z - depth
            back = other.z + depth
            points = ((left, bottom, front),
                      (right, bottom, front),
                      (right, top, front),
                      (left, top, front),
                      (left, bottom, back),
                      (right, bottom, back),
                      (right, top, back),
                      (left, top, back))

            for i in points:
                if self.collide(Vector(i)):
                    return True
            return False
        else:
            return other.collide(self)

def calcTriNormal(t1, t2, t3, flip=False):
    """Return a normal for lighting based on 3 points.
       Code provided by Ian Mallet."""
    v1 = t2[0]-t1[0],t2[1]-t1[1],t2[2]-t1[2]
    v2 = t3[0]-t1[0],t3[1]-t1[1],t3[2]-t1[2]
    vx = (v1[1] * v2[2]) - (v1[2] * v2[1])
    vy = (v1[2] * v2[0]) - (v1[0] * v2[2])
    vz = (v1[0] * v2[1]) - (v1[1] * v2[0])
    if flip:
        vx = -vx
        vy = -vy
        vz = -vz
    return (vx, vy, vz)
