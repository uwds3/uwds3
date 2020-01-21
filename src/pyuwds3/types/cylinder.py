from math import pi
from .vector.vector3d import Vector3D


class Cylinder(object):
    """Represents a 2D BoundingBox + depth in the world space (e.g. cylinder)"""
    def __init__(self, w, h,
                 x=.0, y=.0, z=.0,
                 rx=.0, ry=.0, rz=.0):
        """Cylinder constructor"""
        self.position = Vector3D(x=x, y=y, z=z)
        self.rotation = Vector3D(x=rx, y=ry, z=rz)
        self.w = w
        self.h = h

    def center(self):
        """Returns the bbox's center in pixels"""
        return self.position

    def radius(self):
        """Returns the cylinder's radius in meters"""
        return self.width()/2.0

    def width(self):
        """Returns the cylinder's width in meters"""
        return self.w

    def height(self):
        """Returns the cylinder's height in meters"""
        return self.h

    def area(self):
        """Returns the cylinder's area in cube meters"""
        return 2.0*pi*self.radius()*self.height()
