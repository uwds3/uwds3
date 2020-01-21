import numpy as np
import geometry_msgs
from .vector2d import Vector2D


class Vector3D(object):
    """Represents a 3D vector"""
    def __init__(self, x=.0, y=.0, z=.0):
        """Vector 3D constructor"""
        self.x = x
        self.y = y
        self.z = z

    def from_array(self, array):
        """ """
        assert array.shape == (3, 1)
        self.x = array[0]
        self.y = array[1]
        self.z = array[2]

    def to_array(self):
        """Returns the 3D point's array representation"""
        return np.array([self.x, self.y, self.z])

    def __len__(self):
        """Returns the lenght of the vector"""
        return 3

    def __add__(self, vector):
        """Adds the given 2D vector"""
        assert len(vector) == 3
        return Vector2D(x=self.x+vector.x,
                        y=self.y+vector.y,
                        z=self.z+vector.z)

    def __sub__(self, vector):
        """Subtracts the given 2D vector"""
        assert len(vector) == 3
        return Vector2D(x=self.x-vector.x,
                        y=self.y-vector.y,
                        z=self.z-vector.z)

    def to_msg(self):
        """Converts to ROS message"""
        return geometry_msgs.msg.Vector3(x=self.x, y=self.y, z=self.z)