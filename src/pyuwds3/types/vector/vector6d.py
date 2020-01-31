import numpy as np
import math
import cv2
import geometry_msgs
from tf.transformations import translation_matrix, euler_matrix, euler_from_matrix
from tf.transformations import translation_from_matrix, quaternion_from_matrix
from tf.transformations import quaternion_from_euler, euler_from_quaternion, unit_vector
from .vector3d import Vector3D


class Vector6D(object):
    """Represents a 6D pose (position + orientation)"""
    def __init__(self, x=.0, y=.0, z=.0,
                 rx=.0, ry=.0, rz=.0):
        """Vector 6D constructor"""
        self.pos = Vector3D(x=x, y=y, z=z)
        self.rot = Vector3D(x=rx, y=ry, z=rz)

    def position(self):
        """ """
        return self.pos

    def rotation(self):
        """ """
        return self.rot

    def from_array(self, array):
        """Set the 6D vector"""
        assert len(array) == 6
        self.pos.x = array[0]
        self.pos.y = array[1]
        self.pos.z = array[2]
        self.rot.x = array[3]
        self.rot.y = array[4]
        self.rot.z = array[5]
        return self

    def to_array(self):
        """Returns the 6D vector's array representation"""
        return np.array([[self.pos.x], [self.pos.y], [self.pos.z],
                        [self.rot.x], [self.rot.y], [self.rot.z]])

    def from_transform(self, transform):
        """Set the vector from an homogenous transform"""
        r = euler_from_matrix(transform, "rxyz")
        t = translation_from_matrix(transform)
        self.pos.x = t[0]
        self.pos.y = t[1]
        self.pos.z = t[2]
        self.rot.x = r[0]
        self.rot.y = r[1]
        self.rot.z = r[2]
        return self

    def transform(self):
        """Returns the homogenous transform"""
        mat_pos = translation_matrix(self.pos.to_array().flatten()[:3])
        mat_rot = euler_matrix(self.rot.x,
                               self.rot.y,
                               self.rot.z, "rxyz")
        return np.dot(mat_pos, mat_rot)

    def inv(self):
        """Inverse the 6d vector"""
        return Vector6D().from_transform(np.linalg.inv(self.transform()))

    def from_quaternion(self, rx, ry, rz, rw):
        euler = euler_from_quaternion([rx, ry, rz, rw], "rxyz")
        self.rot.x = euler[0]
        self.rot.y = euler[1]
        self.rot.z = euler[2]
        return self

    def quaternion(self):
        """Returns the rotation quaternion"""
        q = quaternion_from_euler(self.rot.x, self.rot.y, self.rot.z, "rxyz")
        q /= math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        return q

    def __len__(self):
        """Returns the vector's lenght"""
        return 6

    def __add__(self, vector):
        """Adds the given vector"""
        return Vector6D().from_transform(np.dot(self.transform(), vector.transform()))

    def __sub__(self, vector):
        """Substracts the given vector"""
        return Vector6D().from_transform(np.dot(self.transform(), vector.inv().transform()))

    def to_msg(self):
        """Converts to ROS message"""
        msg = geometry_msgs.msg.Pose()
        msg.position.x = self.pos.x
        msg.position.y = self.pos.y
        msg.position.z = self.pos.z
        q = self.quaternion()
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.x = q[3]
        return msg

    def __str__(self):
        return("6d vector : {}".format(self.to_array().flatten()))
