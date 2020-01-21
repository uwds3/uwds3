import numpy as np
import cv2
import geometry_msgs
from tf.transformations import translation_matrix, euler_matrix, euler_from_matrix
from tf.transformations import translation_from_matrix, quaternion_from_matrix
from tf.transformations import euler_from_quaternion
from .vector3d import Vector3D


class Vector6D(object):
    """Represents a 6D pose (position + orientation)"""
    def __init__(self, x=.0, y=.0, z=.0,
                 rx=.0, ry=.0, rz=.0):
        """Vector 6D constructor"""
        self.position = Vector3D(x=x, y=y, z=z)
        self.rotation = Vector3D(x=rx, y=ry, z=rz)

    def from_array(self, array):
        """Set the 6D vector"""
        assert len(array) == 6
        self.position.x = array[0]
        self.position.y = array[1]
        self.position.z = array[2]
        self.rotation.x = array[3]
        self.rotation.y = array[4]
        self.rotation.z = array[5]

    def to_array(self):
        """Returns the 6D vector's array representation"""
        return np.array([self.position.x, self.position.y, self.position.z,
                         self.rotation.x, self.rotation.y, self.rotation.z])

    def from_transform(self, transform):
        """Set the vector from an homogenous transform"""
        r = euler_from_matrix(transform, "rxyz")
        t = translation_from_matrix(transform)
        self.position.x = t[0]
        self.position.y = t[1]
        self.position.z = t[2]
        self.rotation.x = r[0]
        self.rotation.y = r[1]
        self.rotation.z = r[2]

    def transform(self):
        """Returns the homogenous transform"""
        mat_pos = translation_matrix(self.position.to_array())
        mat_rot = euler_matrix(self.rotation.x,
                               self.rotation.y,
                               self.rotation.z, "rxyz")
        return np.dot(mat_pos, mat_rot)

    def inv(self):
        """Inverse the vector"""
        return Vector6D().from_transform(np.linalg.inv(self.transform()))

    def from_quaternion(self, rx, ry, rz, rw):
        euler = euler_from_quaternion([rx, ry, rz, rw])
        self.rotation.x = euler[0]
        self.rotation.y = euler[1]
        self.rotation.z = euler[2]

    def quaternion(self):
        """Returns the rotation quaternion"""
        return quaternion_from_matrix(self.transform())

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
        msg.position.x = self.position.x
        msg.position.y = self.position.y
        msg.position.z = self.position.z
        q = self.quaternion()
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.x = q[3]
        return msg
