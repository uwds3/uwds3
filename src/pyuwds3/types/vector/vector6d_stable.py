import numpy as np
from .vector6d import Vector6D
from .vector3d_stable import Vector3DStable


class Vector6DStable(Vector6D):
    """ """
    def __init__(self, x=.0, y=.0, z=.0,
                 vx=.0, vy=.0, vz=.0,
                 rx=.0, ry=.0, rz=.0,
                 vrx=.0, vry=.0, vrz=.0,
                 p_cov=.03, m_cov=.028):
        """ """
        self.pos = Vector3DStable(x=x, y=y, z=z,
                                  vx=vx, vy=vy, vz=vz,
                                  p_cov=p_cov, m_cov=m_cov, use_accel=True)

        self.rot = Vector3DStable(x=rx, y=ry, z=rz,
                                  vx=vrx, vy=vry, vz=vrz,
                                  p_cov=p_cov, m_cov=m_cov, use_accel=True)

    def position(self):
        """ """
        return self.pos.position()

    def rotation(self):
        """ """
        return self.rot.position()

    def linear_velocity(self):
        """ """
        return self.pos.velocity()

    def angular_velocity(self):
        """ """
        return self.rot.velocity()

    def linear_acceleration(self):
        """ """
        return self.pos.acceleration()

    def angular_acceleration(self):
        """ """
        return self.rot.acceleration()

    def __add__(self, vector):
        """Adds the given vector"""
        return Vector6D().from_transform(np.dot(self.transform(), vector.transform()))

    def __sub__(self, vector):
        """Substracts the given vector"""
        return Vector6D().from_transform(np.dot(self.transform(), vector.inv().transform()))

    def from_array(self, array):
        """ """
        assert array[:9][0].shape == (9, 1)
        assert array[9:][0].shape == (9, 1)
        self.pos.from_array(array[:9][0])
        self.rot.from_array(array[9:][0])

    def to_array(self):
        """ """
        return np.array((self.pos.to_array(), self.rot.to_array()))

    def __str__(self):
        return("6d vector stable: {}".format(self.to_array().flatten()))
