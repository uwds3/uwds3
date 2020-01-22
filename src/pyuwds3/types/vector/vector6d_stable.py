import numpy as np
from .vector6d import Vector6D
from .vector3d_stable import Vector3DStable


class Vector6DStable(Vector6D):
    """ """
    def __init__(self, x=.0, y=.0, z=.0,
                 vx=.0, vy=.0, vz=.0,
                 rx=.0, ry=.0, rz=.0,
                 vrx=.0, vry=.0, vrz=.0,
                 dt=0.066, p_cov=100, m_cov=.001):
        """ """
        self.pos = Vector3DStable(x=x, y=y, z=z,
                                  vx=vx, vy=vy, vz=vz,
                                  dt=dt, p_cov=p_cov, m_cov=m_cov)

        self.rot = Vector3DStable(x=rx, y=ry, z=rz,
                                  vx=vrx, vy=vry, vz=vrz,
                                  dt=dt, p_cov=p_cov, m_cov=m_cov)

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

    def linear_acceleration(self):
        """ """
        return self.rot.acceleration()

    def from_array(self, array):
        """ """
        assert array[:9][0] == (9, 1)
        assert array[9:][0] == (9, 1)
        self.pos.from_array(array[:9][0])
        self.rot.from_array(array[9:][0])

    def to_array(self):
        """ """
        return np.array((self.pos.to_array(), self.rot.to_array()))

    def __str__(self):
        return("6d vector stable: {}".format(self.to_array().flatten()))
