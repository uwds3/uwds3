import numpy as np
from .vector3d_stable import Vector3DStable


class Vector6DStable(Vector6D):
    """ """
    def __init__(self, x=.0, y=.0, z=.0,
                 vx=.0, vy=.0, vz=.0,
                 rx=.0, ry=.0, rz=.0,
                 vrx=.0, vry=.0, vrz=.0,
                 p_cov=0.01, m_cov=0.1):
        """ """
        self.position = Vector3DStabilized(x=x, y=y, z=z,
                                           vx=vx, vy=vy, vz=vz,
                                           p_cov=p_cov, m_cov=m_cov)

        self.rotation = Vector3DStabilized(x=rx, y=ry, z=rz,
                                           vx=vrx, vy=vry, vz=vrz,
                                           p_cov=p_cov, m_cov=m_cov)

    def from_array(self, array):
        """ """
        assert array.shape == (12, 1)
        self.position.from_array(array[:6])
        self.rotation.from_array(array[6:])

    def to_array(self):
        """ """
        return np.concatenate(self.positon.to_array(),
                              self.rotation.to_array(),
                              axis=0)
