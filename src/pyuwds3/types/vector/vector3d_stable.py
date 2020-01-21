import numpy as np
import cv2
from .vector3d import Vector3D


class Vector3DStable(Vector3D):
    """Represents a 3D vector stabilized"""
    def __init__(self, x=.0, y=.0, z=.0,
                 vx=.0, vy=.0, vz=.0,
                 p_cov=0.01, m_cov=0.1):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vz
        self.vz = vz
        self.filter = cv2.KalmanFilter(6, 3, 0)
        self.filter.statePost = self.to_array()
        self.filter.statePre = self.filter.statePost
        self.filter.transitionMatrix = np.array([[1, 0, 0, 1, 0, 0],
                                                 [0, 1, 0, 0, 1, 0],
                                                 [0, 0, 1, 0, 0, 1],
                                                 [0, 0, 0, 1, 0, 0],
                                                 [0, 0, 0, 0, 1, 0],
                                                 [0, 0, 0, 0, 0, 1]], np.float32)

        self.filter.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                                  [0, 1, 0, 0, 0, 0],
                                                  [0, 0, 1, 0, 0, 0]], np.float32)
        self.update_cov(p_cov, m_cov)

    def from_array(self, array):
        """ """
        assert array.shape == (6, 1)
        self.x = array[0]
        self.y = array[1]
        self.z = array[2]
        self.vx = array[3]
        self.vy = array[4]
        self.vz = array[5]
        self.filter.statePost = array
        self.filter.statePre = self.filter.statePost

    def to_array(self):
        """ """
        return np.array([[self.x],
                         [self.y],
                         [self.z],
                         [self.vx],
                         [self.vy],
                         [self.vz]], np.float32)

    def position(self):
        """ """
        return Vector3D(x=self.x, y=self.y, z=self.z)

    def velocity(self):
        """ """
        return Vector3D(x=self.vx, y=self.vy, z=self.vz)

    def update(self, x, y, z):
        """Updates/Filter the 3D vector"""
        self.filter.predict()
        self.filter.correct(np.array([[np.float32(x)],
                                      [np.float32(y)],
                                      [np.float32(z)]]))
        self.from_array(self.filter.statePost)

    def predict(self):
        """Predicts the 3D vector based on motion model"""
        self.filter.predict()
        self.from_array(self.filter.statePost)

    def update_cov(self, p_cov, m_cov):
        """Updates the process and measurement covariances"""
        self.filter.processNoiseCov = np.array([[1, 0, 0, 0, 0, 0],
                                                [0, 1, 0, 0, 0, 0],
                                                [0, 0, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 0, 0],
                                                [0, 0, 0, 0, 1, 0],
                                                [0, 0, 0, 0, 0, 1]], np.float32) * p_cov

        self.filter.measurementNoiseCov = np.array([[1, 0, 0, 0, 0, 0],
                                                    [0, 1, 0, 0, 0, 0],
                                                    [0, 0, 1, 0, 0, 0]], np.float32) * m_cov
