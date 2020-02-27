import numpy as np
import cv2
from .vector3d import Vector3D


class Vector3DStable(Vector3D):
    """Represents a 3D vector stabilized"""
    def __init__(self,
                 x=.0, y=.0, z=.0,
                 vx=.0, vy=.0, vz=.0,
                 ax=.0, ay=.0, az=.0,
                 vmax=0.0001, amax=0.00001,
                 p_cov=.03, m_cov=.01, use_accel=True):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vz
        self.vz = vz
        self.vmax = vmax
        self.amax = amax
        self.use_accel = use_accel
        if self.use_accel is True:
            self.ax = ax
            self.ay = ay
            self.az = az
            self.filter = cv2.KalmanFilter(9, 3)
        else:
            self.ax = .0
            self.ay = .0
            self.az = .0
            self.filter = cv2.KalmanFilter(6, 3)

        self.filter.statePost = self.to_array()
        self.use_accel = use_accel
        if self.use_accel is True:
            self.filter.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                      [0, 1, 0, 0, 0, 0, 0, 0, 0],
                                                      [0, 0, 1, 0, 0, 0, 0, 0, 0]], np.float32)
        else:
            self.filter.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                                      [0, 1, 0, 0, 0, 0],
                                                      [0, 0, 1, 0, 0, 0]], np.float32)
        self.__update_noise_cov(p_cov, m_cov)
        self.last_update = cv2.getTickCount()

    def from_array(self, array):
        """ """
        if self.use_accel is True:
            assert array.shape == (9, 1)
        else:
            assert array.shape == (6, 1)
        self.x = array[0][0]
        self.y = array[1][0]
        self.z = array[2][0]
        self.vx = array[3][0]
        self.vy = array[4][0]
        self.vz = array[5][0]
        if self.use_accel is True:
            self.ax = array[6][0]
            self.ay = array[7][0]
            self.az = array[8][0]
        self.filter.statePost = array
        self.filter.statePre = self.filter.statePost
        return self

    def to_array(self):
        """ """
        if self.use_accel is True:
            return np.array([[self.x],
                             [self.y],
                             [self.z],
                             [self.vx],
                             [self.vy],
                             [self.vz],
                             [self.ax],
                             [self.ay],
                             [self.az]], np.float32)
        else:
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

    def acceleration(self):
        """ """
        return Vector3D(x=self.ax, y=self.ay, z=self.az)

    def update(self, x, y, z):
        """Updates/Filter the 3D vector"""
        self.__update_time()
        self.predict()
        measurement = np.array([[x], [y], [z]], np.float32)
        measurement = measurement.flatten().reshape((3, 1)) # ugly fix
        assert measurement.shape == (3, 1)
        self.filter.correct(measurement)
        self.from_array(self.filter.statePost)

    def predict(self):
        """Predicts the 3D vector based on motion model"""
        self.__update_time()
        self.filter.predict()
        x = self.filter.statePost[0][0]
        y = self.filter.statePost[1][0]
        z = self.filter.statePost[2][0]
        vx = self.filter.statePost[3][0]
        vy = self.filter.statePost[4][0]
        vz = self.filter.statePost[5][0]
        vx = self.vmax if vx > self.vmax else vx
        vy = self.vmax if vy > self.vmax else vy
        vz = self.vmax if vz > self.vmax else vz
        if self.use_accel is True:
            ax = self.filter.statePost[6][0]
            ay = self.filter.statePost[7][0]
            az = self.filter.statePost[8][0]
            ax = self.amax if ax > self.amax else ax
            ay = self.amax if ay > self.amax else ay
            az = self.amax if az > self.amax else az
            state = np.array([[x],[y],[z],[vx],[vy],[vz],[ax],[ay],[az]], dtype=np.float32)
        else:
            state = np.array([[x],[y],[z],[vx],[vy],[vz]], dtype=np.float32)
        self.from_array(state)

    def __update_noise_cov(self, p_cov, m_cov):
        """Updates the process and measurement covariances"""
        if self.use_accel is True:
            self.filter.processNoiseCov = np.eye(9, dtype=np.float32) * p_cov
        else:
            self.filter.processNoiseCov = np.eye(6, dtype=np.float32) * p_cov

        self.filter.measurementNoiseCov = np.eye(3, dtype=np.float32) * m_cov

    def __update_transition(self, dt):
        if self.use_accel is True:
            a = 0.5*dt*dt
            self.filter.transitionMatrix = np.array([[1, 0, 0, dt, 0, 0, a, 0, 0],
                                                     [0, 1, 0, 0, dt, 0, 0, a, 0],
                                                     [0, 0, 1, 0, 0, dt, 0, 0, a],
                                                     [0, 0, 0, 1, 0, 0, dt, 0, 0],
                                                     [0, 0, 0, 0, 1, 0, 0, dt, 0],
                                                     [0, 0, 0, 0, 0, 1, 0, 0, dt],
                                                     [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 1, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1]], np.float32)
        else:
            self.filter.transitionMatrix = np.array([[1, 0, 0, dt, 0, 0],
                                                     [0, 1, 0, 0, dt, 0],
                                                     [0, 0, 1, 0, 0, dt],
                                                     [0, 0, 0, 1, 0, 0],
                                                     [0, 0, 0, 0, 1, 0],
                                                     [0, 0, 0, 0, 0, 1]], np.float32)

    def __update_time(self):
        now = cv2.getTickCount()
        elapsed_time = (now - self.last_update)/cv2.getTickFrequency()
        self.last_update = now
        self.__update_transition(elapsed_time)

    def __str__(self):
        return("3d vector stable: {}".format(self.to_array().flatten()))
