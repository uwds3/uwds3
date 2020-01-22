import numpy as np
import cv2


class ScalarStable(object):
    """Represents a stabilized scalar"""
    def __init__(self, x=.0, vx=.0, dt=0.066, p_cov=100, m_cov=.2):
        """ScalarStabilized constructor"""
        self.x = x
        self.vx = vx
        self.filter = cv2.KalmanFilter(2, 1)
        self.filter.statePost = self.to_array()
        self.filter.statePre = self.filter.statePost
        self.filter.transitionMatrix = np.array([[1, dt],
                                                 [0, 1]], np.float32)
        self.filter.measurementMatrix = np.array([[1, 1]], np.float32)
        self.update_cov(p_cov, m_cov)

    def from_array(self, array):
        """Updates the scalar stabilized state from array"""
        assert array.shape == (2, 1)
        self.x = array[0]
        self.vx = array[1]
        self.filter.statePre = self.filter.statePost

    def to_array(self):
        """Returns the scalar stabilizer state array representation"""
        return np.array([[self.x], [self.vx]], np.float32)

    def position(self):
        """Returns the scalar's position"""
        return self.x

    def velocity(self):
        """Returns the scalar's velocity"""
        return self.vx

    def update(self, x):
        """Updates/Filter the scalar"""
        self.filter.predict()
        measurement = np.array([[np.float32(x)]])
        assert measurement.shape == (1, 1)
        self.filter.correct(measurement)
        self.from_array(self.filter.statePost)

    def predict(self):
        """Predicts the scalar state"""
        self.filter.predict()
        self.from_array(self.filter.statePost)

    def update_cov(self, p_cov, m_cov):
        """Updates the process and measurement covariances"""
        self.filter.processNoiseCov = np.array([[1, 0],
                                                [0, 1]], np.float32) * p_cov

        self.filter.measurementNoiseCov = np.array([[1]], np.float32) * m_cov

    def __len__(self):
        return 1

    def __add__(self, scalar):
        return self.x + scalar.x

    def __sub__(self, scalar):
        return self.x - scalar.x

    def __str__(self):
        return("scalar stable: {}".format(self.to_array().flatten()))
