import uwds3_msgs
import numpy as np


class Features(object):
    """Represent a features vector with a confidence"""

    def __init__(self, name, dimensions, data, confidence):
        """Features constructor"""
        self.name = name
        self.data = np.array(data)
        self.dimensions = dimensions
        self.confidence = confidence

    def to_array(self):
        """ """
        return np.array(self.data, np.float32)

    def from_msg(self, msg):
        """ """
        self.name = msg.name
        self.dimensions = tuple(msg.dimensions)
        self.data = np.array(msg.data).reshape(tuple(self.dimensions))
        return self

    def to_msg(self):
        """Converts into ROS message"""
        return uwds3_msgs.msg.Features(name=self.name,
                                       dimensions=list(self.dimensions),
                                       data=list(self.data.flatten()),
                                       confidence=self.confidence)
