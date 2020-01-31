import uwds3_msgs
import numpy as np


class Features(object):
    """Represent a features vector with a confidence"""

    def __init__(self, name, data, confidence):
        """Features constructor"""
        self.name = name
        self.data = np.array(data)
        self.confidence = confidence

    def to_array(self):
        return np.array(self.data, np.float32)

    def to_msg(self):
        """Converts into ROS message"""
        return uwds3_msgs.msg.Features(name=self.name,
                                       data=list(self.data.flatten()),
                                       confidence=self.confidence)
