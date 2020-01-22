import uwds3_msgs
from .vector.vector6d import Vector6D


class NodeState:
    PERCEIVED = 0
    OCCLUDED = 1


class SceneNode(object):
    def __init__(self, type, x=.0, y=.0, z=.0,
                 rx=.0, ry=.0, rz=.0):
        """
        """
        self.uuid
        self.label
        self.state = NodeState.PERCEIVED
        self.pose = Vector6D(x=x, y=y, z=z,
                             rx=rx, ry=ry, rz=rz)

    def to_msg(self):
        """
        """
        msg = uwds3_msgs.msg.SceneNode()
        return msg
