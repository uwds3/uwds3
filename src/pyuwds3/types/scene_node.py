import uuid
import cv2
import uwds3_msgs
from .camera import Camera
from .vector.vector6d_stable import Vector6DStable


class SceneNodeType(object):
    OBJECT = uwds3_msgs.msg.SceneNode.OBJECT
    AGENT = uwds3_msgs.msg.SceneNode.AGENT


class SceneNode(object):
    def __init__(self, type,
                 x=.0, y=.0, z=.0,
                 rx=.0, ry=.0, rz=.0,
                 vx=.0, vy=.0, vz=.0,
                 vrx=.0, vry=.0, vrz=.0,
                 ax=.0, ay=.0, az=.0,
                 arx=.0, ary=.0, arz=.0):
        """
        """
        self.id = str(uuid.uuid4()).replace("-", "")
        self.label = "thing"
        self.parent = ""
        self.type = type
        self.description = ""
        self.pose = Vector6DStable(x=x, y=y, z=z,
                                   rx=rx, ry=ry, rz=rz,
                                   vx=vx, vy=vy, vz=vz,
                                   vrx=vrx, vry=vry, vrz=vrz,
                                   ax=ax, ay=ay, az=az,
                                   arx=arx, ary=ary, arz=arz)

        self.last_update = cv2.getTickCount()
        self.expiration_duration = 1.0

    def is_located(self):
        return self.pose is not None

    def is_occluded(self):
        pass

    def is_perceived(self):
        pass

    def to_delete(self):
        pass

    def has_shape(self):
        return self.shape is not None

    def has_camera(self):
        return self.camera is not None

    def from_msg(self, msg):
        self.id = msg.id
        self.label = msg.label
        self.parent = msg.parent
        self.type = msg.type
        self.description = msg.description
        if msg.is_located is True:
            x = msg.pose_stamped.pose.pose.position.x
            y = msg.pose_stamped.pose.pose.position.y
            z = msg.pose_stamped.pose.pose.position.z
            qx = msg.pose_stamped.pose.pose.orientation.x
            qy = msg.pose_stamped.pose.pose.orientation.y
            qz = msg.pose_stamped.pose.pose.orientation.z
            qw = msg.pose_stamped.pose.pose.orientation.w
            self.pose = Vector6DStable(x=x, y=y, z=z).from_quaternion(qx, qy, qz, qw)
        else:
            self.pose = None

        if msg.has_shape is True:
            self.shape = None
        else:
            self.shape = None

        if msg.has_camera is True:
            self.camera = Camera()
        else:
            self.camera = None

        return self

    def to_msg(self):
        """
        """
        msg = uwds3_msgs.msg.SceneNode()
        return msg
