import rospy
import uuid
import cv2
import numpy as np
import uwds3_msgs
import uwds3_msgs.msg
from tf.transformations import euler_matrix
from .camera import Camera
from .shape.cylinder import Cylinder
from .shape.sphere import Sphere
from .vector.vector6d import Vector6D
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

        self.features = {}

        self.shapes = []

        self.last_update = rospy.Time().now()
        self.expiration_duration = 1.0

    def is_occluded(self):
        pass

    def is_perceived(self):
        pass

    def to_delete(self):
        pass

    def is_located(self):
        return self.pose is not None

    def has_shape(self):
        return len(self.shapes) > 0

    def has_camera(self):
        return self.camera is not None

    def draw(self, image, color, thickness, view_matrix, camera_matrix, dist_coeffs):
        """Draws the track"""
        if self.is_confirmed():
            track_color = (0, 200, 0, 0)
            text_color = (50, 50, 50)
        else:
            if self.is_occluded():
                track_color = (0, 0, 200, 0.3)
                text_color = (250, 250, 250)
            else:
                track_color = (200, 0, 0, 0.3)
                text_color = (250, 250, 250)

        if self.is_confirmed():
            if self.is_located() and self.is_confirmed():
                sensor_pose = Vector6D().from_transform(np.dot(np.linalg.inv(view_matrix), self.pose.transform()))
                rot = sensor_pose.rotation().to_array()
                # for opencv convention
                R = euler_matrix(rot[0][0], rot[1][0], rot[2][0], "rxyz")
                rvec = cv2.Rodrigues(R[:3, :3])[0]
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs,
                                  rvec,
                                  sensor_pose.position().to_array(), 0.1)
            cv2.rectangle(image, (self.bbox.xmin, self.bbox.ymax-20),
                                 (self.bbox.xmax, self.bbox.ymax),
                                 (200, 200, 200), -1)
            self.bbox.draw(image, track_color, 2)
            self.bbox.draw(image, text_color, 1)

            cv2.putText(image,
                        "{}".format(self.id[:6]),
                        (self.bbox.xmax-60, self.bbox.ymax-8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        text_color,
                        1)
            cv2.putText(image,
                        self.label,
                        (self.bbox.xmin+5, self.bbox.ymax-8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, text_color, 1)
            if "facial_landmarks" in self.features:
                self.features["facial_landmarks"].draw(image,
                                                       track_color,
                                                       thickness)
        else:
            self.bbox.draw(image, track_color, 1)

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
            vx = msg.twist_stamped.twist.twist.linear.x
            vy = msg.twist_stamped.twist.twist.linear.y
            vz = msg.twist_stamped.twist.twist.linear.z
            vrx = msg.twist_stamped.twist.twist.angular.x
            vry = msg.twist_stamped.twist.twist.angular.y
            vrz = msg.twist_stamped.twist.twist.angular.z
            ax = msg.accel_stamped.accel.accel.linear.x
            ay = msg.accel_stamped.accel.accel.linear.y
            az = msg.accel_stamped.accel.accel.linear.z
            arx = msg.accel_stamped.accel.accel.angular.x
            ary = msg.accel_stamped.accel.accel.angular.y
            arz = msg.accel_stamped.accel.accel.angular.z
            self.pose = Vector6DStable(x=x, y=y, z=z,
                                       vx=vx, vy=vy, vz=vz,
                                       vrx=vrx, vry=vry, vrz=vrz,
                                       ax=ax, ay=ay, az=az,
                                       arx=arx, ary=ary, arz=arz).from_quaternion(qx, qy, qz, qw)
        else:
            self.pose = None

        msg.features = []
        for features in msg.features:
            msg.features.append(features.from_msg(features))

        if msg.has_shape is True:
            for shape in msg.shapes:
                if shape.type == uwds3_msgs.msg.PrimitiveShape.CYLINDER:
                    self.shapes.append(Cylinder().from_msg(shape))
                if shape.type == uwds3_msgs.msg.PrimitiveShape.SPHERE:
                    self.shapes.append(Sphere().from_msg(shape))
                # if shape.type == uwds3_msgs.msg.PrimitiveShape.MESH:
                #     self.shapes.append(Mesh().from_msg(shape))

        if msg.has_camera is True:
            self.camera = Camera()
        else:
            self.camera = None

        return self

    def to_msg(self, header):
        """
        """
        msg = uwds3_msgs.msg.SceneNode()
        msg.id = self.id
        msg.label = self.label
        if self.is_located():
            msg.is_located = True
            msg.pose_stamped.header = header
            position = self.pose.position()
            msg.pose_stamped.pose.pose.position.x = position.x
            msg.pose_stamped.pose.pose.position.y = position.y
            msg.pose_stamped.pose.pose.position.z = position.z
            q = self.pose.quaternion()
            msg.pose_stamped.pose.pose.orientation.x = q[0]
            msg.pose_stamped.pose.pose.orientation.y = q[1]
            msg.pose_stamped.pose.pose.orientation.z = q[2]
            msg.pose_stamped.pose.pose.orientation.w = q[3]
            msg.twist_stamped.header = header
            linear_velocity = self.pose.linear_velocity()
            msg.twist_stamped.twist.twist.linear.x = linear_velocity.x
            msg.twist_stamped.twist.twist.linear.y = linear_velocity.y
            msg.twist_stamped.twist.twist.linear.z = linear_velocity.z
            angular_velocity = self.pose.angular_velocity()
            msg.twist_stamped.twist.twist.angular.x = angular_velocity.x
            msg.twist_stamped.twist.twist.angular.y = angular_velocity.y
            msg.twist_stamped.twist.twist.angular.z = angular_velocity.z
            msg.accel_stamped.header = header
            linear_acceleration = self.pose.linear_acceleration()
            msg.accel_stamped.accel.accel.linear.x = linear_acceleration.x
            msg.accel_stamped.accel.accel.linear.y = linear_acceleration.y
            msg.accel_stamped.accel.accel.linear.z = linear_acceleration.z
            angular_acceleration = self.pose.angular_acceleration()
            msg.accel_stamped.accel.accel.angular.x = angular_acceleration.x
            msg.accel_stamped.accel.accel.angular.y = angular_acceleration.y
            msg.accel_stamped.accel.accel.angular.z = angular_acceleration.z

        for features in self.features.values():
            msg.features.append(features.to_msg())

        if self.has_camera():
            msg.has_camera = True
            msg.camera.info.header = header
            #msg.camera.info.header.frame_id = msg.id
            msg.camera = self.camera.to_msg()

        if self.has_shape():
            msg.has_shape = True
            for shape in self.shapes:
                msg.shapes.append(shape.to_msg())

        msg.last_update = header.stamp
        msg.expiration_duration = rospy.Duration(self.expiration_duration)
        return msg
