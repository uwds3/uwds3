import numpy as np
import cv2
from .vector.vector2d import Vector2D
from .features import Features
from .cylinder import Cylinder


class BoundingBox(object):
    """Represents a 2D+depth aligned bounding box in the image space"""

    def __init__(self, xmin, ymin, xmax, ymax, depth=None, mode="xyxy"):
        """BoundingBox constructor"""
        self.__check_mode(mode)
        if mode == "xyxy":
            self.xmin = int(xmin)
            self.ymin = int(ymin)
            self.xmax = int(xmax)
            self.ymax = int(ymax)
        else:
            self.xmin = int(xmin - xmax/2.0)
            self.ymin = int(ymin - ymax/2.0)
            self.xmax = int(xmin + xmax/2.0)
            self.ymax = int(ymax + ymax/2.0)
        self.depth = depth

    def center(self):
        """Returns the bbox's center in pixels"""
        return Vector2D(self.xmin + self.width()/2, self.ymin + self.height()/2)

    def width(self):
        """Returns the bbox's width in pixels"""
        return self.xmax - self.xmin

    def height(self):
        """Returns the bbox's height in pixels"""
        return self.ymax - self.ymin

    def diagonal(self):
        """Returns the bbox's diagonal in pixels"""
        return sqrt(pow(self.width(), 2)+pow(self.height(), 2))

    def radius(self):
        """Returns the bbox's radius in pixels"""
        return self.diagonal()/2.0

    def area(self):
        """Returns the bbox's area in pixels"""
        return (self.width()+1)*(self.height()+1)

    def cylinder(self, camera_matrix, dist_coeffs):
        assert self.depth is not None
        z = self.depth
        fx = camera_matrix[0][0]
        fy = camera_matrix[1][1]
        cx = camera_matrix[0][2]
        cy = camera_matrix[1][2]
        w = self.width()
        h = self.height()
        x = (self.center().x - cx) * z / fx
        y = (self.center().y - cy) * z / fy
        d = w * z / fx
        h = h * z / fy
        return Cylinder(d, h, x=x, y=y, z=z)

    def draw(self, frame, color, thickness):
        """Draws the bbox"""
        cv2.rectangle(frame, (self.xmin, self.ymin), (self.xmax, self.ymax), color, thickness)

    def from_array(self, array, mode="xyxy"):
        """ """
        self.__check_mode(mode)
        assert array.shape == (5, 1) or array.shape == (6, 1)
        if mode == "xyxy":
            self.xmin = array[0]
            self.ymin = array[1]
            self.xmax = array[2]
            self.ymax = array[3]
        else:
            x = array[0]
            y = array[1]
            w = array[2]
            h = array[3]
            self.xmin = x - w/2.0
            self.ymin = y - h/2.0
            self.xmax = x + w/2.0
            self.ymax = y + h/2.0
        if array.shape == (6, 1):
            self.depth = array[4]

    def to_array(self):
        if self.depth is not None:
            return np.array([self.xmin, self.ymin, self.xmax, self.ymax, self.depth], np.float32)
        else:
            return np.array([self.xmin, self.ymin, self.xmax, self.ymax], np.float32)

    def to_xyxy(self):
        raise NotImplementedError()

    def to_xywh(self):
        c = self.center()
        if depth is None:
            return np.array([c.x, c.y, self.width(), self.height()], np.float32)
        else:
            return np.array([c.x, c.y, self.width(), self.height(), self.depth], np.float32)

    def features(self, image_width, image_height, max_depth=25):
        """Returns the bbox geometric features"""
        features = [self.xmin/float(image_width),
                    self.ymin/float(image_height),
                    self.xmax/float(image_width),
                    self.ymax/float(image_height),
                    min(self.depth/float(max_depth), float(1.0))]
        return Features("geometric", np.array(features).flatten(), 0.89)

    def __check_mode(self, mode):
        if mode not in ["xyxy", "xywh"]:
            raise ValueError("BoundingBox mode should be 'xyxy' (xmin, xmax, ymin, ymax) or 'xywh' (cx, cy, w, h)")
