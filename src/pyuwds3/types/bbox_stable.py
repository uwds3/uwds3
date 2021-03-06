import numpy as np
from .vector.vector2d_stable import Vector2DStable
from .vector.scalar_stable import ScalarStable
from .bbox import BoundingBox


class BoundingBoxStable(BoundingBox):
    """ """
    def __init__(self, xmin, ymin, xmax, ymax, depth=None, p_cov=10, m_cov=10):
        self.xmin = int(xmin)
        self.ymin = int(ymin)
        self.xmax = int(xmax)
        self.ymax = int(ymax)
        w = xmax - xmin
        h = ymax - ymin
        center = self.center()
        x = center.x
        y = center.y
        a = w/float(h)
        self.m_cov = m_cov
        self.p_cov = p_cov
        self.center_filter = Vector2DStable(x=x, y=y, p_cov=p_cov, m_cov=m_cov)
        self.aspect_filter = ScalarStable(x=a, p_cov=p_cov, m_cov=m_cov)
        self.height_filter = ScalarStable(x=h, p_cov=p_cov, m_cov=m_cov)
        if depth is not None:
            self.depth = float(depth)
        else:
            self.depth = None

    def update(self, xmin, ymin, xmax, ymax, depth=None):
        w = xmax - xmin
        h = ymax - ymin
        self.center_filter.update(xmin + w/2.0, ymin + h/2.0)
        self.aspect_filter.update(w/float(h))
        self.height_filter.update(h)
        if depth is not None:
            self.depth = depth
        h = self.height_filter.x
        w = self.height_filter.x * self.aspect_filter.x
        x = self.center_filter.x
        y = self.center_filter.y
        self.xmin = x - w/2.0
        self.ymin = y - h/2.0
        self.xmax = x + w/2.0
        self.ymax = y + h/2.0

    def predict(self):
        self.center_filter.predict()
        self.aspect_filter.predict()
        self.height_filter.predict()
        h = self.height_filter.x
        w = self.height_filter.x * self.aspect_filter.x
        x = self.center_filter.x
        y = self.center_filter.y
        self.xmin = x - w/2.0
        self.ymin = y - h/2.0
        self.xmax = x + w/2.0
        self.ymax = y + h/2.0

    def update_cov(self, p_cov, m_cov):
        self.center_filter.update_cov(p_cov, m_cov)
        self.aspect_filter.update_cov(p_cov, m_cov)
        self.height_filter.update_cov(p_cov, m_cov)
        if self.depth_filter is not None:
            self.depth_filter.update_cov(p_cov, m_cov)
