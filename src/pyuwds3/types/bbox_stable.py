from .vector.vector2d_stable import Vector2DStable
from .vector.scalar_stable import ScalarStable
from .bbox import BoundingBox


class BoundingBoxStable(BoundingBox):
    def __init__(self, xmin, ymin, xmax, ymax, depth=None, p_cov=10000, m_cov=1):
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
        self.center_filter = Vector2DStable(x=x, y=y, p_cov=p_cov, m_cov=m_cov)
        self.aspect_filter = ScalarStable(x=a, p_cov=p_cov, m_cov=m_cov)
        self.height_filter = ScalarStable(x=h, p_cov=p_cov, m_cov=m_cov)
        if depth is not None:
            self.depth_filter = ScalarStable(x=depth, p_cov=p_cov, m_cov=m_cov)
            self.depth = float(depth)
        else:
            self.depth_filter = None
            self.depth = None

    def update(self, xmin, ymin, xmax, ymax, depth=None):
        w = xmax - xmin
        h = ymax - ymin
        self.center_filter.update(xmin + w/2.0, ymin + h/2.0)
        self.aspect_filter.update(w/float(h))
        self.height_filter.update(h)
        if self.depth_filter is not None:
            if depth is not None:
                self.depth_filter.update(depth)
        h = self.height_filter.x
        w = self.height_filter.x * self.aspect_filter.x
        x = self.center_filter.x
        y = self.center_filter.y
        self.xmin = x - w/2.0
        self.ymin = y - h/2.0
        self.xmax = x + w/2.0
        self.ymax = y + h/2.0
        if self.depth_filter is not None:
            self.depth = self.depth_filter.x
        assert self.width() > 20 and self.height() > 20
        if self.depth is not None:
            assert self.depth < 100

    def predict(self):
        self.center_filter.predict()
        self.aspect_filter.predict()
        self.height_filter.predict()
        if self.depth_filter is not None:
            self.depth_filter.predict()
        h = self.height_filter.x
        w = self.height_filter.x * self.aspect_filter.x
        x = self.center_filter.x
        y = self.center_filter.y
        self.xmin = x - w/2.0
        self.ymin = y - h/2.0
        self.xmax = x + w/2.0
        self.ymax = y + h/2.0
        if self.depth_filter is not None:
            self.depth = self.depth_filter.x

    def update_cov(self, p_cov, m_cov):
        self.center_filter.update_cov(p_cov, m_cov)
        self.aspect_filter.update_cov(p_cov, m_cov)
        self.height_filter.update_cov(p_cov, m_cov)
        if self.depth_filter is not None:
            self.depth_filter.update_cov(p_cov, m_cov)
