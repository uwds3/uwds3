import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyuwds3.types.vector.vector6d import Vector6D


class PerspectiveMonitor(object):
    def __init__(self, simulator):
        self.simulator = simulator
        self.cv_bridge = CvBridge()
        self.view_publisher = rospy.Publisher("person_view", Image, queue_size=1)

    def monitor(self, face_tracks):
        camera_tracks_id = []
        object_tracks_id = []

        camera = None
        min_depth = 1000.0
        for t in face_tracks:
            if t.is_confirmed():
                if t.has_camera() is True and t.is_located() is True:
                    if t.bbox.depth < min_depth:
                        min_depth = t.bbox.depth
                        camera = t

        if camera is not None:
            view_image, _, _ = self.simulator.get_camera_view(camera.pose, camera.camera)
            view_image = cv2.cvtColor(view_image, cv2.COLOR_RGB2BGR)
            self.view_publisher.publish(self.cv_bridge.cv2_to_imgmsg(view_image))
        return []
