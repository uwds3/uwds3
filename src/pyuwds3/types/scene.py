import cv2


class Scene(object):
    def __init__(self):
        self.nodes = []
        self.last_update = cv2.getTickCount()

    def update(self, node):
        self.nodes.append(node)
