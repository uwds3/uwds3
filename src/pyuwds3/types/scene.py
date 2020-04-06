import rospy
import cv2


class Scene(object):
    def __init__(self):
        self.nodes = []
        self.nodes_index = {}
        self.last_update = rospy.Time().now()

    def update(self, node):
        if node.id not in self.nodes_index:
            self.nodes.append(node)
            self.nodes_index[node.id] = len(self.nodes)-1
        self.nodes[self.nodes_index[node.id]]
