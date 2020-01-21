from math import sqrt


def iou(bbox_a, bbox_b):
    """Returns the intersection over union metric"""
    xmin = int(min(bbox_a.xmin, bbox_b.xmin))
    ymin = int(min(bbox_a.ymin, bbox_b.ymin))
    xmax = int(max(bbox_a.xmax, bbox_b.xmax))
    ymax = int(max(bbox_a.ymax, bbox_b.ymax))
    intersection_area = ((xmax-xmin)*(ymax-ymin))+1
    union_area = bbox_a.area() + bbox_b.area()
    if float(union_area - intersection_area) == 0.0:
        return 0.0
    else:
        return intersection_area / float(union_area - intersection_area)


def centroid(bbox_a, bbox_b):
    """Returns the euler distance between centroids"""
    xa = bbox_a.center().x
    xb = bbox_b.center().x
    ya = bbox_a.center().y
    yb = bbox_b.center().y
    return sqrt(pow(xa-xb, 2)+pow(ya-yb, 2))


def overlap(bbox_a, bbox_b):
    """Returns the overlap ratio"""
    xmin = int(min(bbox_a.xmin, bbox_b.xmin))
    ymin = int(min(bbox_a.ymin, bbox_b.ymin))
    xmax = int(max(bbox_a.xmax, bbox_b.xmax))
    ymax = int(max(bbox_a.ymax, bbox_b.ymax))
    intersection_area = ((xmax-xmin)*(ymax-ymin))+1
    return intersection_area / bbox_a.area()
