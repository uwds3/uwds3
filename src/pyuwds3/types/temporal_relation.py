import cv2


class TemporalRelation(object):
    def __init__(self, subject, description, object, confidence=1.0, expiration=2.0):
        self.description = description
        self.subject = subject
        self.object = object
        self.confidence = confidence
        self.start = cv2.getTickCount()
        self.last_update = cv2.getTickCount()
        self.expiration_duration = expiration
        self.default_value = 0.0

    def to_msg(self):
        raise NotImplementedError()
