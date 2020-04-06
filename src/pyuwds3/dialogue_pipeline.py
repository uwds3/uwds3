import rospy
from .base_pipeline import BasePipeline
from .reasoning.detection.ssd_detector import SSDDetector
from .reasoning.tracking.multi_object_tracker import MultiObjectTracker, iou_cost, color_cost, centroid_cost
from .reasoning.estimation.head_pose_estimator import HeadPoseEstimator
from .reasoning.estimation.object_pose_estimator import ObjectPoseEstimator
from .reasoning.estimation.shape_estimator import ShapeEstimator
from .reasoning.estimation.facial_landmarks_estimator import FacialLandmarksEstimator
from .reasoning.estimation.facial_features_estimator import FacialFeaturesEstimator
from .reasoning.estimation.color_features_estimator import ColorFeaturesEstimator
from .reasoning.monitoring.perspective_monitor import PerspectiveMonitor


class DialoguePipeline(BasePipeline):
    def __init__(self):
        super(DialoguePipeline, self).__init__()

    def initialize_pipeline(self, internal_simulator):
        """ """
        ######################################################
        # Detection
        ######################################################
        detector_model_filename = rospy.get_param("~detector_model_filename", "")
        detector_weights_filename = rospy.get_param("~detector_weights_filename", "")
        detector_config_filename = rospy.get_param("~detector_config_filename", "")

        face_detector_model_filename = rospy.get_param("~face_detector_model_filename", "")
        face_detector_weights_filename = rospy.get_param("~face_detector_weights_filename", "")
        face_detector_config_filename = rospy.get_param("~face_detector_config_filename", "")

        self.person_detector = SSDDetector(detector_model_filename,
                                           detector_weights_filename,
                                           detector_config_filename)

        self.face_detector = SSDDetector(face_detector_model_filename,
                                         face_detector_weights_filename,
                                         face_detector_config_filename)

        ####################################################################
        # Features estimation
        ####################################################################

        facial_features_model_filename = rospy.get_param("~facial_features_model_filename", "")

        face_3d_model_filename = rospy.get_param("~face_3d_model_filename", "")
        self.facial_features_estimator = FacialFeaturesEstimator(face_3d_model_filename, facial_features_model_filename)

        self.color_features_estimator = ColorFeaturesEstimator()

        ######################################################
        # Tracking
        ######################################################

        self.n_init = rospy.get_param("~n_init", 1)
        self.max_iou_distance = rospy.get_param("~max_iou_distance", 0.8)
        self.max_color_distance = rospy.get_param("~max_color_distance", 0.8)
        self.max_face_distance = rospy.get_param("~max_face_distance", 0.8)
        self.max_centroid_distance = rospy.get_param("~max_centroid_distance", 0.8)
        self.max_lost = rospy.get_param("~max_lost", 4)
        self.max_age = rospy.get_param("~max_age", 20)

        self.face_tracker = MultiObjectTracker(iou_cost,
                                               centroid_cost,
                                               self.max_iou_distance,
                                               None,
                                               self.n_init,
                                               self.max_lost,
                                               self.max_age)

        self.person_tracker = MultiObjectTracker(iou_cost,
                                                 color_cost,
                                                 self.max_iou_distance,
                                                 self.max_color_distance,
                                                 self.n_init,
                                                 self.max_lost,
                                                 self.max_age)

        ########################################################
        # Pose & Shape estimation
        ########################################################

        shape_predictor_config_filename = rospy.get_param("~shape_predictor_config_filename", "")
        self.facial_landmarks_estimator = FacialLandmarksEstimator(shape_predictor_config_filename)
        face_3d_model_filename = rospy.get_param("~face_3d_model_filename", "")
        self.head_pose_estimator = HeadPoseEstimator(face_3d_model_filename)

        self.shape_estimator = ShapeEstimator()
        self.object_pose_estimator = ObjectPoseEstimator()

        self.perspective_monitor = PerspectiveMonitor(internal_simulator)

    def perception_pipeline(self, view_matrix, rgb_image, depth_image=None, time=None):
        """ """
        ######################################################
        # Detection
        ######################################################

        detections = []
        if self.frame_count == 0:
            detections = self.person_detector.detect(rgb_image, depth_image=depth_image)
        elif self.frame_count == 1:
            detections = self.face_detector.detect(rgb_image, depth_image=depth_image)
        else:
            detections = []

        ####################################################################
        # Features estimation
        ####################################################################

        self.color_features_estimator.estimate(rgb_image, detections)

        ######################################################
        # Tracking
        ######################################################

        if self.frame_count == 0:
            person_detections = [d for d in detections if d.label == "person"]
            face_tracks = self.face_tracker.update(rgb_image, [])
            person_tracks = self.person_tracker.update(rgb_image, person_detections)
        elif self.frame_count == 1:
            face_tracks = self.face_tracker.update(rgb_image, detections)
            person_tracks = self.person_tracker.update(rgb_image, [])
        else:
            face_tracks = self.face_tracker.update(rgb_image, [])
            person_tracks = self.person_tracker.update(rgb_image, [])

        tracks = face_tracks + person_tracks

        ########################################################
        # Pose & Shape estimation
        ########################################################

        self.facial_landmarks_estimator.estimate(rgb_image, face_tracks)

        self.head_pose_estimator.estimate(face_tracks, view_matrix, self.camera_matrix, self.dist_coeffs)
        self.object_pose_estimator.estimate(person_tracks, view_matrix, self.camera_matrix, self.dist_coeffs)

        self.shape_estimator.estimate(rgb_image, tracks, self.camera_matrix, self.dist_coeffs)

        ########################################################
        # Recognition
        ########################################################

        self.facial_features_estimator.estimate(rgb_image, face_tracks)

        ########################################################
        # Monitoring
        ########################################################

        events = self.perspective_monitor.monitor(face_tracks)

        return tracks, events
