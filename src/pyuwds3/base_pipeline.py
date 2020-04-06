import cv2
import rospy
import math
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from uwds3_msgs.msg import SceneChangesStamped
from cv_bridge import CvBridge
from .utils.tf_bridge import TfBridge
from pyuwds3.types.vector.vector6d import Vector6D
from .reasoning.simulation.internal_simulator import InternalSimulator
from pyuwds3.types.camera import Camera


class BasePipeline(object):
    """ Base class to implement perception pipelines """
    def __init__(self):
        """ """
        self.tf_bridge = TfBridge()

        self.rgb_image_topic = rospy.get_param("~rgb_image_topic", "/camera/rgb/image_raw")
        self.rgb_camera_info_topic = rospy.get_param("~rgb_camera_info_topic", "/camera/rgb/camera_info")

        self.depth_image_topic = rospy.get_param("~depth_image_topic", "/camera/depth/image_raw")
        self.depth_camera_info_topic = rospy.get_param("~depth_camera_info_topic", "/camera/depth/camera_info")

        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.global_frame_id = rospy.get_param("~global_frame_id", "odom")

        self.bridge = CvBridge()

        self.robot_camera = None
        self.camera_info = None
        self.camera_frame_id = None

        self.n_frame = rospy.get_param("~n_frame", 4)
        self.frame_count = 0

        self.use_depth = rospy.get_param("~use_depth", False)

        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.publish_viz = rospy.get_param("~publish_viz", True)

        self.scene_publisher = rospy.Publisher("tracks", SceneChangesStamped, queue_size=1)

        self.visualization_publisher = rospy.Publisher("tracks_image", Image, queue_size=1)

        self.robot_view_publisher = rospy.Publisher("robot_view", Image, queue_size=1)

        use_simulation_gui = rospy.get_param("~use_simulation_gui", True)
        cad_models_additional_search_path = rospy.get_param("~cad_models_additional_search_path", "")
        static_env_urdf_file_path = rospy.get_param("~static_env_urdf_file_path", "")
        static_entities_config_filename = rospy.get_param("~static_entities_config_filename", "")
        robot_urdf_file_path = rospy.get_param("~robot_urdf_file_path", "")

        self.robot_camera_fov = rospy.get_param("~robot_camera_fov", 60.0)
        self.robot_camera_clipnear = rospy.get_param("~robot_camera_clipfar", 1000.0)
        self.robot_camera_clipfar = rospy.get_param("~robot_camera_clipnear", 0.3)

        self.internal_simulator = InternalSimulator(use_simulation_gui,
                                                    cad_models_additional_search_path,
                                                    static_env_urdf_file_path,
                                                    static_entities_config_filename,
                                                    robot_urdf_file_path,
                                                    self.global_frame_id,
                                                    self.base_frame_id)

        self.initialize_pipeline(self.internal_simulator)

        rospy.loginfo("[perception] Subscribing to '/{}' topic...".format(self.depth_camera_info_topic))
        self.camera_info_subscriber = rospy.Subscriber(self.depth_camera_info_topic, CameraInfo, self.camera_info_callback)

        if self.use_depth is True:
            rospy.loginfo("[perception] Subscribing to '/{}' topic...".format(self.rgb_image_topic))
            self.rgb_image_sub = message_filters.Subscriber(self.rgb_image_topic, Image)
            rospy.loginfo("[perception] Subscribing to '/{}' topic...".format(self.depth_image_topic))
            self.depth_image_sub = message_filters.Subscriber(self.depth_image_topic, Image)

            self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 10, 0.1, allow_headerless=True)
            self.sync.registerCallback(self.observation_callback)
        else:
            rospy.loginfo("[perception] Subscribing to '/{}' topic...".format(self.rgb_image_topic))
            self.rgb_image_sub = rospy.Subscriber(self.rgb_image_topic, Image, self.observation_callback, queue_size=1)

    def camera_info_callback(self, msg):
        """ """
        if self.camera_info is None:
            rospy.loginfo("[perception] Camera info received !")
        self.camera_info = msg
        self.camera_frame_id = msg.header.frame_id
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)
        self.robot_camera = Camera().from_msg(msg,
                                              fov=self.robot_camera_fov,
                                              clipnear=self.robot_camera_clipnear,
                                              clipfar=self.robot_camera_clipfar)

    def initialize_pipeline(self, internal_simulator):
        raise NotImplementedError("You should implement the initialization of the pipeline.")

    def observation_callback(self, bgr_image_msg, depth_image_msg=None):
        """ """
        if self.robot_camera is not None:
            header = bgr_image_msg.header
            header.frame_id = self.global_frame_id
            bgr_image = self.bridge.imgmsg_to_cv2(bgr_image_msg, "bgr8")
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            if depth_image_msg is not None:
                depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg)
            else:
                depth_image = None

            _, self.image_height, self.image_width = bgr_image.shape

            success, view_pose = self.tf_bridge.get_pose_from_tf(self.global_frame_id, self.camera_frame_id)

            if success is not True:
                rospy.logwarn("[perception] The camera sensor is not localized in world space (frame '{}'), please check if the sensor frame is published in /tf".format(self.global_frame_id))
            else:
                view_matrix = view_pose.transform()
                self.frame_count %= self.n_frame

                pipeline_timer = cv2.getTickCount()
                tracks, events = self.perception_pipeline(view_matrix, rgb_image, depth_image=depth_image, time=header.stamp)
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - pipeline_timer)

                myself = self.internal_simulator.get_myself()

                static_nodes = self.internal_simulator.get_static_entities()

                nodes = tracks+[myself]+static_nodes

                self.publish_changes(nodes, events, header)

                if self.publish_tf is True:
                    self.publish_tf_frames(nodes, events, time=header.stamp)

                if self.publish_viz is True:
                    self.publish_visualization(rgb_image, tracks, events, view_pose, fps)

                self.internal_simulator.step_simulation()

                self.frame_count += 1

    def perception_pipeline(self, view_matrix, rgb_image, depth_image=None, time=None):
        raise NotImplementedError("You should implement the perception pipeline.")

    def publish_visualization(self, rgb_image, tracks, events, view_pose, fps):
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        cv2.rectangle(bgr_image, (0, 0), (250, 40), (200, 200, 200), -1)
        perception_fps_str = "Pipeline fps : {:0.1f}hz".format(fps)
        cv2.putText(bgr_image, "Nb tracks : {}".format(len(tracks)), (5, 15),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(bgr_image, perception_fps_str, (5, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        for track in tracks:
            track.draw(bgr_image, (230, 0, 120, 125), 1, view_pose.transform(), self.camera_matrix, self.dist_coeffs)
        self.visualization_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image))

    def publish_tf_frames(self, tracks, events, time=None):
        for track in tracks:
            if track.is_located() is True and track.is_confirmed() is True:
                self.tf_bridge.publish_pose_to_tf(track.pose, self.global_frame_id, track.id, time=time)
        for event in events:
            if event.is_located() is True:
                frame = event.subject+event.description+event.object
                pose = Vector6D(x=event.point.x, y=event.point.y, z=event.point.z)
                self.tf_bridge.publish_pose_to_tf(pose, self.global_frame_id, frame, time=time)

    def publish_changes(self, tracks, events, header):
        """ """
        scene_changes = SceneChangesStamped()
        scene_changes.world = "tracks"
        scene_changes.header.frame_id = self.global_frame_id
        for track in tracks:
            if track.is_confirmed():
                scene_changes.changes.nodes.append(track.to_msg(header))
        self.scene_publisher.publish(scene_changes)
