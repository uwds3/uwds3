import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from pyuwds3.types.vector.vector6d import Vector6D


class TfBridge(object):
    """ Utility class to interface with /tf2 """
    def __init__(self):
        """ """
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()

    def publish_pose_to_tf(self, pose, source_frame, target_frame, time=None):
        """ Publish the given pose to /tf2 """
        msg = pose.to_msg()
        transform = TransformStamped()
        transform.child_frame_id = target_frame
        transform.header.frame_id = source_frame
        if time is not None:
            transform.header.stamp = time
        else:
            transform.header.stamp = rospy.Time().now()
        transform.transform.translation = msg.position
        transform.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(transform)

    def get_pose_from_tf(self, source_frame, target_frame, time=None):
        """ Get the pose from /tf2 """
        try:
            if time is not None:
                trans = self.tf_buffer.lookup_transform(source_frame, target_frame, time)
            else:
                trans = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            rx = trans.transform.rotation.x
            ry = trans.transform.rotation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w
            pose = Vector6D(x=x, y=y, z=z).from_quaternion(rx, ry, rz, rw)
            return True, pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[perception] Exception occured: {}".format(e))
            return False, None
