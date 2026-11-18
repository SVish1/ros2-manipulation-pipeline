import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SceneBroadcaster(Node):
    """
    Publish the scene frames over TF2.

    Static frames (table, platform) plus a simulated object pose, standing in
    for a perception system.
    """

    def __init__(self):
        super().__init__('scene_broadcaster')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        self._publish_static_frames()

        self.timer = self.create_timer(0.5, self._publish_object_pose)
        self.get_logger().info('Scene broadcaster started.')

    def _publish_static_frames(self):
        table_tf = TransformStamped()
        table_tf.header.stamp = self.get_clock().now().to_msg()
        table_tf.header.frame_id = 'world'
        table_tf.child_frame_id = 'table'
        table_tf.transform.translation.x = 0.5
        table_tf.transform.translation.y = 0.0
        table_tf.transform.translation.z = 0.0
        table_tf.transform.rotation.w = 1.0

        # The place target: a second fixed location on a raised platform,
        # offset from the object so the place motion is a real traverse
        # across the workspace, not a trivial in-place drop.
        platform_tf = TransformStamped()
        platform_tf.header.stamp = self.get_clock().now().to_msg()
        platform_tf.header.frame_id = 'table'
        platform_tf.child_frame_id = 'platform'
        platform_tf.transform.translation.x = -0.15
        platform_tf.transform.translation.y = -0.25
        platform_tf.transform.translation.z = 0.05
        platform_tf.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([table_tf, platform_tf])

    def _publish_object_pose(self):
        obj_tf = TransformStamped()
        obj_tf.header.stamp = self.get_clock().now().to_msg()
        obj_tf.header.frame_id = 'table'
        obj_tf.child_frame_id = 'object'
        obj_tf.transform.translation.x = 0.1
        obj_tf.transform.translation.y = 0.05
        obj_tf.transform.translation.z = 0.02
        obj_tf.transform.rotation.w = 1.0
        self.dynamic_broadcaster.sendTransform(obj_tf)


def main():
    rclpy.init()
    node = SceneBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
