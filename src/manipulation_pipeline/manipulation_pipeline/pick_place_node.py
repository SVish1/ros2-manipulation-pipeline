import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose


DOWN_ORIENTATION = (1.0, 0.0, 0.0, 0.0)  # gripper pointing straight down


def make_pose(frame_id, x, y, z, orientation=DOWN_ORIENTATION):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    ox, oy, oz, ow = orientation
    pose.pose.orientation.x = ox
    pose.pose.orientation.y = oy
    pose.pose.orientation.z = oz
    pose.pose.orientation.w = ow
    return pose


class PickPlaceNode(Node):
    """State-machine pick-and-place: picks an object detected via TF2 and
    places it on a second fixed platform pose, with monitored execution,
    timeouts, and single-retry recovery on every motion."""

    PLANNING_FRAME = 'panda_link0'
    EE_LINK = 'panda_link8'
    PLANNING_GROUP = 'panda_arm'
    APPROACH_HEIGHT = 0.10
    GRASP_HEIGHT = 0.02
    HOME_POSE_XYZ = (0.4, 0.0, 0.5)
    MOVE_TIMEOUT_SEC = 8.0
    GRIPPER_TIMEOUT_SEC = 4.0
    MAX_ATTEMPTS = 2

    def __init__(self):
        super().__init__('pick_place_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd')

        self.get_logger().info('Waiting for /move_action and gripper servers...')
        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers available.')


    # ---------- top-level sequencing ----------

    def run_pick_and_place(self):
        object_pose = self._lookup_frame_pose('object')
        platform_pose = self._lookup_frame_pose('platform')
        if object_pose is None or platform_pose is None:
            self.get_logger().error('Aborting: required TF frame(s) unavailable.')
            return False

        ox, oy, oz = object_pose
        px, py, pz = platform_pose

        steps = [
            ('approach-pick', make_pose(self.PLANNING_FRAME, ox, oy, oz + self.APPROACH_HEIGHT)),
            ('descend-pick', make_pose(self.PLANNING_FRAME, ox, oy, oz + self.GRASP_HEIGHT)),
            ('__grasp__', None),
            ('retreat-pick', make_pose(self.PLANNING_FRAME, ox, oy, oz + self.APPROACH_HEIGHT)),
            ('approach-place', make_pose(self.PLANNING_FRAME, px, py, pz + self.APPROACH_HEIGHT)),
            ('descend-place', make_pose(self.PLANNING_FRAME, px, py, pz + self.GRASP_HEIGHT)),
            ('__release__', None),
            ('retreat-place', make_pose(self.PLANNING_FRAME, px, py, pz + self.APPROACH_HEIGHT)),
            ('return-home', make_pose(self.PLANNING_FRAME, *self.HOME_POSE_XYZ)),
        ]

        for name, pose in steps:
            if name == '__grasp__':
                ok = self._monitored_gripper(close=True, description='grasp')
            elif name == '__release__':
                ok = self._monitored_gripper(close=False, description='release')
            else:
                ok = self._move_with_recovery(pose, name)

            if not ok:
                self.get_logger().error(
                    f'Sequence ABORTED at step "{name}" after exhausting recovery.')
                return False

        self.get_logger().info('Pick-and-place sequence completed successfully.')
        return True

    # ---------- TF2 ----------

    def _lookup_frame_pose(self, frame_name):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, frame_name, Time())
        except TransformException as ex:
            self.get_logger().error(f'TF2 lookup for "{frame_name}" failed: {ex}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    # ---------- monitored motion with recovery ----------

    def _move_with_recovery(self, pose, description):
        for attempt in range(1, self.MAX_ATTEMPTS + 1):
            self.get_logger().info(f'[{description}] attempt {attempt}/{self.MAX_ATTEMPTS}')
            if self._move_to_pose(pose, description):
                return True
            if attempt < self.MAX_ATTEMPTS:
                self.get_logger().warn(
                    f'[{description}] failed — recovering to home pose before retry.')
                self._recover_to_home()
        self.get_logger().error(
            f'[{description}] failed after {self.MAX_ATTEMPTS} attempts.')
        return False

    def _recover_to_home(self):
        home_pose = make_pose(self.PLANNING_FRAME, *self.HOME_POSE_XYZ)
        self._move_to_pose(home_pose, 'recovery-to-home')

    def _move_to_pose(self, pose, description):
        goal = self._build_move_goal(pose)
        ok, result = self._send_and_wait(
            self.move_client, goal, self.MOVE_TIMEOUT_SEC, description)
        if not ok:
            return False
        if result.error_code.val != 1:
            self.get_logger().error(
                f'[{description}] MoveIt error code {result.error_code.val} (!= SUCCESS).')
            return False
        self.get_logger().info(f'[{description}] succeeded.')
        return True

    def _monitored_gripper(self, close, description):
        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if close else 0.04
        goal.command.max_effort = 20.0
        ok, _ = self._send_and_wait(
            self.gripper_client, goal, self.GRIPPER_TIMEOUT_SEC, description)
        if ok:
            self.get_logger().info(f'[{description}] succeeded.')
        return ok

    def _send_and_wait(self, client, goal, timeout_sec, description):
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)

        if not send_future.done():
            self.get_logger().error(f'[{description}] timed out waiting for goal acceptance.')
            return False, None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'[{description}] goal rejected.')
            return False, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error(f'[{description}] timed out waiting for result — cancelling.')
            goal_handle.cancel_goal_async()
            return False, None

        return True, result_future.result().result

    def _build_move_goal(self, pose: PoseStamped):
        request = MotionPlanRequest()
        request.group_name = self.PLANNING_GROUP
        request.goal_constraints.append(self._make_goal_constraints(pose))
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        options = PlanningOptions()
        options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    def _make_goal_constraints(self, pose: PoseStamped, tolerance=0.02):
        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = self.EE_LINK
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tolerance]
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose.header
        orient_constraint.link_name = self.EE_LINK
        orient_constraint.orientation = pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.15
        orient_constraint.absolute_y_axis_tolerance = 0.15
        orient_constraint.absolute_z_axis_tolerance = 0.15
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        return constraints


def main():
    rclpy.init()
    node = PickPlaceNode()

    # Let TF2's buffer populate (static + object frames) before we start
    # looking up poses from it. No outer rclpy.spin(node) here on purpose:
    # nesting rclpy.spin_until_future_complete() inside a timer callback
    # driven by an outer spin() double-spins the same node on two
    # executors at once, which silently starves action responses (this is
    # exactly what caused the earlier timeout bug).
    settle_end = node.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
    while rclpy.ok() and node.get_clock().now() < settle_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    success = node.run_pick_and_place()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())

