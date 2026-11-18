import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    PlanningScene,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    CollisionObject,
    AttachedCollisionObject,
)
from moveit_msgs.srv import ApplyPlanningScene
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose


DOWN_ORIENTATION = (1.0, 0.0, 0.0, 0.0)  # flange pointing straight down

# Flange pointing down, rotated +45 deg about the vertical axis. The Panda hand
# is mounted at -45 deg on panda_link8, so this lines the fingers up with the
# object's x/y faces instead of closing across its diagonal.
GRASP_ORIENTATION = (0.9238795325112867, -0.3826834323650898, 0.0, 0.0)


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
    """
    State-machine pick-and-place for the Panda arm.

    Picks an object detected via TF2 and places it on a second fixed platform
    pose, with monitored execution, timeouts, and single-retry recovery on
    every motion. The object is tracked in the MoveIt planning scene and
    attached to the hand while carried, so planning accounts for it.
    """

    PLANNING_FRAME = 'panda_link0'
    EE_LINK = 'panda_link8'
    HAND_LINK = 'panda_hand'
    TOUCH_LINKS = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger']
    PLANNING_GROUP = 'panda_arm'

    # Motion goals are expressed for EE_LINK (the flange), but the grasp point
    # between the fingertips sits this far below it along the gripper axis
    # (hand-to-finger offset 0.0584 m + finger length). Every target height is
    # raised by this amount so the *fingertips* reach the intended height.
    TCP_OFFSET = 0.1034

    APPROACH_HEIGHT = 0.10   # fingertips this far above the grasp/place point
    GRASP_HEIGHT = 0.0       # fingertips level with the object's centre
    PLACE_CLEARANCE = 0.005  # release the object just above the platform

    OBJECT_ID = 'object'
    OBJECT_SIZE = 0.04       # cube edge length (m)
    GRIPPER_OPEN = 0.04      # per-finger opening (m)
    GRIPPER_CLOSED = OBJECT_SIZE / 2.0

    # Radius of the sphere the flange must end up in. Kept tight: the hand clears
    # the top of the object by under 2 cm at the grasp pose.
    POSITION_TOLERANCE = 0.005

    LINEAR_PIPELINE = 'pilz_industrial_motion_planner'
    LINEAR_PLANNER = 'LIN'
    FREE_SPACE_SCALING = 0.3  # velocity/acceleration scaling for OMPL moves
    # Straight-line moves need more joint acceleration than the joint-space
    # path between the same poses; at 0.3 Pilz rejects some of them for
    # exceeding the Panda's joint acceleration limits.
    LINEAR_SCALING = 0.1

    HOME_POSE_XYZ = (0.4, 0.0, 0.5)
    MOVE_TIMEOUT_SEC = 8.0
    GRIPPER_TIMEOUT_SEC = 4.0
    SCENE_TIMEOUT_SEC = 3.0
    MAX_ATTEMPTS = 2

    def __init__(self):
        super().__init__('pick_place_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')

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

        ox, oy, oz = object_pose   # object centre
        px, py, pz = platform_pose  # platform top surface
        place_z = pz + self.OBJECT_SIZE / 2.0 + self.PLACE_CLEARANCE

        if not self._add_object_to_scene(ox, oy, oz):
            self.get_logger().error('Aborting: could not add object to planning scene.')
            return False

        steps = [
            ('open-gripper', lambda: self._monitored_gripper(False, 'open-gripper')),
            ('approach-pick', self._motion(ox, oy, oz + self.APPROACH_HEIGHT, 'approach-pick')),
            ('descend-pick',
             self._motion(ox, oy, oz + self.GRASP_HEIGHT, 'descend-pick', linear=True)),
            ('grasp', lambda: self._monitored_gripper(True, 'grasp')),
            ('attach-object', self._attach_object),
            ('retreat-pick',
             self._motion(ox, oy, oz + self.APPROACH_HEIGHT, 'retreat-pick', linear=True)),
            ('approach-place',
             self._motion(px, py, place_z + self.APPROACH_HEIGHT, 'approach-place')),
            ('descend-place', self._motion(px, py, place_z, 'descend-place', linear=True)),
            ('release', lambda: self._monitored_gripper(False, 'release')),
            ('detach-object', self._detach_object),
            ('retreat-place',
             self._motion(px, py, place_z + self.APPROACH_HEIGHT, 'retreat-place',
                          linear=True)),
            ('return-home', lambda: self._move_with_recovery(
                make_pose(self.PLANNING_FRAME, *self.HOME_POSE_XYZ), 'return-home')),
        ]

        for name, action in steps:
            if not action():
                self.get_logger().error(
                    f'Sequence ABORTED at step "{name}" after exhausting recovery.')
                return False

        self.get_logger().info('Pick-and-place sequence completed successfully.')
        return True

    def _motion(self, x, y, z, description, linear=False):
        """
        Return a step that moves the fingertips to (x, y, z) with recovery.

        linear=True plans a straight-line Cartesian move (used for the short
        vertical descend/retreat moves next to the object); otherwise the
        free-space sampling planner is used.
        """
        pose = self._tcp_pose(x, y, z)
        return lambda: self._move_with_recovery(pose, description, linear)

    def _tcp_pose(self, x, y, z):
        """Flange goal pose that puts the fingertip grasp point at (x, y, z)."""
        return make_pose(
            self.PLANNING_FRAME, x, y, z + self.TCP_OFFSET, GRASP_ORIENTATION)

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

    # ---------- planning scene ----------

    def _add_object_to_scene(self, x, y, z):
        obj = CollisionObject()
        obj.header.frame_id = self.PLANNING_FRAME
        obj.id = self.OBJECT_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.OBJECT_SIZE] * 3
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        return self._apply_scene(scene, 'add-object')

    def _attach_object(self):
        """
        Attach the object to the hand.

        The object moves from the world onto the robot, so it is carried and
        collision-checked as part of the arm.
        """
        return self._apply_scene(
            self._attached_object_scene(CollisionObject.ADD), 'attach-object')

    def _detach_object(self):
        """Detach the object; MoveIt puts it back into the world where it is."""
        return self._apply_scene(
            self._attached_object_scene(CollisionObject.REMOVE), 'detach-object')

    def _attached_object_scene(self, operation):
        attached = AttachedCollisionObject()
        attached.link_name = self.HAND_LINK
        attached.object.id = self.OBJECT_ID
        attached.object.operation = operation
        attached.touch_links = list(self.TOUCH_LINKS)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        return scene

    def _apply_scene(self, scene, description):
        if not self.scene_client.wait_for_service(timeout_sec=self.SCENE_TIMEOUT_SEC):
            self.get_logger().error(f'[{description}] /apply_planning_scene unavailable.')
            return False

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.SCENE_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error(f'[{description}] timed out applying planning scene.')
            return False
        if not future.result().success:
            self.get_logger().error(f'[{description}] planning scene update rejected.')
            return False
        self.get_logger().info(f'[{description}] succeeded.')
        return True

    # ---------- monitored motion with recovery ----------

    def _move_with_recovery(self, pose, description, linear=False):
        for attempt in range(1, self.MAX_ATTEMPTS + 1):
            self.get_logger().info(f'[{description}] attempt {attempt}/{self.MAX_ATTEMPTS}')
            if self._move_to_pose(pose, description, linear):
                return True
            if linear:
                # Straight-line plans can be rejected when the arm's current
                # configuration would need a joint acceleration spike to follow
                # the line. The sampling planner still produces a collision-
                # checked path to the same pose, so try it before giving up.
                self.get_logger().warn(
                    f'[{description}] straight-line plan failed — '
                    'falling back to free-space planner.')
                if self._move_to_pose(pose, description, linear=False):
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

    def _move_to_pose(self, pose, description, linear=False):
        goal = self._build_move_goal(pose, linear)
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
        goal.command.position = self.GRIPPER_CLOSED if close else self.GRIPPER_OPEN
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

    def _build_move_goal(self, pose: PoseStamped, linear=False):
        request = MotionPlanRequest()
        request.group_name = self.PLANNING_GROUP
        if linear:
            # OMPL plans in joint space, so even a short "descent" can sweep the
            # fingers sideways into the object. Pilz LIN moves the flange along
            # a straight line instead.
            request.pipeline_id = self.LINEAR_PIPELINE
            request.planner_id = self.LINEAR_PLANNER
        scaling = self.LINEAR_SCALING if linear else self.FREE_SPACE_SCALING
        request.goal_constraints.append(self._make_goal_constraints(pose))
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = scaling
        request.max_acceleration_scaling_factor = scaling

        options = PlanningOptions()
        options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    def _make_goal_constraints(self, pose: PoseStamped, tolerance=None):
        if tolerance is None:
            tolerance = self.POSITION_TOLERANCE
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
