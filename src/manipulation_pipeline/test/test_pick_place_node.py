from unittest.mock import MagicMock, patch

import pytest
import rclpy
from rclpy.task import Future
from tf2_ros import TransformException

from moveit_msgs.msg import CollisionObject, PlanningScene

from manipulation_pipeline.pick_place_node import PickPlaceNode, make_pose


@pytest.fixture
def node():
    with patch('manipulation_pipeline.pick_place_node.ActionClient') as mock_client_cls:
        mock_client_cls.return_value.wait_for_server.return_value = True
        n = PickPlaceNode()
    yield n
    n.destroy_node()


def _done_future(result):
    f = Future()
    f.set_result(result)
    return f


# ---------- pure helpers ----------

def test_make_pose_sets_position_and_default_down_orientation():
    pose = make_pose('panda_link0', 0.1, 0.2, 0.3)
    assert pose.header.frame_id == 'panda_link0'
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (0.1, 0.2, 0.3)
    assert pose.pose.orientation.x == 1.0 and pose.pose.orientation.w == 0.0


def test_make_goal_constraints_carries_link_and_tolerance(node):
    pose = make_pose('panda_link0', 0.5, 0.0, 0.2)
    constraints = node._make_goal_constraints(pose, tolerance=0.03)

    pos = constraints.position_constraints[0]
    assert pos.link_name == node.EE_LINK
    assert list(pos.constraint_region.primitives[0].dimensions) == [0.03]

    orient = constraints.orientation_constraints[0]
    assert orient.orientation == pose.pose.orientation


# ---------- TF2 lookup ----------

def test_lookup_frame_pose_success(node):
    fake_tf = MagicMock()
    fake_tf.transform.translation.x = 0.6
    fake_tf.transform.translation.y = 0.05
    fake_tf.transform.translation.z = 0.02
    node.tf_buffer.lookup_transform = MagicMock(return_value=fake_tf)

    assert node._lookup_frame_pose('object') == (0.6, 0.05, 0.02)


def test_lookup_frame_pose_returns_none_on_tf_exception(node):
    node.tf_buffer.lookup_transform = MagicMock(
        side_effect=TransformException('frame does not exist'))
    assert node._lookup_frame_pose('object') is None


# ---------- _send_and_wait: the execution monitor ----------

def test_send_and_wait_success(node, monkeypatch):
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k: None)

    goal_handle = MagicMock(accepted=True)
    result_msg = MagicMock()
    goal_handle.get_result_async.return_value = _done_future(MagicMock(result=result_msg))

    client = MagicMock()
    client.send_goal_async.return_value = _done_future(goal_handle)

    ok, result = node._send_and_wait(client, MagicMock(), timeout_sec=1.0, description='t')
    assert ok is True and result is result_msg


def test_send_and_wait_goal_rejected(node, monkeypatch):
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k: None)

    goal_handle = MagicMock(accepted=False)
    client = MagicMock()
    client.send_goal_async.return_value = _done_future(goal_handle)

    ok, result = node._send_and_wait(client, MagicMock(), timeout_sec=1.0, description='t')
    assert ok is False and result is None


def test_send_and_wait_timeout_on_acceptance(node, monkeypatch):
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k: None)

    client = MagicMock()
    client.send_goal_async.return_value = Future()  # never completes

    ok, result = node._send_and_wait(client, MagicMock(), timeout_sec=1.0, description='t')
    assert ok is False and result is None


def test_send_and_wait_timeout_on_result_cancels_goal(node, monkeypatch):
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k: None)

    goal_handle = MagicMock(accepted=True)
    goal_handle.get_result_async.return_value = Future()  # never completes

    client = MagicMock()
    client.send_goal_async.return_value = _done_future(goal_handle)

    ok, result = node._send_and_wait(client, MagicMock(), timeout_sec=1.0, description='t')
    assert ok is False and result is None
    goal_handle.cancel_goal_async.assert_called_once()


# ---------- retry / recovery ----------

def test_move_with_recovery_succeeds_first_try(node):
    node._move_to_pose = MagicMock(return_value=True)
    node._recover_to_home = MagicMock()

    pose = make_pose('panda_link0', 0.1, 0.1, 0.1)
    assert node._move_with_recovery(pose, 'approach-pick') is True
    node._move_to_pose.assert_called_once()
    node._recover_to_home.assert_not_called()


def test_move_with_recovery_retries_once_then_succeeds(node):
    node._move_to_pose = MagicMock(side_effect=[False, True])
    node._recover_to_home = MagicMock()

    pose = make_pose('panda_link0', 0.1, 0.1, 0.1)
    assert node._move_with_recovery(pose, 'approach-pick') is True
    assert node._move_to_pose.call_count == 2
    node._recover_to_home.assert_called_once()


def test_move_with_recovery_gives_up_after_max_attempts(node):
    node._move_to_pose = MagicMock(return_value=False)
    node._recover_to_home = MagicMock()

    pose = make_pose('panda_link0', 0.1, 0.1, 0.1)
    assert node._move_with_recovery(pose, 'approach-pick') is False
    assert node._move_to_pose.call_count == node.MAX_ATTEMPTS
    assert node._recover_to_home.call_count == node.MAX_ATTEMPTS - 1


# ---------- straight-line moves with free-space fallback ----------

def test_linear_move_falls_back_to_free_space_planner(node):
    node._move_to_pose = MagicMock(side_effect=[False, True])
    node._recover_to_home = MagicMock()

    pose = make_pose('panda_link0', 0.1, 0.1, 0.1)
    assert node._move_with_recovery(pose, 'descend-pick', linear=True) is True
    assert node._move_to_pose.call_args_list[0].args[2] is True    # straight line first
    assert node._move_to_pose.call_args_list[1].kwargs['linear'] is False  # then fallback
    node._recover_to_home.assert_not_called()


def test_build_move_goal_uses_pilz_lin_and_lower_scaling_for_linear_moves(node):
    pose = make_pose('panda_link0', 0.5, 0.0, 0.2)

    linear = node._build_move_goal(pose, linear=True).request
    assert linear.pipeline_id == node.LINEAR_PIPELINE
    assert linear.planner_id == node.LINEAR_PLANNER
    assert linear.max_acceleration_scaling_factor == node.LINEAR_SCALING

    free = node._build_move_goal(pose).request
    assert free.pipeline_id == ''  # default (OMPL) pipeline
    assert free.max_acceleration_scaling_factor == node.FREE_SPACE_SCALING


def test_default_goal_tolerance_is_tight(node):
    constraints = node._make_goal_constraints(make_pose('panda_link0', 0.5, 0.0, 0.2))
    region = constraints.position_constraints[0].constraint_region
    assert list(region.primitives[0].dimensions) == [node.POSITION_TOLERANCE]


# ---------- fingertip (TCP) offset ----------

def test_tcp_pose_raises_flange_goal_by_tcp_offset(node):
    pose = node._tcp_pose(0.6, 0.05, 0.02)
    assert pose.pose.position.z == pytest.approx(0.02 + node.TCP_OFFSET)
    assert (pose.pose.position.x, pose.pose.position.y) == (0.6, 0.05)


# ---------- planning scene ----------

def _scene_response(success):
    return _done_future(MagicMock(success=success))


def test_apply_scene_success(node, monkeypatch):
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k: None)
    node.scene_client = MagicMock()
    node.scene_client.wait_for_service.return_value = True
    node.scene_client.call_async.return_value = _scene_response(True)

    assert node._apply_scene(PlanningScene(), 't') is True


def test_apply_scene_rejected(node, monkeypatch):
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k: None)
    node.scene_client = MagicMock()
    node.scene_client.wait_for_service.return_value = True
    node.scene_client.call_async.return_value = _scene_response(False)

    assert node._apply_scene(PlanningScene(), 't') is False


def test_apply_scene_service_unavailable(node):
    node.scene_client = MagicMock()
    node.scene_client.wait_for_service.return_value = False

    assert node._apply_scene(PlanningScene(), 't') is False
    node.scene_client.call_async.assert_not_called()


def test_add_object_to_scene_builds_box_at_object_pose(node):
    node._apply_scene = MagicMock(return_value=True)

    assert node._add_object_to_scene(0.6, 0.05, 0.02) is True
    scene = node._apply_scene.call_args.args[0]
    obj = scene.world.collision_objects[0]
    assert obj.id == node.OBJECT_ID and obj.operation == CollisionObject.ADD
    assert list(obj.primitives[0].dimensions) == [node.OBJECT_SIZE] * 3
    assert obj.primitive_poses[0].position.z == 0.02


def test_attach_and_detach_target_the_hand_with_touch_links(node):
    node._apply_scene = MagicMock(return_value=True)

    node._attach_object()
    node._detach_object()
    (attach_scene, _), (detach_scene, _) = [c.args for c in node._apply_scene.call_args_list]

    attach = attach_scene.robot_state.attached_collision_objects[0]
    assert attach.link_name == node.HAND_LINK
    assert attach.object.operation == CollisionObject.ADD
    assert set(attach.touch_links) == set(node.TOUCH_LINKS)
    detach = detach_scene.robot_state.attached_collision_objects[0]
    assert detach.object.operation == CollisionObject.REMOVE


def test_gripper_closes_to_object_width(node):
    node._send_and_wait = MagicMock(return_value=(True, None))

    node._monitored_gripper(close=True, description='grasp')
    goal = node._send_and_wait.call_args.args[1]
    assert goal.command.position == pytest.approx(node.OBJECT_SIZE / 2)


# ---------- top-level sequencing ----------

OBJECT = (0.6, 0.05, 0.02)
PLATFORM = (0.35, -0.25, 0.05)


def _mock_sequence(node, move_result=True):
    node._lookup_frame_pose = MagicMock(side_effect=[OBJECT, PLATFORM])
    node._add_object_to_scene = MagicMock(return_value=True)
    node._attach_object = MagicMock(return_value=True)
    node._detach_object = MagicMock(return_value=True)
    node._monitored_gripper = MagicMock(return_value=True)
    if isinstance(move_result, list):
        node._move_with_recovery = MagicMock(side_effect=move_result)
    else:
        node._move_with_recovery = MagicMock(return_value=move_result)


def test_run_pick_and_place_aborts_when_tf_lookup_fails(node):
    node._lookup_frame_pose = MagicMock(return_value=None)
    node._move_with_recovery = MagicMock()
    node._monitored_gripper = MagicMock()

    assert node.run_pick_and_place() is False
    node._move_with_recovery.assert_not_called()


def test_run_pick_and_place_aborts_when_scene_setup_fails(node):
    _mock_sequence(node)
    node._add_object_to_scene = MagicMock(return_value=False)

    assert node.run_pick_and_place() is False
    node._move_with_recovery.assert_not_called()
    node._monitored_gripper.assert_not_called()


def test_run_pick_and_place_runs_full_sequence_on_success(node):
    _mock_sequence(node)

    assert node.run_pick_and_place() is True
    assert node._move_with_recovery.call_count == 7  # approach/descend/retreat x2 + home
    assert node._monitored_gripper.call_count == 3   # open + grasp + release
    node._add_object_to_scene.assert_called_once_with(*OBJECT)
    node._attach_object.assert_called_once()
    node._detach_object.assert_called_once()


def test_run_pick_and_place_targets_fingertips_not_flange(node):
    _mock_sequence(node)
    node.run_pick_and_place()
    calls = node._move_with_recovery.call_args_list

    descend_pick_pose, _, descend_pick_linear = calls[1].args
    assert descend_pick_pose.pose.position.z == pytest.approx(
        OBJECT[2] + node.GRASP_HEIGHT + node.TCP_OFFSET)
    assert descend_pick_linear is True

    descend_place_pose, _, _ = calls[4].args
    assert descend_place_pose.pose.position.z == pytest.approx(
        PLATFORM[2] + node.OBJECT_SIZE / 2 + node.PLACE_CLEARANCE + node.TCP_OFFSET)


def test_run_pick_and_place_aborts_mid_sequence_on_move_failure(node):
    # Succeed on approach-pick, fail on descend-pick -> should abort right there.
    _mock_sequence(node, move_result=[True, False])

    assert node.run_pick_and_place() is False
    assert node._move_with_recovery.call_count == 2
    assert node._monitored_gripper.call_count == 1  # only the initial open
    node._attach_object.assert_not_called()
