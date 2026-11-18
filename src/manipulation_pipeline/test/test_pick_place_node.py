from unittest.mock import MagicMock, patch

import pytest
import rclpy
from rclpy.task import Future
from tf2_ros import TransformException

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

    assert node._move_with_recovery(make_pose('panda_link0', 0.1, 0.1, 0.1), 'approach-pick') is True
    node._move_to_pose.assert_called_once()
    node._recover_to_home.assert_not_called()


def test_move_with_recovery_retries_once_then_succeeds(node):
    node._move_to_pose = MagicMock(side_effect=[False, True])
    node._recover_to_home = MagicMock()

    assert node._move_with_recovery(make_pose('panda_link0', 0.1, 0.1, 0.1), 'approach-pick') is True
    assert node._move_to_pose.call_count == 2
    node._recover_to_home.assert_called_once()


def test_move_with_recovery_gives_up_after_max_attempts(node):
    node._move_to_pose = MagicMock(return_value=False)
    node._recover_to_home = MagicMock()

    assert node._move_with_recovery(make_pose('panda_link0', 0.1, 0.1, 0.1), 'approach-pick') is False
    assert node._move_to_pose.call_count == node.MAX_ATTEMPTS
    assert node._recover_to_home.call_count == node.MAX_ATTEMPTS - 1


# ---------- top-level sequencing ----------

def test_run_pick_and_place_aborts_when_tf_lookup_fails(node):
    node._lookup_frame_pose = MagicMock(return_value=None)
    node._move_with_recovery = MagicMock()
    node._monitored_gripper = MagicMock()

    assert node.run_pick_and_place() is False
    node._move_with_recovery.assert_not_called()


def test_run_pick_and_place_runs_full_sequence_on_success(node):
    node._lookup_frame_pose = MagicMock(side_effect=[(0.6, 0.05, 0.02), (0.35, -0.25, 0.05)])
    node._move_with_recovery = MagicMock(return_value=True)
    node._monitored_gripper = MagicMock(return_value=True)

    assert node.run_pick_and_place() is True
    assert node._move_with_recovery.call_count == 7  # approach/descend/retreat x2 pairs + home
    assert node._monitored_gripper.call_count == 2   # grasp + release


def test_run_pick_and_place_aborts_mid_sequence_on_move_failure(node):
    node._lookup_frame_pose = MagicMock(side_effect=[(0.6, 0.05, 0.02), (0.35, -0.25, 0.05)])
    # Succeed on approach-pick, fail on descend-pick -> should abort right there.
    node._move_with_recovery = MagicMock(side_effect=[True, False])
    node._monitored_gripper = MagicMock(return_value=True)

    assert node.run_pick_and_place() is False
    assert node._move_with_recovery.call_count == 2
    node._monitored_gripper.assert_not_called()
