import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "goal_pose_nav2_bridge.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("goal_pose_nav2_bridge", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


def test_new_request_cancels_active_goal_and_increments_request_id():
    tracker = load_module().GoalRequestTracker()
    active = FakeGoalHandle()
    tracker.active_goal = active

    first_id = tracker.begin_request()
    second_id = tracker.begin_request()

    assert (first_id, second_id) == (1, 2)
    assert active.cancel_count == 1
    assert tracker.active_goal is None


def test_stale_accepted_goal_is_canceled_instead_of_becoming_active():
    tracker = load_module().GoalRequestTracker()
    old_id = tracker.begin_request()
    tracker.begin_request()
    stale_handle = FakeGoalHandle(accepted=True)

    assert tracker.accept_goal(old_id, stale_handle) is False
    assert stale_handle.cancel_count == 1
    assert tracker.active_goal is None


def test_current_accepted_goal_becomes_active():
    tracker = load_module().GoalRequestTracker()
    request_id = tracker.begin_request()
    handle = FakeGoalHandle(accepted=True)

    assert tracker.accept_goal(request_id, handle) is True
    assert tracker.active_goal is handle


def test_relay_source_preserves_pose_and_uses_navigate_to_pose():
    source = MODULE_PATH.read_text()
    assert "NavigateToPose" in source
    assert "goal_msg.pose = copy.deepcopy(pose)" in source
    assert '"/goal_pose"' in source
    assert '"navigate_to_pose"' in source
