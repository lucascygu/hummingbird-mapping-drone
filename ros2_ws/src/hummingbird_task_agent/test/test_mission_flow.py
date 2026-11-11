# Generated / assisted by Claude
import math

import pytest

from hummingbird_task_agent.mission_flow import (
    ACTION_GOTO, ACTION_LAND, ACTION_LOITER, ACTION_RTL, CODE_NO_DRAFT, CODE_PENDING,
    MissionFlow, expand_setpoints, quaternion_from_yaw, yaw_from_quaternion,
)
from hummingbird_task_agent.mission_gate import (
    CODE_NEEDS_ARM, CODE_NEEDS_TAKEOFF, CODE_OUTSIDE_GEOFENCE, GateConfig, MissionRequest, Waypoint,
)


def task(**kw):
    base = dict(mission_id="t1", mission_type="map_region", frame_id="map",
                waypoints=(Waypoint(1, 1, 10), Waypoint(2, 2, 10)))
    base.update(kw)
    return MissionRequest(**base)


def flow(**kw):
    return MissionFlow(GateConfig(allowed_frames=("map",)), **kw)


# --------------------------------------------------------------------------- drafts and approval
def test_ground_task_is_parked_then_released_by_one_approval():
    f = flow()
    out = f.submit_draft(task(), now_s=0.0)
    assert out.state == "pending" and out.result.code == CODE_PENDING and not out.result.accepted
    assert f.pending_ids() == ["t1"]

    r = f.approve("t1", True, True, now_s=10.0)
    assert r.accepted and len(r.setpoints) == 2 and r.frame_id == "map"
    assert f.pending_ids() == []
    # The approval was consumed with the draft: it cannot release anything a second time.
    assert f.approve("t1", True, True, now_s=11.0).code == CODE_NO_DRAFT


def test_partial_approval_is_refused_and_keeps_the_draft():
    f = flow()
    f.submit_draft(task(), now_s=0.0)
    assert f.approve("t1", True, False, now_s=1.0).code == CODE_NEEDS_TAKEOFF
    assert f.approve("t1", False, True, now_s=2.0).code == CODE_NEEDS_ARM
    assert f.pending_ids() == ["t1"]
    assert f.approve("t1", True, True, now_s=3.0).accepted


def test_a_sender_cannot_approve_its_own_draft():
    f = flow()
    out = f.submit_draft(task(arm_approval=True, takeoff_approval=True), now_s=0.0)
    assert out.state == "pending"   # flags inside a GroundTask-style draft are ignored
    assert f.approve("t1", False, False, now_s=1.0).code == CODE_NEEDS_ARM


def test_invalid_draft_is_refused_and_not_stored():
    f = flow()
    out = f.submit_draft(task(waypoints=(Waypoint(500, 0, 10),)), now_s=0.0)
    assert out.state == "refused" and out.result.code == CODE_OUTSIDE_GEOFENCE
    assert f.pending_ids() == []
    assert f.approve("t1", True, True, now_s=1.0).code == CODE_NO_DRAFT


@pytest.mark.parametrize("mission_type", ["rtl", "land"])
def test_safety_tasks_are_released_without_approval(mission_type):
    f = flow()
    out = f.submit_draft(MissionRequest("abort", mission_type, ""), now_s=0.0)
    assert out.state == "released" and out.result.accepted
    assert f.pending_ids() == []


def test_drafts_expire():
    f = flow(draft_ttl_s=30.0)
    f.submit_draft(task(), now_s=0.0)
    assert f.expire(now_s=29.0) == []
    assert f.approve("t1", True, True, now_s=31.0).code == CODE_NO_DRAFT   # too late
    f.submit_draft(task(mission_id="t2"), now_s=40.0)
    assert f.expire(now_s=100.0) == ["t2"]


def test_resubmitting_replaces_and_store_is_bounded():
    f = flow(max_drafts=2)
    f.submit_draft(task(mission_id="a"), now_s=0.0)
    f.submit_draft(task(mission_id="b"), now_s=1.0)
    f.submit_draft(task(mission_id="a", waypoints=(Waypoint(3, 3, 5),)), now_s=2.0)  # replaces a
    assert f.pending_ids() == ["a", "b"]
    f.submit_draft(task(mission_id="c"), now_s=3.0)   # evicts the oldest (b, received at 1.0)
    assert f.pending_ids() == ["a", "c"]
    r = f.approve("a", True, True, now_s=4.0)
    assert r.accepted and r.setpoints == (Waypoint(3, 3, 5),)


def test_one_shot_submit_needs_flags_in_the_request():
    f = flow()
    assert f.submit(task()).code == CODE_NEEDS_ARM
    assert f.submit(task(arm_approval=True, takeoff_approval=True)).accepted


def test_bad_flow_config():
    with pytest.raises(ValueError):
        MissionFlow(GateConfig(), draft_ttl_s=0.0)
    with pytest.raises(ValueError):
        MissionFlow(GateConfig(), max_drafts=0)


# --------------------------------------------------------------------------- setpoint expansion
def test_expand_orders_and_numbers_setpoints():
    f = flow()
    r = f.submit(task(arm_approval=True, takeoff_approval=True, max_speed_mps=2.5,
                      waypoints=(Waypoint(1, 1, 10, yaw=1.0), Waypoint(2, 2, 10), Waypoint(3, 3, 8))))
    specs = expand_setpoints(r)
    assert [s.sequence for s in specs] == [0, 1, 2] and {s.total for s in specs} == {3}
    assert all(s.action == ACTION_GOTO and s.frame_id == "map" and s.mission_id == "t1" for s in specs)
    assert all(s.max_speed_mps == 2.5 for s in specs)
    assert (specs[0].yaw_valid, specs[0].yaw) == (True, 1.0)
    assert (specs[1].yaw_valid, specs[1].yaw) == (False, 0.0)
    assert (specs[2].x, specs[2].y, specs[2].z) == (3.0, 3.0, 8.0)


def test_expand_loiter_holds_at_the_last_waypoint():
    f = flow()
    r = f.submit(task(mission_type="loiter", arm_approval=True, takeoff_approval=True))
    assert [s.action for s in expand_setpoints(r)] == [ACTION_GOTO, ACTION_LOITER]


def test_expand_safety_actions_and_refusals():
    f = flow()
    rtl = expand_setpoints(f.submit(MissionRequest("x", "rtl", "")))
    land = expand_setpoints(f.submit(MissionRequest("y", "land", "")))
    assert [(s.action, s.sequence, s.total) for s in rtl] == [(ACTION_RTL, 0, 1)]
    assert [(s.action, s.sequence, s.total) for s in land] == [(ACTION_LAND, 0, 1)]
    assert expand_setpoints(f.submit(task())) == []   # refused: nothing is ever published


# --------------------------------------------------------------------------- orientation helpers
def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) is None          # identity = unspecified
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 0.0) is None          # all-zero = unspecified
    s = math.sqrt(0.5)
    assert yaw_from_quaternion(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)
    assert yaw_from_quaternion(0.0, 0.0, 2 * s, 2 * s) == pytest.approx(math.pi / 2)   # normalised
    assert yaw_from_quaternion(0.0, 0.0, 1.0, 0.0) == pytest.approx(math.pi)
    with pytest.raises(ValueError):
        yaw_from_quaternion(float("nan"), 0.0, 0.0, 1.0)


@pytest.mark.parametrize("yaw", [-3.0, -1.2, 0.3, 1.5707, 3.1])
def test_yaw_quaternion_roundtrip(yaw):
    assert yaw_from_quaternion(*quaternion_from_yaw(yaw)) == pytest.approx(yaw)
