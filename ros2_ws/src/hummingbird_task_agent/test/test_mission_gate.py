# Generated / assisted by Grok
# Extended by Claude: failsafe priority, geofence tightening, input validation
import math

import pytest

from hummingbird_task_agent.mission_gate import (
    CODE_BAD_FRAME, CODE_BAD_GEOFENCE, CODE_BAD_ID, CODE_BAD_SPEED, CODE_BAD_TYPE,
    CODE_BAD_WAYPOINT, CODE_NEEDS_ARM, CODE_NEEDS_TAKEOFF, CODE_NO_WAYPOINTS, CODE_OK,
    CODE_OUTSIDE_GEOFENCE, CODE_TOO_MANY_WAYPOINTS,
    GateConfig, MissionRequest, Waypoint, evaluate_mission, evaluate_with_config,
)

NAN = float("nan")
INF = float("inf")


def req(**kw):
    base = dict(mission_id="m1", mission_type="map_region", frame_id="map",
                waypoints=(Waypoint(1, 1, 10),), arm_approval=True, takeoff_approval=True)
    base.update(kw)
    return MissionRequest(**base)


def test_accepts_approved_mission():
    assert evaluate_mission(req()).accepted


def test_refuses_without_arm_approval():
    r = evaluate_mission(req(arm_approval=False))
    assert not r.accepted and "arm_approval" in r.reason


def test_refuses_without_takeoff_approval():
    assert not evaluate_mission(req(takeoff_approval=False)).accepted


def test_refuses_outside_geofence():
    assert not evaluate_mission(req(waypoints=(Waypoint(500, 0, 10),))).accepted
    assert not evaluate_mission(req(waypoints=(Waypoint(0, 0, 80),))).accepted


def test_refuses_unknown_frame_when_allow_list_given():
    assert not evaluate_mission(req(frame_id="camera_init"), allowed_frames=["map"]).accepted


# --------------------------------------------------------------------------- rule 5: failsafe priority
@pytest.mark.parametrize("mission_type", ["rtl", "land", " RTL ", "Land"])
def test_safety_actions_are_never_blocked(mission_type):
    # No approval, no frame, no waypoints, frame allow-list that would refuse anything.
    r = evaluate_mission(
        MissionRequest(mission_id="abort-1", mission_type=mission_type, frame_id=""),
        allowed_frames=["map"],
    )
    assert r.accepted and r.code == CODE_OK
    assert r.mission_type == mission_type.strip().lower()
    assert r.setpoints == ()


def test_safety_action_survives_garbage_fields():
    r = evaluate_mission(MissionRequest(
        mission_id="", mission_type="rtl", frame_id="nonsense",
        waypoints=(Waypoint(NAN, INF, -5),), geofence_max_altitude_m=NAN,
        geofence_radius_m=-1.0, max_speed_mps=INF,
    ), allowed_frames=["map"], max_waypoints=0)
    assert r.accepted
    assert r.mission_id == "safety-rtl"   # an empty id must not block the way home


# --------------------------------------------------------------------------- geofence
def test_request_cannot_widen_site_geofence():
    far = req(waypoints=(Waypoint(500, 0, 10),), geofence_radius_m=10_000.0)
    r = evaluate_mission(far)
    assert not r.accepted and r.code == CODE_OUTSIDE_GEOFENCE
    high = req(waypoints=(Waypoint(0, 0, 80),), geofence_max_altitude_m=10_000.0)
    assert evaluate_mission(high).code == CODE_OUTSIDE_GEOFENCE


def test_request_can_tighten_site_geofence():
    r = evaluate_mission(req(waypoints=(Waypoint(20, 0, 10),), geofence_radius_m=15.0))
    assert not r.accepted and r.code == CODE_OUTSIDE_GEOFENCE
    r = evaluate_mission(req(waypoints=(Waypoint(0, 0, 10),), geofence_max_altitude_m=5.0))
    assert not r.accepted and r.code == CODE_OUTSIDE_GEOFENCE
    ok = evaluate_mission(req(waypoints=(Waypoint(10, 0, 4),),
                              geofence_radius_m=15.0, geofence_max_altitude_m=5.0))
    assert ok.accepted and ok.radius_m == 15.0 and ok.max_altitude_m == 5.0


def test_wider_request_is_accepted_under_site_limits_and_says_so():
    r = evaluate_mission(req(geofence_radius_m=10_000.0))
    assert r.accepted and r.radius_m == 100.0 and "site geofence applies" in r.reason


@pytest.mark.parametrize("field,value", [
    ("geofence_max_altitude_m", -1.0), ("geofence_max_altitude_m", NAN),
    ("geofence_radius_m", -0.5), ("geofence_radius_m", INF),
])
def test_invalid_requested_geofence_is_refused(field, value):
    r = evaluate_mission(req(**{field: value}))
    assert not r.accepted and r.code == CODE_BAD_GEOFENCE


def test_invalid_site_geofence_is_refused():
    assert evaluate_mission(req(), default_radius_m=0.0).code == CODE_BAD_GEOFENCE
    assert evaluate_mission(req(), default_max_altitude_m=NAN).code == CODE_BAD_GEOFENCE


def test_geofence_is_relative_to_home():
    home = (100.0, 200.0, 50.0)
    inside = req(waypoints=(Waypoint(110.0, 200.0, 60.0),))
    assert evaluate_mission(inside, home=home).accepted
    # 60 m in the frame is only 10 m above home, but z=40 is 10 m BELOW home.
    below = req(waypoints=(Waypoint(100.0, 200.0, 40.0),))
    assert evaluate_mission(below, home=home).code == CODE_OUTSIDE_GEOFENCE
    # the same waypoint is far outside when home stays at the origin
    assert evaluate_mission(inside).code == CODE_OUTSIDE_GEOFENCE


# --------------------------------------------------------------------------- waypoint validation
@pytest.mark.parametrize("wp", [
    Waypoint(NAN, 0, 10), Waypoint(0, NAN, 10), Waypoint(0, 0, NAN),
    Waypoint(INF, 0, 10), Waypoint(0, -INF, 10), Waypoint(0, 0, INF),
    Waypoint(0, 0, 10, yaw=NAN), Waypoint(0, 0, 10, yaw=INF),
])
def test_non_finite_waypoints_are_refused(wp):
    r = evaluate_mission(req(waypoints=(Waypoint(1, 1, 10), wp)))
    assert not r.accepted and r.code == CODE_BAD_WAYPOINT
    assert "waypoint[1]" in r.reason


def test_waypoints_required_and_bounded():
    assert evaluate_mission(req(waypoints=())).code == CODE_NO_WAYPOINTS
    many = tuple(Waypoint(1, 1, 10) for _ in range(6))
    assert evaluate_mission(req(waypoints=many), max_waypoints=5).code == CODE_TOO_MANY_WAYPOINTS
    assert evaluate_mission(req(waypoints=many), max_waypoints=6).accepted


def test_yaw_is_passed_through():
    r = evaluate_mission(req(waypoints=(Waypoint(1, 1, 10, yaw=math.pi / 2), Waypoint(2, 2, 10))))
    assert r.accepted
    assert r.setpoints[0].yaw == pytest.approx(math.pi / 2) and r.setpoints[1].yaw is None


# --------------------------------------------------------------------------- speed
def test_speed_limit():
    assert evaluate_mission(req(max_speed_mps=2.0)).max_speed_mps == 2.0
    assert evaluate_mission(req(max_speed_mps=0.0)).accepted
    over = evaluate_mission(req(max_speed_mps=6.0))            # site limit 5 m/s
    assert not over.accepted and over.code == CODE_BAD_SPEED   # refused, never clamped
    assert evaluate_mission(req(max_speed_mps=6.0), max_speed_limit_mps=8.0).accepted
    assert evaluate_mission(req(max_speed_mps=-1.0)).code == CODE_BAD_SPEED
    assert evaluate_mission(req(max_speed_mps=NAN)).code == CODE_BAD_SPEED


# --------------------------------------------------------------------------- codes and approval
def test_refusal_codes():
    assert evaluate_mission(req(mission_id="  ")).code == CODE_BAD_ID
    assert evaluate_mission(req(mission_type="backflip")).code == CODE_BAD_TYPE
    assert evaluate_mission(req(frame_id="")).code == CODE_BAD_FRAME
    assert evaluate_mission(req(frame_id="x"), allowed_frames=["map"]).code == CODE_BAD_FRAME
    assert evaluate_mission(req(arm_approval=False)).code == CODE_NEEDS_ARM
    assert evaluate_mission(req(takeoff_approval=False)).code == CODE_NEEDS_TAKEOFF
    ok = evaluate_mission(req())
    assert ok.code == CODE_OK and ok.frame_id == "map" and ok.mission_type == "map_region"


@pytest.mark.parametrize("mission_type", ["explore", "map_region", "inspect", "loiter"])
def test_every_flying_mission_needs_both_approvals(mission_type):
    assert evaluate_mission(req(mission_type=mission_type)).accepted
    assert evaluate_mission(req(mission_type=mission_type, arm_approval=False)).code == CODE_NEEDS_ARM
    assert evaluate_mission(
        req(mission_type=mission_type, takeoff_approval=False)).code == CODE_NEEDS_TAKEOFF


def test_draft_precheck_skips_only_the_approval():
    draft = req(arm_approval=False, takeoff_approval=False)
    assert evaluate_mission(draft, require_approval=False).accepted
    outside = req(arm_approval=False, takeoff_approval=False, waypoints=(Waypoint(500, 0, 10),))
    assert evaluate_mission(outside, require_approval=False).code == CODE_OUTSIDE_GEOFENCE


# --------------------------------------------------------------------------- site config
def test_gate_config_is_validated_and_applied():
    cfg = GateConfig(max_altitude_m=12.0, radius_m=20.0, home=(5.0, 0.0, 0.0),
                     allowed_frames=("map",), max_waypoints=3, max_speed_limit_mps=2.0)
    assert evaluate_with_config(req(waypoints=(Waypoint(20.0, 0.0, 10.0),)), cfg).accepted
    assert evaluate_with_config(req(waypoints=(Waypoint(30.0, 0.0, 10.0),)), cfg).code == \
        CODE_OUTSIDE_GEOFENCE
    assert evaluate_with_config(req(frame_id="odom"), cfg).code == CODE_BAD_FRAME
    assert evaluate_with_config(req(max_speed_mps=3.0), cfg).code == CODE_BAD_SPEED
    for bad in (dict(max_altitude_m=0.0), dict(radius_m=NAN), dict(max_speed_limit_mps=-1.0),
                dict(home=(0.0, NAN, 0.0)), dict(max_waypoints=0)):
        with pytest.raises(ValueError):
            GateConfig(**bad)
