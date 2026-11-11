from hummingbird_task_agent.mission_gate import MissionRequest, Waypoint, evaluate_mission


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
