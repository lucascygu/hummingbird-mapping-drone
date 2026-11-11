# Generated / assisted by Grok
"""Pure mission-gate logic. No ROS import, so it can be compiled or unit-tested alone.

The gate never emits actuator or arm commands. It only decides whether a
high-level mission may be turned into setpoint requests.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

SUPPORTED_MISSION_TYPES = frozenset(
    {"explore", "map_region", "inspect", "loiter", "rtl", "land"}
)
# Missions that leave the ground or change arm state require explicit flags.
REQUIRES_ARM = frozenset({"explore", "map_region", "inspect", "loiter", "rtl", "land"})
REQUIRES_TAKEOFF = frozenset({"explore", "map_region", "inspect", "loiter"})

# Development placeholders only. Replace with a surveyed geofence before flight.
DEFAULT_MAX_ALTITUDE_M = 30.0
DEFAULT_RADIUS_M = 100.0
DEFAULT_HOME = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class MissionRequest:
    mission_id: str
    mission_type: str
    frame_id: str
    waypoints: Sequence[Waypoint] = field(default_factory=tuple)
    arm_approval: bool = False
    takeoff_approval: bool = False
    geofence_max_altitude_m: float = 0.0
    geofence_radius_m: float = 0.0


@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reason: str
    mission_id: str
    setpoints: Tuple[Waypoint, ...] = ()


def _horizontal_distance(x: float, y: float, home: Tuple[float, float, float]) -> float:
    dx = x - home[0]
    dy = y - home[1]
    return (dx * dx + dy * dy) ** 0.5


def evaluate_mission(
    request: MissionRequest,
    *,
    default_max_altitude_m: float = DEFAULT_MAX_ALTITUDE_M,
    default_radius_m: float = DEFAULT_RADIUS_M,
    home: Tuple[float, float, float] = DEFAULT_HOME,
    allowed_frames: Optional[Sequence[str]] = None,
) -> GateResult:
    """Return whether the mission may be converted into high-level setpoints.

    This function must not be extended to return arm/actuator commands.
    """
    mission_id = (request.mission_id or "").strip()
    if not mission_id:
        return GateResult(False, "mission_id is empty", mission_id)

    mission_type = (request.mission_type or "").strip().lower()
    if mission_type not in SUPPORTED_MISSION_TYPES:
        return GateResult(
            False,
            f"unsupported mission_type {request.mission_type!r}",
            mission_id,
        )

    frame_id = (request.frame_id or "").strip()
    if not frame_id:
        return GateResult(False, "frame_id is empty", mission_id)

    if allowed_frames is not None and frame_id not in allowed_frames:
        return GateResult(False, f"frame_id {frame_id!r} is not in the allow-list", mission_id)

    if mission_type in REQUIRES_ARM and not request.arm_approval:
        return GateResult(
            False,
            "arming refused: arm_approval is not set; the task agent never arms by itself",
            mission_id,
        )

    if mission_type in REQUIRES_TAKEOFF and not request.takeoff_approval:
        return GateResult(
            False,
            "takeoff refused: takeoff_approval is not set; human approval is required",
            mission_id,
        )

    max_alt = request.geofence_max_altitude_m or default_max_altitude_m
    radius = request.geofence_radius_m or default_radius_m
    if max_alt <= 0.0 or radius <= 0.0:
        return GateResult(False, "geofence limits must be positive", mission_id)

    # rtl/land may have an empty waypoint list (use current position / home).
    if mission_type not in {"rtl", "land"} and not request.waypoints:
        return GateResult(False, "waypoints are required for this mission_type", mission_id)

    violations: List[str] = []
    for i, wp in enumerate(request.waypoints):
        if wp.z > max_alt or wp.z < 0.0:
            violations.append(
                f"waypoint[{i}] altitude {wp.z} m outside [0, {max_alt}]"
            )
        if _horizontal_distance(wp.x, wp.y, home) > radius:
            violations.append(
                f"waypoint[{i}] horizontal distance exceeds geofence radius {radius} m"
            )
    if violations:
        return GateResult(False, "; ".join(violations), mission_id)

    setpoints = tuple(request.waypoints)
    return GateResult(True, "accepted: high-level setpoints only, no actuator command", mission_id, setpoints)
