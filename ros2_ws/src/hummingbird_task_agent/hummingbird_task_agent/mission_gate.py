# Generated / assisted by Grok
# Reviewed and extended by Claude (failsafe priority, geofence tightening, input validation)
"""Pure mission-gate logic. No ROS import, so it can be compiled or unit-tested alone.

The gate never emits actuator or arm commands. It only decides whether a
high-level mission may be turned into setpoint requests.

Rules enforced here (docs/task-agent-design.md):
  2. arm / takeoff need explicit per-request human approval;
  3. every waypoint must be inside the geofence, otherwise the whole mission is refused
     (never silently clipped);
  4. every refusal carries a human-readable ``reason`` and a machine-readable ``code``;
  5. failsafe priority: ``rtl`` and ``land`` are never refused by this gate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# Requests that make the vehicle safer. Rule 5: the gate must never block them, so they skip
# the approval, frame, geofence and waypoint checks entirely.
SAFETY_ACTIONS = frozenset({"rtl", "land"})
SUPPORTED_MISSION_TYPES = frozenset({"explore", "map_region", "inspect", "loiter"}) | SAFETY_ACTIONS
# Missions that leave the ground or change arm state require explicit flags.
REQUIRES_ARM = SUPPORTED_MISSION_TYPES - SAFETY_ACTIONS
REQUIRES_TAKEOFF = SUPPORTED_MISSION_TYPES - SAFETY_ACTIONS

# Development placeholders only. Replace with a surveyed geofence before flight.
DEFAULT_MAX_ALTITUDE_M = 30.0
DEFAULT_RADIUS_M = 100.0
DEFAULT_HOME = (0.0, 0.0, 0.0)
DEFAULT_MAX_WAYPOINTS = 200
DEFAULT_MAX_SPEED_LIMIT_MPS = 5.0

# Machine-readable refusal codes (GateResult.code).
CODE_OK = "ok"
CODE_BAD_ID = "bad_mission_id"
CODE_BAD_TYPE = "unsupported_mission_type"
CODE_BAD_FRAME = "bad_frame"
CODE_NEEDS_ARM = "needs_arm_approval"
CODE_NEEDS_TAKEOFF = "needs_takeoff_approval"
CODE_BAD_GEOFENCE = "bad_geofence"
CODE_NO_WAYPOINTS = "no_waypoints"
CODE_TOO_MANY_WAYPOINTS = "too_many_waypoints"
CODE_BAD_WAYPOINT = "bad_waypoint"
CODE_OUTSIDE_GEOFENCE = "outside_geofence"
CODE_BAD_SPEED = "bad_speed"


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    # Heading in frame_id, radians, counter-clockwise from +x (ROS ENU convention).
    # None = unspecified: the consumer keeps its current heading.
    yaw: Optional[float] = None


@dataclass(frozen=True)
class MissionRequest:
    mission_id: str
    mission_type: str
    frame_id: str
    waypoints: Sequence[Waypoint] = field(default_factory=tuple)
    arm_approval: bool = False
    takeoff_approval: bool = False
    # 0 = use the configured site geofence. A positive value can only TIGHTEN the site geofence.
    geofence_max_altitude_m: float = 0.0
    geofence_radius_m: float = 0.0
    # 0 = unspecified (consumer default). Must not exceed the configured site speed limit.
    max_speed_mps: float = 0.0


@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reason: str
    mission_id: str
    setpoints: Tuple[Waypoint, ...] = ()
    code: str = CODE_OK
    mission_type: str = ""
    max_speed_mps: float = 0.0
    # Geofence actually applied to this mission (0 when the mission was refused early).
    max_altitude_m: float = 0.0
    radius_m: float = 0.0
    # Frame the setpoints are expressed in (may be empty for rtl / land).
    frame_id: str = ""


@dataclass(frozen=True)
class GateConfig:
    """Site limits. Requests may tighten them but never widen them."""

    max_altitude_m: float = DEFAULT_MAX_ALTITUDE_M
    radius_m: float = DEFAULT_RADIUS_M
    home: Tuple[float, float, float] = DEFAULT_HOME
    allowed_frames: Optional[Tuple[str, ...]] = None
    max_waypoints: int = DEFAULT_MAX_WAYPOINTS
    max_speed_limit_mps: float = DEFAULT_MAX_SPEED_LIMIT_MPS

    def __post_init__(self):
        for name in ("max_altitude_m", "radius_m", "max_speed_limit_mps"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be finite and > 0, got {value!r}")
        if len(self.home) != 3 or not all(math.isfinite(c) for c in self.home):
            raise ValueError(f"home must be three finite numbers, got {self.home!r}")
        if self.max_waypoints < 1:
            raise ValueError("max_waypoints must be >= 1")


def _horizontal_distance(x: float, y: float, home: Tuple[float, float, float]) -> float:
    dx = x - home[0]
    dy = y - home[1]
    return (dx * dx + dy * dy) ** 0.5


def _effective_limit(requested: float, site: float) -> Optional[float]:
    """Site limit, tightened by the request. None if the requested value is invalid."""
    if not math.isfinite(requested) or requested < 0.0:
        return None
    if requested == 0.0:
        return site
    return min(requested, site)


def evaluate_mission(
    request: MissionRequest,
    *,
    default_max_altitude_m: float = DEFAULT_MAX_ALTITUDE_M,
    default_radius_m: float = DEFAULT_RADIUS_M,
    home: Tuple[float, float, float] = DEFAULT_HOME,
    allowed_frames: Optional[Sequence[str]] = None,
    max_waypoints: int = DEFAULT_MAX_WAYPOINTS,
    max_speed_limit_mps: float = DEFAULT_MAX_SPEED_LIMIT_MPS,
    require_approval: bool = True,
) -> GateResult:
    """Return whether the mission may be converted into high-level setpoints.

    ``default_max_altitude_m`` / ``default_radius_m`` are the configured site geofence. A request
    may tighten them but can never widen them. ``require_approval=False`` validates everything
    except the human approval flags; it is only used to pre-check a draft that still waits for
    approval and must never be used to release setpoints.

    This function must not be extended to return arm/actuator commands.
    """
    mission_id = (request.mission_id or "").strip()
    mission_type = (request.mission_type or "").strip().lower()

    def refuse(code: str, reason: str) -> GateResult:
        return GateResult(False, reason, mission_id, (), code, mission_type)

    # Rule 5 (failsafe priority): a request to come home or to land is never blocked. It needs
    # no approval, no frame, no waypoints and no geofence check; anything else in it is ignored.
    if mission_type in SAFETY_ACTIONS:
        return GateResult(
            True,
            f"accepted: {mission_type} is a safety action and is never blocked by the gate",
            mission_id or f"safety-{mission_type}",
            (),
            CODE_OK,
            mission_type,
            frame_id=(request.frame_id or "").strip(),
        )

    if not mission_id:
        return refuse(CODE_BAD_ID, "mission_id is empty")

    if mission_type not in SUPPORTED_MISSION_TYPES:
        return refuse(CODE_BAD_TYPE, f"unsupported mission_type {request.mission_type!r}")

    frame_id = (request.frame_id or "").strip()
    if not frame_id:
        return refuse(CODE_BAD_FRAME, "frame_id is empty")

    if allowed_frames is not None and frame_id not in allowed_frames:
        return refuse(CODE_BAD_FRAME, f"frame_id {frame_id!r} is not in the allow-list")

    if require_approval and mission_type in REQUIRES_ARM and not request.arm_approval:
        return refuse(
            CODE_NEEDS_ARM,
            "arming refused: arm_approval is not set; the task agent never arms by itself",
        )

    if require_approval and mission_type in REQUIRES_TAKEOFF and not request.takeoff_approval:
        return refuse(
            CODE_NEEDS_TAKEOFF,
            "takeoff refused: takeoff_approval is not set; human approval is required",
        )

    if not (math.isfinite(default_max_altitude_m) and default_max_altitude_m > 0.0
            and math.isfinite(default_radius_m) and default_radius_m > 0.0):
        return refuse(CODE_BAD_GEOFENCE, "configured geofence limits must be finite and positive")
    max_alt = _effective_limit(request.geofence_max_altitude_m, default_max_altitude_m)
    radius = _effective_limit(request.geofence_radius_m, default_radius_m)
    if max_alt is None or radius is None:
        return refuse(
            CODE_BAD_GEOFENCE,
            "requested geofence limits must be finite and >= 0 (0 = use the site geofence)",
        )

    speed = request.max_speed_mps
    if not math.isfinite(speed) or speed < 0.0:
        return refuse(CODE_BAD_SPEED, "max_speed_mps must be finite and >= 0 (0 = unspecified)")
    if speed > max_speed_limit_mps:
        return refuse(
            CODE_BAD_SPEED,
            f"max_speed_mps {speed} exceeds the site speed limit {max_speed_limit_mps} m/s",
        )

    if not request.waypoints:
        return refuse(CODE_NO_WAYPOINTS, "waypoints are required for this mission_type")
    if len(request.waypoints) > max_waypoints:
        return refuse(
            CODE_TOO_MANY_WAYPOINTS,
            f"{len(request.waypoints)} waypoints exceed the limit of {max_waypoints}",
        )

    malformed: List[str] = []
    violations: List[str] = []
    for i, wp in enumerate(request.waypoints):
        # NaN compares False against every limit, so it must be rejected explicitly.
        if not (math.isfinite(wp.x) and math.isfinite(wp.y) and math.isfinite(wp.z)):
            malformed.append(f"waypoint[{i}] has non-finite coordinates")
            continue
        if wp.yaw is not None and not math.isfinite(wp.yaw):
            malformed.append(f"waypoint[{i}] has a non-finite yaw")
            continue
        altitude = wp.z - home[2]
        if altitude > max_alt or altitude < 0.0:
            violations.append(
                f"waypoint[{i}] altitude {altitude:g} m above home outside [0, {max_alt:g}]"
            )
        if _horizontal_distance(wp.x, wp.y, home) > radius:
            violations.append(
                f"waypoint[{i}] horizontal distance exceeds geofence radius {radius:g} m"
            )
    if malformed:
        return refuse(CODE_BAD_WAYPOINT, "; ".join(malformed))
    if violations:
        return refuse(CODE_OUTSIDE_GEOFENCE, "; ".join(violations))

    reason = "accepted: high-level setpoints only, no actuator command"
    if (request.geofence_max_altitude_m > default_max_altitude_m
            or request.geofence_radius_m > default_radius_m):
        reason += " (requested geofence is wider than the site geofence; the site geofence applies)"
    return GateResult(
        True,
        reason,
        mission_id,
        tuple(request.waypoints),
        CODE_OK,
        mission_type,
        speed,
        max_alt,
        radius,
        frame_id,
    )


def evaluate_with_config(
    request: MissionRequest, config: GateConfig, *, require_approval: bool = True
) -> GateResult:
    return evaluate_mission(
        request,
        default_max_altitude_m=config.max_altitude_m,
        default_radius_m=config.radius_m,
        home=config.home,
        allowed_frames=config.allowed_frames,
        max_waypoints=config.max_waypoints,
        max_speed_limit_mps=config.max_speed_limit_mps,
        require_approval=require_approval,
    )
