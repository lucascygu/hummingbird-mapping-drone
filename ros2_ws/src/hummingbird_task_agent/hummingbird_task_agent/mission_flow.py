# Generated / assisted by Claude
"""ROS-free mission flow around the pure gate: one-shot requests, drafts waiting for a human
approval, and the expansion of an accepted mission into ordered setpoint requests.

Two entry paths, one gate (docs/air-ground-cooperation.md):

* ``submit``        - RequestMission service: the approval flags travel with the request.
* ``submit_draft``  - GroundTask topic: a ground robot / planner cannot approve its own task, so a
                      valid task is parked as a draft until an operator calls ``approve``.
                      ``rtl`` and ``land`` are released immediately (failsafe priority).

An approval is valid for exactly one draft and is never stored: a refused approval leaves the
draft pending, an accepted one consumes it, and drafts expire after ``draft_ttl_s``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

from hummingbird_task_agent.mission_gate import (
    GateConfig, GateResult, MissionRequest, SAFETY_ACTIONS, evaluate_with_config,
)

# HighLevelSetpoint.action values (keep in sync with hummingbird_interfaces/msg/HighLevelSetpoint.msg).
ACTION_GOTO = 0
ACTION_LOITER = 1
ACTION_RTL = 2
ACTION_LAND = 3

CODE_PENDING = "pending_approval"
CODE_NO_DRAFT = "no_pending_draft"


@dataclass(frozen=True)
class SetpointSpec:
    """One HighLevelSetpoint, as plain data (the ROS node copies it into the message)."""

    mission_id: str
    mission_type: str
    frame_id: str
    action: int
    sequence: int
    total: int
    x: float
    y: float
    z: float
    yaw_valid: bool
    yaw: float
    max_speed_mps: float


@dataclass(frozen=True)
class DraftOutcome:
    """Result of ``submit_draft``: released at once, parked for approval, or refused."""

    state: str  # "released" | "pending" | "refused"
    result: GateResult


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Heading of a ROS quaternion, or None when the orientation is unspecified.

    Unspecified = all-zero quaternion or the identity (what a default-constructed
    ``geometry_msgs/Pose`` carries). Raises ValueError for non-finite components so that garbage
    input refuses the mission instead of silently becoming "no yaw".
    """
    if not all(math.isfinite(c) for c in (x, y, z, w)):
        raise ValueError("orientation has non-finite components")
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-6:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9:
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of a pure rotation about +z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def expand_setpoints(result: GateResult) -> List[SetpointSpec]:
    """Ordered setpoint requests for an accepted mission (empty list if it was refused).

    Every setpoint carries ``sequence`` / ``total`` so a consumer can tell a complete mission
    from a truncated one. ``rtl`` / ``land`` become a single action-only setpoint.
    """
    if not result.accepted:
        return []
    frame_id = result.frame_id
    if result.mission_type in SAFETY_ACTIONS:
        action = ACTION_RTL if result.mission_type == "rtl" else ACTION_LAND
        return [SetpointSpec(result.mission_id, result.mission_type, frame_id, action, 0, 1,
                             0.0, 0.0, 0.0, False, 0.0, 0.0)]
    total = len(result.setpoints)
    out = []
    for i, wp in enumerate(result.setpoints):
        last = i == total - 1
        action = ACTION_LOITER if (result.mission_type == "loiter" and last) else ACTION_GOTO
        out.append(SetpointSpec(
            result.mission_id, result.mission_type, frame_id, action, i, total,
            float(wp.x), float(wp.y), float(wp.z),
            wp.yaw is not None, float(wp.yaw) if wp.yaw is not None else 0.0,
            float(result.max_speed_mps),
        ))
    return out


class MissionFlow:
    def __init__(self, config: GateConfig, draft_ttl_s: float = 120.0, max_drafts: int = 16):
        if draft_ttl_s <= 0 or max_drafts < 1:
            raise ValueError("draft_ttl_s must be > 0 and max_drafts >= 1")
        self.config = config
        self.draft_ttl_s = float(draft_ttl_s)
        self.max_drafts = int(max_drafts)
        self._drafts: Dict[str, Tuple[MissionRequest, float]] = {}

    # ------------------------------------------------------------------ one-shot service path
    def submit(self, request: MissionRequest) -> GateResult:
        return evaluate_with_config(request, self.config)

    # ------------------------------------------------------------------ draft + approval path
    def submit_draft(self, request: MissionRequest, now_s: float) -> DraftOutcome:
        """A task without approval flags. Safety actions are released; others wait for approval."""
        self.expire(now_s)
        mission_type = (request.mission_type or "").strip().lower()
        if mission_type in SAFETY_ACTIONS:
            return DraftOutcome("released", evaluate_with_config(request, self.config))
        # A draft never carries approval, whatever the sender put in the request.
        draft = replace(request, arm_approval=False, takeoff_approval=False)
        check = evaluate_with_config(draft, self.config, require_approval=False)
        if not check.accepted:
            return DraftOutcome("refused", check)
        if check.mission_id not in self._drafts and len(self._drafts) >= self.max_drafts:
            oldest = min(self._drafts, key=lambda k: self._drafts[k][1])
            del self._drafts[oldest]
        self._drafts[check.mission_id] = (draft, float(now_s))
        pending = GateResult(
            False,
            "pending: valid task parked; an operator must approve arm/takeoff via "
            f"/hummingbird/mission/approve within {self.draft_ttl_s:g} s",
            check.mission_id, (), CODE_PENDING, check.mission_type,
        )
        return DraftOutcome("pending", pending)

    def approve(self, mission_id: str, arm_approval: bool, takeoff_approval: bool,
                now_s: float) -> GateResult:
        """Apply a one-shot human approval to a pending draft and re-run the full gate."""
        self.expire(now_s)
        key = (mission_id or "").strip()
        entry = self._drafts.get(key)
        if entry is None:
            return GateResult(False, f"no pending draft with mission_id {key!r} (never received, "
                              "already released, or expired)", key, (), CODE_NO_DRAFT)
        approved = replace(entry[0], arm_approval=bool(arm_approval),
                           takeoff_approval=bool(takeoff_approval))
        result = evaluate_with_config(approved, self.config)
        if result.accepted:
            del self._drafts[key]
        return result

    def expire(self, now_s: float) -> List[str]:
        """Drop drafts older than the TTL; returns their mission ids."""
        dead = [k for k, (_, t) in self._drafts.items() if now_s - t > self.draft_ttl_s]
        for k in dead:
            del self._drafts[k]
        return dead

    def pending_ids(self) -> List[str]:
        return sorted(self._drafts)
