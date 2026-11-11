# Generated / assisted by Grok
"""Static guard: the bridge must never gain arming / offboard / actuator code paths."""

import pathlib
import re

PKG = pathlib.Path(__file__).resolve().parents[1] / "hummingbird_px4_bridge"
FORBIDDEN = [
    r"VehicleCommand", r"OffboardControlMode", r"TrajectorySetpoint", r"ActuatorMotors",
    r"ActuatorServos", r"VehicleRatesSetpoint", r"VehicleAttitudeSetpoint",
    r"/fmu/in/(?!vehicle_visual_odometry)", r"\barm\(", r"ARM_DISARM",
]


def test_no_forbidden_symbols_in_bridge_sources():
    hits = []
    for path in PKG.glob("*.py"):
        text = path.read_text()
        for pat in FORBIDDEN:
            if re.search(pat, text):
                hits.append(f"{path.name}: {pat}")
    assert not hits, hits
