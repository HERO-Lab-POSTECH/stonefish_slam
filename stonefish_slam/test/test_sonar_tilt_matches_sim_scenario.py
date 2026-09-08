"""sonar.yaml's sonar_tilt_deg must equal the tilt the simulator actually mounts the FLS at.

The two numbers live in two repos — the FLS <origin rpy> in stonefish_sim's
bluerov2.scn and sonar_tilt_deg in this repo's sonar.yaml — and nothing held
them together: they drifted twice (sim 54636c6 read the .scn roll as the tilt
and moved the real down-angle 30° -> 10° while sonar.yaml kept 30°).

Geometry, from the simulator source rather than from the .scn comments:
- ScenarioParser.cpp:4211 builds the sensor origin as
  Quaternion(rpy.z, rpy.y, rpy.x) with BT_EULER_DEFAULT_ZYX, i.e.
  R = Rz(yaw) · Ry(pitch) · Rx(roll).
- Camera.cpp:76 makes a camera-type sensor (FLS inherits it) look along its
  local +Z, so the look vector in the body (FRD) frame is column 2 of R.
- The down-tilt is the elevation of that vector below the body x-y plane.

The check is done on the full rotation, not on the "90 - roll" shortcut, so a
future edit to pitch or yaw is caught as well. It needs the sibling
stonefish_sim checkout that stonefish.repos lays out next to this repo; without
it (slam CI runs alone) the test skips rather than fabricating a pass.
"""
import math
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SONAR_YAML = REPO_ROOT / "config" / "sonar.yaml"
SIM_SCN = (
    REPO_ROOT.parent / "stonefish_sim" / "stonefish_description" / "data"
    / "robots" / "bluerov2" / "bluerov2.scn"
)


def _fls_origin_rpy(scn_text):
    """(roll, pitch, yaw) in radians of the <origin> inside <sensor name="fls">."""
    sensor = re.search(r'<sensor name="fls"[^>]*>(.*?)</sensor>', scn_text, re.S)
    assert sensor, 'no <sensor name="fls"> block in the scenario'
    origin = re.search(r'<origin[^>]*\brpy="([^"]+)"', sensor.group(1))
    assert origin, "FLS sensor has no <origin rpy=...>"
    values = [float(v) for v in origin.group(1).split()]
    assert len(values) == 3, f"rpy needs 3 values, got {values}"
    return tuple(values)


def look_vector_body(roll, pitch, yaw):
    """Column 2 (+Z) of Rz(yaw)·Ry(pitch)·Rx(roll) — the FLS look direction in FRD."""
    # +Z after Rx(roll)
    x, y, z = 0.0, -math.sin(roll), math.cos(roll)
    # Ry(pitch)
    x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)
    # Rz(yaw)
    x, y = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
    return x, y, z


def down_tilt_deg(roll, pitch, yaw):
    """Elevation of the look vector below the body x-y plane (FRD: +z is down)."""
    x, y, z = look_vector_body(roll, pitch, yaw)
    return math.degrees(math.atan2(z, math.hypot(x, y)))


def test_geometry_helper_reproduces_the_known_mountings():
    # roll 60° = the historic mounting that matched sonar.yaml's 30°,
    # roll 80° = the mounting 54636c6 introduced, which is 10° down, not 80°.
    assert down_tilt_deg(math.radians(60), 0.0, math.radians(90)) == pytest.approx(30.0, abs=1e-6)
    assert down_tilt_deg(math.radians(80), 0.0, math.radians(90)) == pytest.approx(10.0, abs=1e-6)


@pytest.mark.xfail(
    strict=True,
    reason="A2 pending: bluerov2.scn mounts the FLS 10° down (roll 80°) while "
           "sonar.yaml says 30°. Drop this marker in the PR that aligns them.",
)
def test_sonar_tilt_deg_matches_the_simulator_mounting():
    if not SIM_SCN.exists():
        pytest.skip(f"sibling stonefish_sim checkout not found at {SIM_SCN}")
    yaml = pytest.importorskip("yaml")
    with open(SONAR_YAML) as fh:
        tilt_cfg = yaml.safe_load(fh)["slam_node"]["ros__parameters"]["sonar"]["sonar_tilt_deg"]

    roll, pitch, yaw = _fls_origin_rpy(SIM_SCN.read_text())
    x, _, _ = look_vector_body(roll, pitch, yaw)
    assert x > 0, "FLS look vector does not point forward — mounting is not a tilt at all"

    tilt_sim = down_tilt_deg(roll, pitch, yaw)
    assert tilt_sim == pytest.approx(tilt_cfg, abs=0.05), (
        f"simulator mounts the FLS {tilt_sim:.2f}° down (rpy={roll:.5f} {pitch:.5f} {yaw:.5f}) "
        f"but sonar.yaml says {tilt_cfg}° — fix both repos together (CONTRIBUTING.md §5)"
    )
