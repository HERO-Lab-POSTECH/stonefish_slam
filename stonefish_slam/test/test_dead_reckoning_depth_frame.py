"""Freeze the dead-reckoning depth/frame convention established by tracing.

`core/depth.py::pressure_to_depth` returns NED depth (deeper => larger
POSITIVE). `core/dead_reckoning.py` puts that value straight into the pose's
z and publishes it under frames named `odom` -> `base_link`, which
CONVENTIONS.md previously described as REP-105 ENU (z-up). That reads like a
sign error, so the consumers were traced end to end before touching anything:

  - the node publishes /dead_reck/odom, /dead_reck/path, /dead_reck/key_traj
    and the odom->base_link TF;
  - nothing in the repo subscribes to any of them - `slam.py` takes
    `LOCALIZATION_ODOM_TOPIC = "/bluerov2/odometry"`, the simulator's own NED
    odometry, per the deliberate note at utils/topics.py:18;
  - `launch/slam.launch.py` does not start the dead-reckoning node at all,
    and the rviz config references no /dead_reck topic;
  - every TF the node emits is identity in rotation, so no frame conversion
    happens anywhere along the chain.

So the chain is internally consistent NED end to end and no compensating sign
flip exists downstream to break. Negating z here would make the published
depth wrong; the defect was in the documentation, which now records the
frames as NED. This test pins the code side of that decision so a later
"fix" to the sign has to confront the trace.

Real-vehicle sign-off is deferred (see P4_FLAGS.md).
"""
import ast
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DR_PY = REPO_ROOT / "stonefish_slam" / "core" / "dead_reckoning.py"
DEPTH_REL = "stonefish_slam/core/depth.py"


def test_depth_is_positive_down(load_module):
    """The value dead_reckoning puts in z is NED, positive downward."""
    pressure_to_depth = load_module(DEPTH_REL, "depth_ned_convention").pressure_to_depth
    p_10m = 101325.0 + 1025.0 * 9.80665 * 10.0
    assert pressure_to_depth(p_10m) > 0
    assert np.isclose(pressure_to_depth(p_10m), 10.0)


def test_dead_reckoning_passes_depth_through_unnegated():
    """No sign flip between pressure_to_depth and the published pose z.

    dead_reckoning.py imports rclpy/gtsam at module top, so this is a static
    AST check (CONVENTIONS.md §2.8): the call result is bound straight to
    curr_depth with no unary minus.
    """
    tree = ast.parse(DR_PY.read_text())
    assignments = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(t, ast.Name) and t.id == "curr_depth" for t in node.targets
        )
    ]
    assert assignments, "curr_depth assignment not found in dead_reckoning.py"
    for node in assignments:
        assert not isinstance(node.value, ast.UnaryOp), (
            "curr_depth negates pressure_to_depth; the dead-reckoning chain is "
            "NED end to end and has no consumer expecting ENU (see module "
            "docstring for the trace)"
        )
        assert isinstance(node.value, ast.Call), (
            "curr_depth is no longer the pressure_to_depth result; re-run the "
            "consumer trace before changing this"
        )


def test_dead_reckoning_output_has_no_in_repo_consumer():
    """The premise of the trace: nothing subscribes to the /dead_reck topics.

    If this ever fails, the new consumer decides the convention and the
    sign question must be reopened rather than assumed settled.
    """
    sources = [
        p
        for p in (REPO_ROOT / "stonefish_slam").rglob("*.py")
        if "__pycache__" not in p.parts and p != DR_PY and "test" not in p.parts
    ]
    sources += list((REPO_ROOT / "launch").glob("*.py"))
    # The stated premise covers rviz and config too, so scan them as well.
    sources += list((REPO_ROOT / "rviz").glob("*.rviz"))
    sources += list((REPO_ROOT / "config").rglob("*.yaml"))

    offenders = [
        str(p.relative_to(REPO_ROOT))
        for p in sources
        if "/dead_reck/" in p.read_text()
    ]
    assert not offenders, (
        f"{offenders} now reference the dead-reckoning topics; re-examine the "
        "NED/ENU decision recorded in this module's docstring"
    )
