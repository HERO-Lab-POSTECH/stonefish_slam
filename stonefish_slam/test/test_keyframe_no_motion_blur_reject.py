"""`is_keyframe` must NOT reject a frame for high angular velocity.

The criterion existed but never ran: `slam.py` assigns `frame.twist` at line
824, *after* calling `is_keyframe` at 818, so the guard always read `None`
and fell through. Measured on the tilt30 bag (2026-09-02) by reviving it —
assigning `twist` before the call — the guard costs 19% of keyframes and 43%
of accepted loop closures, and 2D mean trajectory error rises 2.745 -> 6.623 m
(+141%). NSSM is worth 2.8x on this data, so losing loop closures dominates.

So the condition is deleted rather than repaired, and this test pins that: a
frame spinning far faster than the old 0.1 rad/s ceiling is still a keyframe
when the distance thresholds say it is.
"""
import numpy as np
import pytest

import gtsam


class _Twist:
    def __init__(self, wz):
        self.angular = type("A", (), {"x": 0.0, "y": 0.0, "z": wz})()


class _Time:
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class _Frame:
    """Only the fields is_keyframe reads."""

    def __init__(self, sec, dr_pose, twist=None):
        self.time = _Time(sec)
        self.dr_pose = dr_pose
        self.twist = twist


class _FactorGraph:
    def __init__(self, keyframes):
        self.keyframes = keyframes
        self.current_keyframe = keyframes[-1] if keyframes else None


class _Duration:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)


@pytest.fixture
def loc(load_localization):
    module = load_localization
    o = module.Localization.__new__(module.Localization)
    o.keyframe_duration = _Duration(1.0)
    o.keyframe_duration_max = None
    o.keyframe_translation = 1.0
    o.keyframe_rotation = np.deg2rad(10.0)
    origin = _Frame(0, gtsam.Pose2(0.0, 0.0, 0.0))
    o.fg = _FactorGraph([origin])
    return o


# 0.1 rad/s was the deleted ceiling; 5.0 is 50x past it.
@pytest.mark.parametrize("wz", [0.0, 0.5, 5.0])
def test_high_angular_velocity_does_not_block_a_keyframe(loc, wz):
    moved = _Frame(2, gtsam.Pose2(3.0, 0.0, 0.0), twist=_Twist(wz))
    # numpy bool: the final `translation > ...` is a np.bool_, so truthiness only.
    assert loc.is_keyframe(moved)


def test_the_distance_thresholds_still_decide(loc):
    """Deleting the guard must not turn is_keyframe into 'always true'."""
    spinning_but_still = _Frame(2, gtsam.Pose2(0.0, 0.0, 0.0), twist=_Twist(5.0))
    assert not loc.is_keyframe(spinning_but_still)

    too_soon = _Frame(0, gtsam.Pose2(3.0, 0.0, 0.0), twist=_Twist(0.0))
    assert not loc.is_keyframe(too_soon)
