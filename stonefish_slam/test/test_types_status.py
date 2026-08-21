"""STATUS enum must not leak `.description` between users.

`STATUS` members are Enum singletons, so `self.description = ...` in
`__init__` creates ONE attribute per member shared by every holder. Two
concurrent scan-matching results that both set `ret.status = STATUS.SUCCESS`
and then write a description end up reading each other's text.

types.py imports gtsam/rclpy at module top, so it is loaded via the
load_factor_graph fixture (which stubs rclpy/cv_bridge) rather than
load_module — the same pattern test_factor_graph.py uses.
"""
import pytest


@pytest.fixture
def STATUS(load_factor_graph):
    import sys
    load_factor_graph  # ensures stubs + sibling modules are in place
    return sys.modules["stonefish_slam.core.types"].STATUS


def test_description_does_not_leak_between_holders(STATUS):
    """Writing a description on one holder must not change another's."""
    a = STATUS.SUCCESS
    a.description = "matching cost 1.00"

    b = STATUS.SUCCESS
    assert b.description != "matching cost 1.00"


def test_str_reflects_only_this_holders_description(STATUS):
    """str(status) must not pick up a description written elsewhere."""
    first = STATUS.NOT_ENOUGH_POINTS
    first.description = "source points 3"
    assert str(first) == "Not enough points: source points 3"

    second = STATUS.NOT_ENOUGH_POINTS
    assert str(second) == "Not enough points"


def test_bool_semantics_preserved(STATUS):
    """SUCCESS is truthy, everything else falsy — unchanged contract."""
    assert bool(STATUS.SUCCESS) is True
    assert bool(STATUS.NOT_ENOUGH_POINTS) is False
    assert bool(STATUS.INITIALIZATION_FAILURE) is False


def test_description_defaults_to_none(STATUS):
    """A fresh reference reports no description."""
    assert STATUS.NOT_CONVERGED.description is None
