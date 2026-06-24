"""Characterization tests for pressure_to_depth (numpy-only).

depth.py imports only numpy, so it loads directly via the load_module
fixture without rclpy/gtsam. Expected values are physical ground truth for
hydrostatic pressure in seawater:
    P = P_atm + rho * g * h   =>   h = (P - P_atm) / (rho * g)
with P_atm = 101325 Pa, rho = 1025 kg/m^3, g = 9.80665 m/s^2.
Frame convention: NED (z-down), so deeper water => larger POSITIVE depth.
A mismatch means a code bug, not a test bug.
"""
import numpy as np

REL = "stonefish_slam/core/depth.py"

P_ATM = 101325.0
RHO = 1025.0
G = 9.80665


def _fn(load_module):
    return load_module(REL, "depth_under_test").pressure_to_depth


def test_surface_pressure_is_zero_depth(load_module):
    pressure_to_depth = _fn(load_module)
    # At the surface the absolute pressure equals atmospheric -> depth 0.
    assert np.isclose(pressure_to_depth(P_ATM), 0.0)


def test_ten_meters_seawater(load_module):
    pressure_to_depth = _fn(load_module)
    # 10 m of seawater adds rho*g*10 Pa above atmospheric.
    p_10m = P_ATM + RHO * G * 10.0
    # NED z-down: 10 m depth is +10, not -10.
    assert np.isclose(pressure_to_depth(p_10m), 10.0)


def test_above_surface_is_negative_not_clamped(load_module):
    pressure_to_depth = _fn(load_module)
    # Below-atmospheric reading (sensor above the waterline / untared) yields a
    # small negative depth — honest sensor data, not clamped to 0.
    p = P_ATM - RHO * G * 1.0
    assert pressure_to_depth(p) < 0.0
    assert np.isclose(pressure_to_depth(p), -1.0)


def test_monotonic_increasing_with_pressure(load_module):
    pressure_to_depth = _fn(load_module)
    # Deeper (higher pressure) must give strictly larger depth.
    assert pressure_to_depth(P_ATM + 1000.0) > pressure_to_depth(P_ATM)
