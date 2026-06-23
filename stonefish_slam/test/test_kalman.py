"""Unit tests for the pure Kalman filter math extracted from KalmanNode.

Expected values are mathematical ground truth from the standard discrete
linear Kalman filter equations (Welch & Bishop, TR 95-041):
    predict:  x- = A x ;  P- = A P A^T + Q
    correct:  K = P- H^T (H P- H^T + R)^-1
              x = x- + K (z - H x-) ;  P = (I - K H) P-
Hand-computed and cross-checked with numpy. A mismatch means a code bug,
not a test bug (see CONVENTIONS.md §2.8).
"""
import numpy as np

REL = "stonefish_slam/core/kalman_filter.py"


def _fns(load_module):
    mod = load_module(REL, "kalman_filter_under_test")
    return mod.kalman_predict, mod.kalman_correct


def test_predict_propagates_state_and_covariance(load_module):
    kalman_predict, _ = _fns(load_module)
    A = np.array([[1.0, 1.0], [0.0, 1.0]])
    x = np.array([[1.0], [2.0]])
    P = np.eye(2)
    Q = np.diag([0.1, 0.1])

    pred_x, pred_P = kalman_predict(x, P, A, Q)

    # x- = A x = [[3], [2]]
    assert np.allclose(pred_x, np.array([[3.0], [2.0]]))
    # P- = A P A^T + Q = [[2.1, 1.0], [1.0, 1.1]]
    assert np.allclose(pred_P, np.array([[2.1, 1.0], [1.0, 1.1]]))


def test_correct_applies_measurement_update(load_module):
    _, kalman_correct = _fns(load_module)
    pred_x = np.array([[3.0], [2.0]])
    pred_P = np.diag([2.0, 2.0])
    H = np.array([[1.0, 0.0]])
    R = np.array([[1.0]])
    z = np.array([[5.0]])

    corr_x, corr_P = kalman_correct(pred_x, pred_P, z, H, R)

    # K = P- H^T (H P- H^T + R)^-1 = [[2],[0]] / 3 -> first row 2/3
    # x = x- + K (z - H x-) = [[3 + (2/3)*2], [2]] = [[4.3333...], [2]]
    assert np.allclose(corr_x, np.array([[13.0 / 3.0], [2.0]]))
    # P = (I - K H) P- = [[2/3, 0], [0, 2]]
    assert np.allclose(corr_P, np.array([[2.0 / 3.0, 0.0], [0.0, 2.0]]))


def test_correct_with_zero_innovation_leaves_state(load_module):
    # If the measurement exactly matches the prediction, the state estimate
    # must not move (z - H x- = 0), though covariance still shrinks.
    _, kalman_correct = _fns(load_module)
    pred_x = np.array([[4.0], [1.0]])
    pred_P = np.diag([1.0, 1.0])
    H = np.array([[1.0, 0.0]])
    R = np.array([[0.5]])
    z = np.array([[4.0]])  # equals H @ pred_x

    corr_x, corr_P = kalman_correct(pred_x, pred_P, z, H, R)

    assert np.allclose(corr_x, pred_x)
    # covariance of the observed component must shrink, unobserved unchanged
    assert corr_P[0, 0] < pred_P[0, 0]
    assert np.isclose(corr_P[1, 1], pred_P[1, 1])


# --- pressure_to_depth ---
# CHARACTERIZATION tests: these pin the CURRENT behavior of the extracted
# formula, which is PHYSICALLY WRONG — it divides a Pascal pressure by 101.325
# (1 atm in kPa, not Pa), a 1000x unit mismatch confirmed against the Stonefish
# publisher (ROS2Interface.cpp emits raw Pa; environment.scn sets 101300 Pa).
# See P4_FLAGS.md. The expected values below are the code's present output, not
# the correct depth; they exist so a P4 fix has a regression net.


def test_pressure_to_depth_current_behavior_at_assumed_surface(load_module):
    pressure_to_depth = load_module(REL, "kf_p2d").pressure_to_depth
    # If fed 101.325 (the kPa value the buggy formula implicitly assumes is
    # "surface"), (P/101.325 - 1) = 0, so depth = -offset_z.
    assert np.isclose(pressure_to_depth(101.325, 2.5), -2.5)


def test_pressure_to_depth_current_behavior_one_tenth_atm(load_module):
    pressure_to_depth = load_module(REL, "kf_p2d2").pressure_to_depth
    # 111.4575 = 1.1 * 101.325 -> -((1.1)-1)*10 - 0 = -1.0 (code's own scale)
    assert np.isclose(pressure_to_depth(111.4575, 0.0), -1.0)


# --- dvl_velocity_rejected ---
# Pure gate, no bug: True means "reject this DVL measurement" (a component
# exceeds max_velocity), mirroring the early-return in the original callback.


def test_dvl_gate_accepts_in_range(load_module):
    dvl_velocity_rejected = load_module(REL, "kf_dvl").dvl_velocity_rejected
    assert dvl_velocity_rejected(0.1, 0.2, 0.3, 5.0) is False


def test_dvl_gate_rejects_when_any_component_exceeds(load_module):
    dvl_velocity_rejected = load_module(REL, "kf_dvl2").dvl_velocity_rejected
    assert dvl_velocity_rejected(6.0, 0.0, 0.0, 5.0) is True
    # a large NEGATIVE component must also be rejected — pins the abs() semantics
    assert dvl_velocity_rejected(-6.0, 0.0, 0.0, 5.0) is True
    # boundary: equal to max is NOT rejected (strict >)
    assert dvl_velocity_rejected(5.0, 0.0, 0.0, 5.0) is False
