"""Pure Kalman filter math, independent of ROS and gtsam.

These are the standard discrete linear Kalman filter equations (Welch &
Bishop, "An Introduction to the Kalman Filter", TR 95-041). They are kept
free of any rclpy/gtsam imports so they can be unit-tested directly without
launching a ROS node — the ROS layer (`KalmanNode`) is a thin wrapper that
calls into these functions. See docs/CONVENTIONS.md §2.0 for the ROS2
core-vs-node separation rationale.
"""
import numpy as np


def kalman_predict(
    previous_x: np.ndarray,
    previous_P: np.ndarray,
    A: np.ndarray,
    Q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Propagate the state and the error covariance ahead (time update).

    Args:
        previous_x (np.ndarray): previous state vector, shape (n, 1).
        previous_P (np.ndarray): previous covariance matrix, shape (n, n).
        A (np.ndarray): state transition matrix, shape (n, n).
        Q (np.ndarray): process noise covariance, shape (n, n).

    Returns:
        tuple[np.ndarray, np.ndarray]: predicted state ``x- = A x`` and
        predicted covariance ``P- = A P A^T + Q``.
    """
    A = np.array(A)
    predicted_x = A @ previous_x
    predicted_P = A @ previous_P @ A.T + Q
    return predicted_x, predicted_P


def kalman_correct(
    predicted_x: np.ndarray,
    predicted_P: np.ndarray,
    z: np.ndarray,
    H: np.ndarray,
    R: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply a measurement update (correction step).

    Args:
        predicted_x (np.ndarray): a priori state from :func:`kalman_predict`.
        predicted_P (np.ndarray): a priori covariance from :func:`kalman_predict`.
        z (np.ndarray): measurement vector.
        H (np.ndarray): observation matrix.
        R (np.ndarray): measurement noise covariance.

    Returns:
        tuple[np.ndarray, np.ndarray]: corrected state
        ``x = x- + K (z - H x-)`` and corrected covariance
        ``P = (I - K H) P-``, where ``K = P- H^T (H P- H^T + R)^-1``.
    """
    K = predicted_P @ H.T @ np.linalg.inv(H @ predicted_P @ H.T + R)
    corrected_x = predicted_x + K @ (z - H @ predicted_x)
    corrected_P = predicted_P - K @ H @ predicted_P
    return corrected_x, corrected_P


def pressure_to_depth(fluid_pressure: float, offset_z: float) -> float:
    """Convert an absolute fluid pressure reading to a Z-up depth coordinate.

    .. warning::
        This reproduces the EXISTING formula verbatim and is **physically
        incorrect**: ROS ``sensor_msgs/FluidPressure.fluid_pressure`` is in
        Pascals, but the divisor ``101.325`` is 1 atm expressed in *kilo*pascals
        — a 1000x unit mismatch. The Stonefish publisher emits raw Pascals
        (``ROS2Interface.cpp``; ``environment.scn`` sets 101300 Pa), so at the
        surface this returns ~-9990 m rather than ~0. The ``* 10`` factor also
        approximates seawater's 10.08 m/atm as 10. Kept as-is to preserve
        behavior during extraction; the fix is tracked in P4_FLAGS.md.

    Args:
        fluid_pressure (float): pressure reading from the FluidPressure message.
        offset_z (float): additive datum offset subtracted after conversion.

    Returns:
        float: the (Z-up, negative-down) depth value the node feeds into the
        Kalman correction.
    """
    curr_depth = -((fluid_pressure / 101.325) - 1) * 10
    curr_depth -= offset_z
    return curr_depth


def dvl_velocity_rejected(
    velocity_x: float,
    velocity_y: float,
    velocity_z: float,
    max_velocity: float,
) -> bool:
    """Decide whether a DVL velocity measurement should be rejected.

    Mirrors the early-return gate in the original ``dvl_callback``: a
    measurement is rejected when any component's magnitude exceeds
    ``max_velocity`` (strict ``>``), so a value equal to the limit is accepted.

    Args:
        velocity_x (float): DVL velocity x component.
        velocity_y (float): DVL velocity y component.
        velocity_z (float): DVL velocity z component.
        max_velocity (float): rejection threshold.

    Returns:
        bool: ``True`` if the measurement should be discarded, else ``False``.
    """
    measurement = np.array([[velocity_x], [velocity_y], [velocity_z]])
    return bool(np.any(np.abs(measurement) > max_velocity))
