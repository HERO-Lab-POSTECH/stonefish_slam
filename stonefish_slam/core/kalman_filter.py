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
