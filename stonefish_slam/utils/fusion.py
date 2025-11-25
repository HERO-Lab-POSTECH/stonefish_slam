"""Sensor fusion utilities for map integration."""
import numpy as np


def ema_fusion(old_map, new_data, observation_count, alpha=0.3, threshold=0.0):
    """
    Exponential Moving Average (EMA) fusion for sensor data integration.

    Blends new observations with previous values to reduce noise while
    incorporating new information. Uses first-observation handling to
    avoid dark initialization artifacts.

    Args:
        old_map: Previous map intensities (1D flat array)
        new_data: New sensor intensities (1D array, same length as old_map)
        observation_count: Count array (1D, same length) tracking observations
        alpha: Fusion weight in [0, 1] range
               - 0.1: Strong noise reduction (slow update, ~10 frames effective)
               - 0.3: Balanced (default, ~3-4 frames effective)
               - 0.5: Fast update, less smoothing
        threshold: Minimum threshold for considering previous data (default 0.0)

    Returns:
        Fused intensities (1D array, same length as input)

    Examples:
        >>> old = np.array([0.0, 0.5, 1.0])
        >>> new = np.array([0.8, 0.6, 0.9])
        >>> count = np.array([0, 1, 3])
        >>> result = ema_fusion(old, new, count, alpha=0.3)
        >>> # First observation (count=0): result[0] = 0.8 (new value)
        >>> # Existing observation: result[1] = 0.3*0.6 + 0.7*0.5 = 0.53
    """
    # Identify first observations (no previous data above threshold)
    first_observation_mask = (old_map <= threshold)

    # EMA fusion formula: new = alpha * new_data + (1 - alpha) * old_data
    fused = np.where(
        first_observation_mask,
        new_data,  # First observation: use new value directly
        alpha * new_data + (1.0 - alpha) * old_map  # Blend with previous
    )

    return fused
