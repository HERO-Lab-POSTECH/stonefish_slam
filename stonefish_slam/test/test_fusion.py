import numpy as np

REL = "stonefish_slam/utils/fusion.py"


def _ema(load_module):
    return load_module(REL, "fusion_under_test").ema_fusion


def test_first_observation_uses_new_value(load_module):
    ema = _ema(load_module)
    result = ema(np.array([0.0, 0.5, 1.0]), np.array([0.8, 0.6, 0.9]),
                 np.array([0, 1, 3]), alpha=0.3)
    assert np.isclose(result[0], 0.8)  # old[0]=0.0<=threshold → new 사용 (실측)


def test_existing_observation_is_ema(load_module):
    ema = _ema(load_module)
    result = ema(np.array([0.0, 0.5, 1.0]), np.array([0.8, 0.6, 0.9]),
                 np.array([0, 1, 3]), alpha=0.3)
    assert np.isclose(result[1], 0.53)  # 0.3*0.6+0.7*0.5 (실측)
    assert np.isclose(result[2], 0.97)  # 0.3*0.9+0.7*1.0 (실측)
