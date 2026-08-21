"""The polar->cartesian remap cache must not outlive its inputs.

`polar_to_cartesian_image` builds cv2.remap tables sized from the first
call's `polar_img.shape` and its `range_max`/`fov_deg`, then caches them
under `self.p2c_cache` with no key. A later call with a different shape or
range reuses those tables: the output keeps the first call's geometry and the
returned `range_resolution` no longer matches the map that produced the
pixels, so points are placed at wrong distances instead of failing loudly.

mapping_2d.py imports cv2 and tf2 helpers at module top, so the class is
exercised through a bound-method call on an object built with
`__new__` - only the handful of attributes the method reads are set.
"""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

REL = "stonefish_slam/core/mapping_2d.py"


@pytest.fixture
def mapper(load_module):
    """A SonarMapping2D carrying only what polar_to_cartesian_image reads."""
    module = load_module(REL, "mapping_2d_p2c")
    m = module.SonarMapping2D.__new__(module.SonarMapping2D)
    m.sonar_range = 15.0
    m.sonar_fov = 130.0
    m.range_min = 0.5
    m.p2c_cache = None
    return m


def _polar(rows, cols):
    """A polar frame with a single bright return, deterministic per shape."""
    img = np.zeros((rows, cols), dtype=np.uint8)
    img[rows // 2, cols // 2] = 255
    return img


def test_shape_change_rebuilds_the_maps(mapper):
    """A differently-shaped input must not be remapped with stale tables."""
    small, _ = mapper.polar_to_cartesian_image(_polar(100, 64))
    cached_small = mapper.p2c_cache["map_x"].shape

    large, _ = mapper.polar_to_cartesian_image(_polar(200, 128))
    cached_large = mapper.p2c_cache["map_x"].shape

    assert cached_large != cached_small, (
        "remap tables were reused for a different input shape"
    )
    assert large.shape[0] == 200, (
        f"cartesian height {large.shape[0]} still follows the first call's "
        f"{small.shape[0]} rows"
    )


def test_range_change_rebuilds_the_maps(mapper):
    """Changing range_max must re-derive the maps, not reuse the old scale."""
    polar = _polar(120, 64)
    _img_a, res_a = mapper.polar_to_cartesian_image(polar, range_max=15.0)
    map_x_a = mapper.p2c_cache["map_x"].copy()

    _img_b, res_b = mapper.polar_to_cartesian_image(polar, range_max=30.0)
    map_x_b = mapper.p2c_cache["map_x"]

    assert res_b != res_a, "range_resolution must track range_max"
    assert not np.array_equal(map_x_a, map_x_b), (
        "remap tables were reused after range_max changed, so pixels are "
        "placed using the old range scale"
    )


def test_fov_change_rebuilds_the_maps(mapper):
    """Changing the field of view must re-derive the bearing mapping."""
    polar = _polar(120, 64)
    mapper.polar_to_cartesian_image(polar, fov_deg=130.0)
    map_x_a = mapper.p2c_cache["map_x"].copy()

    mapper.polar_to_cartesian_image(polar, fov_deg=60.0)
    assert not np.array_equal(map_x_a, mapper.p2c_cache["map_x"]), (
        "remap tables were reused after fov_deg changed"
    )


def test_repeated_identical_calls_reuse_the_cache(mapper):
    """The fast path is preserved: same inputs must not rebuild."""
    polar = _polar(120, 64)
    mapper.polar_to_cartesian_image(polar)
    first = mapper.p2c_cache

    mapper.polar_to_cartesian_image(_polar(120, 64))
    assert mapper.p2c_cache is first, (
        "identical inputs rebuilt the maps; the cache no longer pays for itself"
    )
