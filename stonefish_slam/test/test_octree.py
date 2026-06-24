"""Characterization tests for the pure HierarchicalOctree (numpy-only).

octree.py imports only numpy, so it loads directly via the load_module
fixture without rclpy/gtsam. Expected values are mathematical ground truth
for a standard log-odds occupancy octree (OctoMap; Hornung et al. 2013):
    voxel key:   key = floor(coord / resolution)
    occupancy:   L_new = L_old + L_measurement  (additive)
    probability: p = 1 / (1 + exp(-L))          (sigmoid)
Hand-computed and confirmed against the module. A mismatch means a code
bug, not a test bug (see CONVENTIONS.md §2.8).
"""
import numpy as np

REL = "stonefish_slam/core/octree.py"


def _tree(load_module, name, **kw):
    Tree = load_module(REL, name).HierarchicalOctree
    return Tree(**kw)


def test_world_to_key_floor_quantization(load_module):
    t = _tree(load_module, "oct_key", resolution=0.1)
    # key = floor(coord / resolution), per voxel cell
    assert t.world_to_key(0.05, 0.15, -0.05) == (0, 1, -1)
    assert t.world_to_key(1.0, 2.0, 3.0) == (10, 20, 30)


def test_world_to_key_negative_floors_down(load_module):
    t = _tree(load_module, "oct_key2", resolution=0.5)
    # floor(-0.1/0.5) = floor(-0.2) = -1 (floor, not truncation toward zero)
    assert t.world_to_key(-0.1, -0.5, -0.6) == (-1, -1, -2)


def test_log_odds_accumulates_additively(load_module):
    t = _tree(load_module, "oct_acc", resolution=0.1)
    p = [0.5, 0.5, 0.5]
    # non-adaptive occupied update of +1.5 (the module's log_odds_occupied)
    t.update_voxel(p, 1.5, adaptive=False)
    assert np.isclose(t.query_voxel(p), 1.5)
    t.update_voxel(p, 1.5, adaptive=False)
    assert np.isclose(t.query_voxel(p), 3.0)  # additive: 1.5 + 1.5


def test_unobserved_voxel_is_zero_log_odds(load_module):
    t = _tree(load_module, "oct_zero", resolution=0.1)
    # a voxel never updated has the flat prior L = 0 (p = 0.5)
    assert np.isclose(t.query_voxel([7.0, 7.0, 7.0]), 0.0)


def test_adaptive_update_scales_down_on_free_voxel(load_module):
    t = _tree(load_module, "oct_adapt", resolution=0.1)
    # First occupied hit on a prior-free voxel (current_prob = 0.5 = threshold):
    # update_scale = (0.5 / 0.5) * 0.5 = 0.5, so applied = 1.5 * 0.5 = 0.75.
    t.update_voxel([1.0, 1.0, 1.0], 1.5, adaptive=True)
    assert np.isclose(t.query_voxel([1.0, 1.0, 1.0]), 0.75)


def test_adaptive_update_honors_config_max_ratio(load_module):
    # Coverage gap (P3): the prior test pins the class-default adaptive_max_ratio (0.5),
    # but deployment injects 0.3 via config (config/mapping/method_log_odds.yaml:13,
    # method_iwlo.yaml:25). Pin the config path: with adaptive_max_ratio = 0.3,
    # update_scale = (0.5 / 0.5) * 0.3 = 0.3, so applied = 1.5 * 0.3 = 0.45.
    t = _tree(load_module, "oct_adapt_cfg", resolution=0.1)
    t.adaptive_max_ratio = 0.3
    t.update_voxel([1.0, 1.0, 1.0], 1.5, adaptive=True)
    assert np.isclose(t.query_voxel([1.0, 1.0, 1.0]), 0.45)


def _leaf_for(t, point):
    """Descend from the root to the leaf node containing `point`.

    Uses only public OctNode API (children / get_octant_index / is_leaf) so it
    works under the numpy-only load_module fixture. Returns the OctNode whose
    .size is the actual leaf voxel edge length.
    """
    p = np.array(point[:3], dtype=np.float64)
    node = t.root
    while not node.is_leaf():
        node = node.children[node.get_octant_index(p)]
    return node


def test_leaf_size_equals_resolution(load_module):
    # The constructor contract (octree.py:94 "resolution: ... leaf node size")
    # and the OctoMap standard (Hornung 2013) require the leaf voxel edge to
    # equal `resolution`. auto-size = 2**max_depth * resolution, halved each
    # level, reaches exactly `resolution` at depth == max_depth. A leaf larger
    # than `resolution` means recursion stopped one (or more) levels too early.
    t = _tree(load_module, "oct_leaf", resolution=0.1, max_depth=9)
    t.update_voxel([1.0, 1.0, 1.0], 1.5, adaptive=False)
    leaf = _leaf_for(t, [1.0, 1.0, 1.0])
    assert np.isclose(leaf.size, 0.1), f"leaf size {leaf.size} != resolution 0.1"


def test_leaf_grid_matches_world_to_key_resolution(load_module):
    # world_to_key quantizes by floor(coord / resolution) — cells of size
    # `resolution`. The hierarchical leaf grid must agree with that cell size,
    # else the two voxel grids in the same map disagree.
    t = _tree(load_module, "oct_leaf_grid", resolution=0.2, max_depth=8)
    t.update_voxel([2.0, 2.0, 2.0], 1.5, adaptive=False)
    leaf = _leaf_for(t, [2.0, 2.0, 2.0])
    assert np.isclose(leaf.size, 0.2), f"leaf size {leaf.size} != resolution 0.2"
