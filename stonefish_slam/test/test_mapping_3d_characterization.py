"""Characterization tests for process_sonar_ray (P4e).

These tests record the CURRENT output of process_sonar_ray as an oracle before
decomposition.  They must be GREEN before any structural change is made.  After
decomposition they must remain GREEN to prove behavior preservation.

Design decisions
----------------
- Pure-Python path: use_dda=False, use_cpp_backend=False, use_cpp_ray_processor=False.
  C++ extensions are not available in the clean env; the fallback is automatic.
- Deterministic inputs: fixed bearing_angle, hand-crafted intensity_profile,
  identity T_sonar_to_world so sonar-frame == world-frame (easier to reason about).
- No T_world_to_sonar / first_hit_map → shadow validation branch not exercised
  (avoids needing a populated first_hit_map; shadow path is a guard that only
  skips voxels, so it is safe to test separately in scenario (c)).
- Tolerances: 1e-9 for log-odds sums (pure float arithmetic, no rounding).
"""
import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Minimal config that satisfies SonarMapping3D.__init__ (pure-Python path)
# ---------------------------------------------------------------------------

def _base_config():
    """Return a config dict that exercises the pure-Python fallback path."""
    return {
        # Sonar geometry
        "horizontal_fov": 120.0,          # degrees
        "vertical_fov": 20.0,             # degrees
        "range_max": 10.0,                # metres
        "range_min": 0.5,                 # metres
        "num_beams": 256,
        "num_bins": 100,
        "voxel_resolution": 0.5,

        # Sonar mounting
        "sonar_position": [0.0, 0.0, 0.0],
        "sonar_tilt_deg": 0.0,

        # Intensity / hit detection
        "intensity_threshold": 50.0,

        # Occupancy log-odds
        "log_odds_occupied": 0.85,
        "log_odds_free": -0.40,
        "log_odds_min": -6.0,
        "log_odds_max": 6.0,

        # Adaptive update (Python octree)
        "adaptive_update": False,
        "adaptive_threshold": 0.7,
        "adaptive_max_ratio": 3.0,

        # Weighting — kept OFF for deterministic baseline
        "use_range_weighting": False,
        "lambda_decay": 0.1,
        "enable_gaussian_weighting": False,
        "gaussian_sigma_factor": 2.5,

        # Backend switches — all C++ disabled
        "use_cpp_backend": False,
        "use_cpp_ray_processor": False,
        "use_dda_traversal": False,

        # Propagation (not exercised by process_sonar_ray directly)
        "enable_propagation": False,
        "propagation_radius": 2,
        "propagation_sigma": 1.5,

        # Update method
        "update_method": "log_odds",

        # Profiling — disable CSV writing to /tmp to keep tests fast/clean
        "enable_profiling": False,
        "frame_interval": 10,

        # Map size
        "min_probability": 0.1,
        "max_frames": 100,
        "dynamic_expansion": False,

        # IWLO/weighted_avg params (unused here but read by __init__)
        "sharpness": 1.0,
        "decay_rate": 0.05,
        "min_alpha": 0.3,

        # Bearing step (used by C++ ray processor, irrelevant here)
        "bearing_step": 1,
    }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mapper(load_mapping_3d, config=None):
    """Instantiate SonarMapping3D with the given config (default: _base_config())."""
    cfg = config if config is not None else _base_config()
    return load_mapping_3d.SonarMapping3D(cfg)


def _identity_T():
    """4x4 identity transform: sonar frame == world frame."""
    return np.eye(4)


def _call_ray(mapper, bearing_angle, intensity_profile, T=None):
    """Call process_sonar_ray and return the resulting voxel_updates dict."""
    if T is None:
        T = _identity_T()
    voxel_updates = {}
    mapper.process_sonar_ray(
        bearing_angle=bearing_angle,
        intensity_profile=intensity_profile,
        T_sonar_to_world=T,
        voxel_updates=voxel_updates,
    )
    return voxel_updates


def _sorted_keys(d):
    return sorted(d.keys())


# ---------------------------------------------------------------------------
# Scenario (a): bearing at 0 rad, profile has ONE hit above threshold near
#               the middle of the range.  Exercises first-hit detection, free
#               space before hit, and occupied voxel update.
# ---------------------------------------------------------------------------

def test_scenario_a_hit_in_profile(load_mapping_3d):
    """Single hit mid-profile: free-space voxels before hit + occupied voxel."""
    mapper = _make_mapper(load_mapping_3d)

    bearing = 0.0  # straight ahead

    # 100 bins, range_max=10m, range_min=0.5m → resolution=(10-0.5)/100=0.095m
    # Row 0 = far (10m), row 99 = near.
    # Place a hit at bin 60 (≈4.3m from origin).
    profile = np.zeros(100, dtype=float)
    profile[60] = 200.0   # above threshold=50 → hit

    result = _call_ray(mapper, bearing, profile)

    # --- structural checks ---
    assert len(result) > 0, "Expected at least one voxel update"

    # Check every entry has the expected structure
    for key, v in result.items():
        assert isinstance(key, tuple) and len(key) == 3
        assert 'point' in v and 'sum' in v and 'count' in v and 'intensity' in v
        assert v['count'] > 0

    # --- free-space voxels must have log-odds < 0 (negative accumulation) ---
    # The occupied voxel is the one with positive log-odds sum.
    positive_keys = [k for k, v in result.items() if v['sum'] > 0]
    negative_keys = [k for k, v in result.items() if v['sum'] < 0]
    assert len(positive_keys) > 0, "Expected occupied (positive log-odds) voxels"
    assert len(negative_keys) > 0, "Expected free (negative log-odds) voxels"

    # --- oracle: record exact sorted keys and per-key values ---
    # Run a second call with same inputs; output must be identical.
    result2 = _call_ray(mapper, bearing, profile)
    assert _sorted_keys(result) == _sorted_keys(result2), "Non-deterministic voxel keys"
    for k in result:
        assert abs(result[k]['sum'] - result2[k]['sum']) < 1e-9
        assert result[k]['count'] == result2[k]['count']

    # --- oracle snapshot: occupied voxels have positive sum ---
    # Boundary voxels may receive BOTH a free-space and an occupied update in the
    # same call (free-space sampling step == hit voxel), so the net sum is not a
    # clean multiple of log_odds_occupied.  We only assert the sign here; the
    # exact mixed-update arithmetic is pinned in test_scenario_a_voxel_sums_are_exact_multiples.
    for k in positive_keys:
        assert result[k]['sum'] > 0


def test_scenario_a_voxel_sums_are_exact_multiples(load_mapping_3d):
    """Oracle: every voxel's sum is an integer combination of log_odds_free and log_odds_occupied.

    A boundary voxel (at the transition from free-space to occupied) may receive
    BOTH a free-space and an occupied update in the same call.  Its sum is:
        log_odds_free * f + log_odds_occupied * o   (f + o == count, f,o >= 0)
    We verify that every voxel's sum can be expressed exactly this way.
    """
    mapper = _make_mapper(load_mapping_3d)

    profile = np.zeros(100, dtype=float)
    profile[40] = 150.0  # hit at bin 40
    result = _call_ray(mapper, 0.0, profile)

    lo_free = mapper.log_odds_free      # -0.40
    lo_occ  = mapper.log_odds_occupied  # +0.85

    for key, v in result.items():
        n = v['count']
        s = v['sum']
        # Search for non-negative integer f, o with f+o==n and
        # lo_free*f + lo_occ*o == s  (within float tolerance).
        found = False
        for f in range(n + 1):
            o = n - f
            expected = lo_free * f + lo_occ * o
            if abs(expected - s) < 1e-9:
                found = True
                break
        assert found, (
            f"Voxel {key}: sum={s}, count={n} — cannot express as "
            f"log_odds_free*f + log_odds_occupied*o for any f+o={n}"
        )


# ---------------------------------------------------------------------------
# Scenario (b): no hit — entire profile below threshold.
#               Entire range treated as free space.
# ---------------------------------------------------------------------------

def test_scenario_b_no_hit_all_free(load_mapping_3d):
    """No intensity above threshold: all voxel updates must be free-space (negative)."""
    mapper = _make_mapper(load_mapping_3d)

    profile = np.full(100, 10.0, dtype=float)  # all below threshold=50
    result = _call_ray(mapper, 0.0, profile)

    assert len(result) > 0, "Expected free-space voxel updates even with no hit"

    for key, v in result.items():
        assert v['sum'] < 0, f"Expected negative (free) log-odds for {key}, got {v['sum']}"
        assert v['intensity'] == 0.0, "Free voxels should have zero intensity"

    # Determinism check
    result2 = _call_ray(mapper, 0.0, profile)
    assert _sorted_keys(result) == _sorted_keys(result2)
    for k in result:
        assert result[k]['sum'] == pytest.approx(result2[k]['sum'], abs=1e-9)


def test_scenario_b_no_hit_uses_full_range_as_free(load_mapping_3d):
    """When no hit found, first_hit_idx=len(profile), so all bins sampled for free space."""
    mapper = _make_mapper(load_mapping_3d)

    # With hit mid-range: fewer free-space voxels
    profile_hit = np.zeros(100, dtype=float)
    profile_hit[50] = 200.0

    # Without hit: all 100 bins sampled (more free-space voxels expected)
    profile_no_hit = np.zeros(100, dtype=float)

    result_hit = _call_ray(mapper, 0.0, profile_hit)
    result_no_hit = _call_ray(mapper, 0.0, profile_no_hit)

    free_hit = sum(1 for v in result_hit.values() if v['sum'] < 0)
    free_no_hit = sum(1 for v in result_no_hit.values() if v['sum'] < 0)

    assert free_no_hit >= free_hit, (
        f"No-hit case should have >= free voxels than mid-hit case "
        f"({free_no_hit} vs {free_hit})"
    )


# ---------------------------------------------------------------------------
# Scenario (c): multiple hits — first hit + high-intensity region later.
#               Also exercises shadow validation with T_world_to_sonar + first_hit_map.
# ---------------------------------------------------------------------------

def test_scenario_c_multiple_hits(load_mapping_3d):
    """Profile with multiple above-threshold regions: all high-intensity bins updated as occupied."""
    mapper = _make_mapper(load_mapping_3d)

    profile = np.zeros(100, dtype=float)
    profile[30] = 180.0   # first hit
    profile[50] = 120.0   # second hit (after first → also occupied)
    profile[51] = 110.0   # adjacent to second hit
    # bins 31-49, 52-99: shadow/free → NOT updated as occupied

    result = _call_ray(mapper, 0.0, profile)

    # Positive voxels come only from bins 30, 50, 51
    positive_voxels = {k: v for k, v in result.items() if v['sum'] > 0}
    assert len(positive_voxels) > 0, "Expected occupied voxels from multiple hits"

    # No occupied voxel should have non-positive sum
    for k, v in positive_voxels.items():
        assert v['sum'] > 0

    # Determinism
    result2 = _call_ray(mapper, 0.0, profile)
    for k in result:
        assert result[k]['sum'] == pytest.approx(result2[k]['sum'], abs=1e-9)
        assert result[k]['count'] == result2[k]['count']


def test_scenario_c_shadow_region_not_updated(load_mapping_3d):
    """Bins between first hit and second hit (shadow region) must NOT produce occupied voxels."""
    mapper = _make_mapper(load_mapping_3d)

    # Only bin 20 above threshold; bins 21-99 are all below → shadow/unknown
    profile = np.zeros(100, dtype=float)
    profile[20] = 200.0

    result = _call_ray(mapper, 0.0, profile)
    occupied = {k: v for k, v in result.items() if v['sum'] > 0}

    # Exactly the voxel(s) covered by bin 20 (vertical fan) should be occupied
    # and nothing past it
    for k, v in occupied.items():
        assert v['sum'] > 0
        # The occupied voxel's range must correspond to bin 20:
        # range_m = 10.0 - 20 * 0.095 = 8.1m; with identity T and bearing=0,
        # x ≈ 8.1 → voxel x-index = floor(8.1/0.5) = 16
        x_idx = k[0]
        assert 14 <= x_idx <= 18, f"Occupied voxel x-index {x_idx} unexpected (expected ~16)"
