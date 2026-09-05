"""ssm.max_icp_move — ICP 결과가 시드에서 너무 멀면 시드로 되돌리는 게이트."""
import gtsam


def test_gate_returns_seed_only_beyond_threshold(load_localization):
    gate = load_localization.Localization.gate_icp_move
    seed, icp = gtsam.Pose2(1.0, 0.0, 0.0), gtsam.Pose2(0.5, 0.1, 0.0)

    t, move, rej = gate(seed, icp, 0.3)
    assert rej and t is seed and abs(move - (0.5 ** 2 + 0.1 ** 2) ** 0.5) < 1e-9

    t, _, rej = gate(seed, icp, 0.6)
    assert not rej and t is icp

    t, _, rej = gate(seed, icp, 0.0)          # 0 = off
    assert not rej and t is icp
