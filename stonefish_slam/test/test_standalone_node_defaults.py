"""Every node's sonar.* declare_parameter defaults must match sonar.yaml — hold them to it.

mapping_2d_standalone_node.py and mapping_3d_standalone_node.py carry the
comment "defaults match sonar.yaml" directly above their sonar.* declarations,
and both shipped range_max=15.0 / sonar_tilt_deg=10.0 against sonar.yaml's
40.0 / 30.0. Run without a parameter file — which is the whole point of a
standalone node — that is silently wrong geometry: a 2.7x range error and a
20 degree tilt error, with nothing in the logs to say so.

core/slam.py (the main SLAM node) and the two other standalone entry points
are held to the same yaml: on 2026-09-01 slam.py alone still declared
range_max=30.0 / sonar_tilt_deg=10.0, a third copy of the same drift.

Static AST rather than a runtime check: the nodes import rclpy at module top,
so they cannot be loaded in this environment (CONVENTIONS.md 2.8).
"""
import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SONAR_YAML = REPO_ROOT / "config" / "sonar.yaml"
NODES = (
    "core/slam.py",
    "nodes/feature_extraction_node.py",
    "nodes/fft_localization_node.py",
    "nodes/mapping_2d_standalone_node.py",
    "nodes/mapping_3d_standalone_node.py",
)


def _sonar_defaults(node_filename):
    """{'range_max': 40.0, ...} from declare_parameter('sonar.<key>', <literal>)."""
    tree = ast.parse((REPO_ROOT / "stonefish_slam" / node_filename).read_text())
    defaults = {}
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
            and node.args[0].value.startswith("sonar.")
        ):
            continue
        key = node.args[0].value[len("sonar."):]
        try:
            defaults[key] = ast.literal_eval(node.args[1])
        except ValueError:
            pass  # non-literal default: nothing to compare against
    return defaults


@pytest.fixture(scope="module")
def sonar_yaml():
    yaml = pytest.importorskip("yaml")
    with open(SONAR_YAML) as fh:
        return yaml.safe_load(fh)["slam_node"]["ros__parameters"]["sonar"]


@pytest.mark.parametrize("node_filename", NODES)
def test_sonar_defaults_match_the_yaml(node_filename, sonar_yaml):
    defaults = _sonar_defaults(node_filename)
    assert defaults, f"no sonar.* declare_parameter found in {node_filename}"

    mismatched = {
        key: (value, sonar_yaml[key])
        for key, value in defaults.items()
        if key in sonar_yaml and value != sonar_yaml[key]
    }
    assert not mismatched, (
        f"{node_filename} says 'defaults match sonar.yaml' but they do not "
        f"(key: (node, yaml)): {mismatched}"
    )
