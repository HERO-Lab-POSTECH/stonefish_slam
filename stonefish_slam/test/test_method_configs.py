"""The three method_*.yaml files must share one parameter nesting.

launch/slam.launch.py picks config/mapping/method_{update_method}.yaml and
hands it to the node as-is, so every key must land at the depth the node
declares: `mapping_3d.<name>` (ROS 2 dot notation, i.e. exactly one level
below `mapping_3d:` in the YAML). A key nested any deeper is loaded into a
parameter name the node never declares and is silently dropped.
"""
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

METHOD_DIR = Path(__file__).resolve().parents[2] / "config" / "mapping"


def _mapping_3d_block(path):
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    return doc["slam_node"]["ros__parameters"]["mapping_3d"]


def _method_files():
    files = sorted(METHOD_DIR.glob("method_*.yaml"))
    assert files, f"no method_*.yaml found under {METHOD_DIR}"
    return files


@pytest.mark.parametrize("path", _method_files(), ids=lambda p: p.name)
def test_parameters_sit_directly_under_mapping_3d(path):
    """No method file may nest its parameters in a sub-dict."""
    block = _mapping_3d_block(path)
    nested = {k: v for k, v in block.items() if isinstance(v, dict)}
    assert not nested, (
        f"{path.name}: keys {sorted(nested)} are nested below mapping_3d, so "
        "they load as parameter names the node never declares and are ignored"
    )


def test_weighted_avg_exposes_its_intensity_threshold():
    """The one key weighted_avg shares with the code must be reachable."""
    block = _mapping_3d_block(METHOD_DIR / "method_weighted_avg.yaml")
    assert "intensity_threshold" in block
    assert isinstance(block["intensity_threshold"], int)
