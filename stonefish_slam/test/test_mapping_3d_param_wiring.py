"""The main slam_node must actually forward the 3D update-method parameters.

`config/mapping.yaml:34` sets `update_method: 'iwlo'` and
`launch/slam.launch.py:45` loads `config/mapping/method_{update_method}.yaml`
accordingly, but `SonarMapping3D` reads the method from the config DICT that
`core/slam.py` builds (`config.get('update_method', 'log_odds')` at
mapping_3d.py:161). If slam.py never puts the key there, every main-pipeline
run silently falls back to log_odds and the whole IWLO branch - including
`set_iwlo_params` and the intensity weighting - is unreachable, no matter what
the YAML says. The standalone node (`nodes/mapping_3d_standalone_node.py`)
does forward it, which is why the method appeared to work when tested there.

slam.py imports rclpy at module top so it cannot be loaded in this
environment (CONVENTIONS.md §2.8). This gate is therefore static AST: it
checks that each parameter SonarMapping3D looks for is both declared and
placed into the config dict, and that the 3D mapper is not fed the 2D
intensity threshold.
"""
import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SLAM_PY = REPO_ROOT / "stonefish_slam" / "core" / "slam.py"

# Keys SonarMapping3D reads for the non-default update methods
# (mapping_3d.py:161 update_method, :199-205 set_iwlo_params, :251-255
# ray_config IWLO fields, :116 intensity_threshold).
METHOD_KEYS = ("update_method", "sharpness", "decay_rate", "min_alpha")


def _slam_tree():
    return ast.parse(SLAM_PY.read_text())


def _declared_parameters(tree):
    """Every name passed to self.declare_parameter('<name>', ...)."""
    names = set()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            names.add(node.args[0].value)
    return names


def _mapping_3d_config_dict(tree):
    """The dict literal assigned to `mapping_3d_config`, key -> value node."""
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "mapping_3d_config":
                    return {
                        k.value: v
                        for k, v in zip(node.value.keys, node.value.values)
                        if isinstance(k, ast.Constant)
                    }
    pytest.fail("mapping_3d_config dict literal not found in core/slam.py")


def _get_parameter_name(value_node):
    """For `self.get_parameter('x').value`, return 'x'; else None."""
    for node in ast.walk(value_node):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "get_parameter"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            return node.args[0].value
    return None


@pytest.mark.parametrize("key", METHOD_KEYS)
def test_method_parameter_is_declared(key):
    """A parameter the launch file supplies must be declared, or ROS drops it."""
    declared = _declared_parameters(_slam_tree())
    assert f"mapping_3d.{key}" in declared, (
        f"core/slam.py never declares mapping_3d.{key}; the value in "
        f"config/mapping/method_*.yaml cannot reach the node"
    )


@pytest.mark.parametrize("key", METHOD_KEYS)
def test_method_parameter_reaches_the_mapper(key):
    """The declared parameter must also be put into mapping_3d_config."""
    config = _mapping_3d_config_dict(_slam_tree())
    assert key in config, (
        f"'{key}' is missing from mapping_3d_config, so SonarMapping3D falls "
        f"back to its default and the configured update method is ignored"
    )
    assert _get_parameter_name(config[key]) == f"mapping_3d.{key}", (
        f"mapping_3d_config['{key}'] does not read mapping_3d.{key}"
    )


def test_launch_override_uses_the_declared_parameter_name():
    """slam.launch.py's runtime override must name mapping_3d.update_method.

    A bare top-level 'update_method' key is not a parameter the node declares,
    so the launch argument would be dropped and the method could never be
    overridden from the command line.
    """
    launch_py = REPO_ROOT / "launch" / "slam.launch.py"
    tree = ast.parse(launch_py.read_text())
    keys = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "param_dict":
                    keys = {
                        k.value for k in node.value.keys if isinstance(k, ast.Constant)
                    }
    assert keys, "param_dict literal not found in launch/slam.launch.py"
    assert "mapping_3d.update_method" in keys
    assert "update_method" not in keys, (
        "bare 'update_method' is not declared by the node and is silently dropped"
    )


def test_launch_default_method_comes_from_mapping_yaml():
    """The launch default must not hardcode a method that contradicts the config."""
    yaml = pytest.importorskip("yaml")
    mapping_yaml = REPO_ROOT / "config" / "mapping.yaml"
    with open(mapping_yaml) as fh:
        configured = (
            yaml.safe_load(fh)["slam_node"]["ros__parameters"]["mapping_3d"]
        )["update_method"]

    source = (REPO_ROOT / "launch" / "slam.launch.py").read_text()
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "update_method"
        ):
            for kw in node.keywords:
                if kw.arg == "default_value":
                    assert not isinstance(kw.value, ast.Constant), (
                        f"update_method default is hardcoded; config/mapping.yaml "
                        f"selects {configured!r} and would be overridden"
                    )
                    return
    pytest.fail("update_method DeclareLaunchArgument not found")


def test_3d_mapper_gets_the_3d_intensity_threshold():
    """The 3D config must not be fed the 2D grid's intensity threshold."""
    config = _mapping_3d_config_dict(_slam_tree())
    assert "intensity_threshold" in config
    source = _get_parameter_name(config["intensity_threshold"])
    assert source == "mapping_3d.intensity_threshold", (
        f"mapping_3d_config['intensity_threshold'] reads {source!r}; the 3D "
        "mapper must use mapping_3d.intensity_threshold, not the 2D one"
    )
