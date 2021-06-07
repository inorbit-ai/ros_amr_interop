import pytest
from pathlib import Path
from ros2mass.config import MassConfig
from ros2mass.config import CFG_PARAMETER_STATIC
from ros2mass.config import CFG_PARAMETER_ROS_TOPIC
from ros2mass.config import CFG_PARAMETER_ENVVAR

cwd = Path(__file__).resolve().parent


def test_mass_config_load():
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    assert MassConfig(str(cfg_file_path)).mapping != {}


@pytest.mark.parametrize("param_name, param_type", [
    ("uuid", CFG_PARAMETER_ENVVAR),
    ("robotModel", CFG_PARAMETER_STATIC),
    ("operationalState", CFG_PARAMETER_ROS_TOPIC),
    ("baseRobotEnvelope", CFG_PARAMETER_STATIC)
])
def test_mass_config_get_parameter_type(param_name, param_type):
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    mass_config = MassConfig(str(cfg_file_path))
    assert mass_config.get_parameter_type(param_name) == param_type


@pytest.mark.parametrize("param_name, value", [
    ("uuid", "foo"),
    ("robotModel", "spoony1.0"),
    ("operationalState", "/we_b_robots/mode"),
    ("baseRobotEnvelope", {'x': 2, 'y': 1, 'z': 3})
])
def test_mass_config_get_parameter_value(monkeypatch, param_name, value):
    monkeypatch.setenv("MY_UUID", "foo")  # Environment variable used on config file
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    mass_config = MassConfig(str(cfg_file_path))
    assert mass_config.get_parameter_value(param_name) == value
