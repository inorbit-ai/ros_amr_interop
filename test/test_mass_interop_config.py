import pytest
from pathlib import Path
from ros2_to_mass_amr_interop.config import MassAMRInteropConfig
from ros2_to_mass_amr_interop.config import CFG_PARAMETER_LOCAL
from ros2_to_mass_amr_interop.config import CFG_PARAMETER_ROS_TOPIC
from ros2_to_mass_amr_interop.config import CFG_PARAMETER_ENVVAR

cwd = Path(__file__).resolve().parent


def test_mass_config_load():
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    assert MassAMRInteropConfig(str(cfg_file_path)).mappings != {}


@pytest.mark.parametrize("param_name, param_type", [
    ("uuid", CFG_PARAMETER_ENVVAR),
    ("robotModel", CFG_PARAMETER_LOCAL),
    ("operationalState", CFG_PARAMETER_ROS_TOPIC),
    ("baseRobotEnvelope", CFG_PARAMETER_LOCAL),
    ("maxSpeed", CFG_PARAMETER_LOCAL)
])
def test_mass_config_get_parameter_type(param_name, param_type):
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    mass_config = MassAMRInteropConfig(str(cfg_file_path))
    assert mass_config.get_parameter_source(param_name) == param_type


@pytest.mark.parametrize("param_name, value", [
    ("uuid", "foo"),
    ("robotModel", "spoony1.0"),
    ("operationalState", "/we_b_robots/mode"),
    ("baseRobotEnvelope", {'x': 2, 'y': 1, 'z': 3})
])
def test_mass_config_get_parameter_value(monkeypatch, param_name, value):
    monkeypatch.setenv("MY_UUID", "foo")  # Environment variable used on config file
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    mass_config = MassAMRInteropConfig(str(cfg_file_path))
    assert mass_config.get_parameter_value(param_name) == value


@pytest.mark.parametrize("param_name, source", [
    ("uuid", CFG_PARAMETER_ENVVAR),
    ("robotModel", CFG_PARAMETER_LOCAL),
    ("operationalState", CFG_PARAMETER_ROS_TOPIC),
    ("baseRobotEnvelope", CFG_PARAMETER_LOCAL),
    ("maxSpeed", CFG_PARAMETER_LOCAL)
])
def test_mass_config_get_parameters_by_source(monkeypatch, param_name, source):
    monkeypatch.setenv("MY_UUID", "foo")  # Environment variable used on config file
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    mass_config = MassAMRInteropConfig(str(cfg_file_path))
    assert param_name in mass_config.parameters_by_source[source]
