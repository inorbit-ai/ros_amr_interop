import pytest
import rclpy
from rclpy import Parameter
from pathlib import Path
from ros2_to_mass_amr_interop import MassAMRInteropNode

cwd = Path(__file__).resolve().parent
config_file_test = cwd / 'test_data' / 'config.yaml'


def test_mass_config_load():
    rclpy.init()
    MassAMRInteropNode(parameter_overrides=[
        Parameter("config_file", value=str(config_file_test))
    ])
    rclpy.shutdown()


def test_mass_config_load_fails_on_missing_config_file():
    rclpy.init()
    with pytest.raises(ValueError):
        MassAMRInteropNode()
    rclpy.shutdown()
