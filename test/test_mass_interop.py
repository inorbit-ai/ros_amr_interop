from pathlib import Path
from ros2_to_mass_amr_interop import MassAMRInteropNode

cwd = Path(__file__).resolve().parent


def test_mass_config_load():
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    MassAMRInteropNode(cfg_file_path)
