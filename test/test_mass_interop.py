from pathlib import Path
from ros2_to_mass_amr_interop import MassAMRInterop

cwd = Path(__file__).resolve().parent


def test_mass_config_load():
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    MassAMRInterop(cfg_file_path)
