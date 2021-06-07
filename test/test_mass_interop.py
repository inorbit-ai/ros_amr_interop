from pathlib import Path
from ros2mass import IOMassInterOp

cwd = Path(__file__).resolve().parent


def test_mass_config_load():
    cfg_file_path = Path(cwd) / "test_data" / "config.yaml"
    IOMassInterOp(cfg_file_path)
