
import logging
from .config import MassConfig

logging.basicConfig(level=logging.DEBUG)


class MassAMRInterop:
    def __init__(self, config_file) -> None:
        self.logger = logging.getLogger(__class__.__name__)

        self._config = MassConfig(config_file)
        self._uri = self._config.server
        self.logger.debug(f"Mass server {self._uri}")
