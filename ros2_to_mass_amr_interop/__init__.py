import logging
import websockets
import json
import asyncio
from pathlib import Path
from rclpy.node import Node
from .config import MassAMRInteropConfig
from .messages import IdentityReport

logging.basicConfig(level=logging.DEBUG)


class MassAMRInteropNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__(node_name=self.__class__.__name__, **kwargs)
        self.logger = logging.getLogger(self.__class__.__name__)

        self.declare_parameter('config_file', './config.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        config_file = Path(config_file).resolve()
        if not config_file.is_file():
            raise ValueError(f"Configuration file '{config_file}' doesn't exist!")

        self.logger.info(f"Using configuration file '{config_file}'")
        self._config = MassAMRInteropConfig(str(config_file))

        self._uri = self._config.server
        self.logger.debug(f"Connecting to Mass server '{self._uri}'")
        self._wss_conn = websockets.connect(self._uri)
        self.logger.debug(f"Connection successful!")

    async def _send_identity_report(self):

        identity_report_data = {
            'uuid': self._config.get_parameter_value('uuid'),
            'manufacturer_name': self._config.get_parameter_value('manufacturerName'),
            'robot_model': self._config.get_parameter_value('robotModel'),
            'robot_serial_number': self._config.get_parameter_value('robotSerialNumber'),
            'base_robot_envelop': self._config.get_parameter_value('baseRobotEnvelope'),
        }

        identity_report = IdentityReport(**identity_report_data)

        async with self._wss_conn as websocket:
            await websocket.send(json.dumps(identity_report.data))

    def send_identity_report(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._send_identity_report())
