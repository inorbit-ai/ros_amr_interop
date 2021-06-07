#!/usr/bin/env python

import asyncio
import websockets
import json
from .config import MassConfig
from .messages import IdentityReport
import os

# config_file_path = Path(__file__).resolve().parent / "config.yaml"
config_file_path = "/home/leandro/dev_ws/src/ros2mass/config.yaml"


async def sendMessage():
    os.environ['MY_UUID'] = "foobar"  # TODO: Remove
    mass_config = MassConfig(config_file_path)

    identity_report_data = {
        'uuid': mass_config.get_parameter_value('uuid'),
        'manufacturer_name': mass_config.get_parameter_value('manufacturerName'),
        'robot_model': mass_config.get_parameter_value('robotModel'),
        'robot_serial_number': mass_config.get_parameter_value('robotSerialNumber'),
        'base_robot_envelop': mass_config.get_parameter_value('baseRobotEnvelope'),
    }
    identity_report = IdentityReport(**identity_report_data)

    async with websockets.connect(mass_config.server) as websocket:

        # Send the identity message once
        await websocket.send(json.dumps(identity_report.data))


def main(args=None):
    asyncio.get_event_loop().run_until_complete(sendMessage())


if __name__ == '__main__':
    main()
