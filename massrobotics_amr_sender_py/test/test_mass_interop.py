# Copyright 2021 InOrbit, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import pytest
import websockets
import asyncio
import rclpy
from rclpy import Parameter
from pathlib import Path
from unittest.mock import AsyncMock
from massrobotics_amr_sender import MassRoboticsAMRInteropNode

from std_msgs import msg as ros_std_msgs
from geometry_msgs import msg as ros_geometry_msgs
from sensor_msgs import msg as ros_sensor_msgs
from nav_msgs import msg as ros_nav_msgs
from builtin_interfaces import msg as ros_builtin_msgs

cwd = Path(__file__).resolve().parent
config_file_test = Path(cwd).parent / "sample_config.yaml"

FAKE_ROBOT_ID = 'd6f7c89c-6b11-45b4-b763-86cec88cc2eb'

# Mass Identity Report built after parsing
# the ``sample_config.yaml`` file
MASS_IDENTITY_REPORT = {
    'uuid': FAKE_ROBOT_ID,
    'manufacturerName': 'Spoonlift',
    'robotModel': 'spoony1.0',
    'robotSerialNumber': '2172837',
    'baseRobotEnvelope': {'x': 2, 'y': 1, 'z': 3},
    'maxSpeed': 2.5,
    'maxRunTime': 8,
    'emergencyContactInformation': '555-5555',
    'chargerType': '24V plus',
    'supportVendorName': 'We-B-Robots',
    'supportVendorContactInformation': 'support@we-b-robots.com',
    'productDocumentation': 'https://spoon.lift/support/docs/spoony1.0',
    'thumbnailImage': 'https://spoon.lift/media/spoony1.0.png',
    'cargoType': 'Anything solid or liquid',
    'cargoMaxVolume': {'x': 2, 'y': 2, 'z': 1},
    'cargoMaxWeight': '4000'
}


@pytest.fixture(autouse=True)
def mock_ws_conn(mocker):
    # mock websockets connect method
    websockets_mock = AsyncMock()
    # websockets.connect returns an instance of WebsocketClientProtocol
    # that is also mocked.
    websocket_client_protocol = AsyncMock()
    websocket_client_protocol.ensure_open = AsyncMock()
    websocket_client_protocol.send = AsyncMock()

    # return mocked WebsocketClientProtocol
    websockets_mock.return_value = websocket_client_protocol

    mocker.patch('websockets.connect', side_effect=websockets_mock)

    # On init, the Node creates a task on a separate thread for publishing
    # Mass status reports on a fixed time interval.
    # To avoid blocking (i.e. a method using an infinite loop), patch
    # the method so it does nothing. FIXME: this can be improved.
    def _fake_status_publisher_thread():
        pass

    mocker.patch('massrobotics_amr_sender.MassRoboticsAMRInteropNode._status_publisher_thread',
                 side_effect=_fake_status_publisher_thread)


@pytest.fixture(autouse=True)
def mock_robot_id(monkeypatch):
    # Environment variable used on config file
    monkeypatch.setenv("MY_UUID", FAKE_ROBOT_ID)


@pytest.fixture
def event_loop():
    # Fixture for running the async method for sending the Mass object
    loop = asyncio.get_event_loop()
    yield loop
    loop.close()


def test_mass_config_load_fails_on_missing_config_file(monkeypatch):
    monkeypatch.delenv("MY_UUID")
    rclpy.init()
    with pytest.raises(ValueError):
        MassRoboticsAMRInteropNode()
    rclpy.shutdown()


def test_massrobotics_amr_node_init():
    rclpy.init()
    node = MassRoboticsAMRInteropNode(parameter_overrides=[
        Parameter("config_file", value=str(config_file_test))
    ])
    rclpy.spin_once(node, timeout_sec=.1)
    rclpy.shutdown()

    mass_identity_report = node.mass_identity_report.data

    # check Node parses configuration file properly
    # and populates Mass Identity report object
    for prop, value in MASS_IDENTITY_REPORT.items():
        assert mass_identity_report[prop] == value

    # assert connect method has been called once
    assert websockets.connect.call_count == 1
    # assert mocked status published thread has been called once
    assert node._status_publisher_thread.call_count == 1
    # Mass identity report is sent once on Node init
    assert node._wss_conn.send.call_count == 1


# List of parameters for publishers that are used to
# invoke callbacks registered on Node init
# Parameters are
#   - msg_type: callback message type
#   - topic: topic where the message will be published
#   - msg: the message that will be published on the topic
#   - property: Mass Status report property that will be updated
#   - value: the value that should be written on the Mass Status report
STATUS_REPORT_TESTS = [
    {
        'msg_type': ros_std_msgs.String,
        'topic': '/we_b_robots/mode',
        'msg': ros_std_msgs.String(data='foo'),
        'property': 'operationalState',
        'value': 'foo'
    },
    {
        'msg_type': ros_geometry_msgs.PoseStamped,
        'topic': '/move_base_simple/goal',
        'msg': ros_geometry_msgs.PoseStamped(
            header=ros_std_msgs.Header(frame_id='floor1'),
            pose=ros_geometry_msgs.Pose(
                position=ros_geometry_msgs.Point(x=42.0, y=4.0, z=2.0),
                orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.1))),
        'property': 'location',
        'value': {
            'x': 42, 'y': 4, 'z': 2,
            'angle': {'w': 0.1, 'x': -1.0, 'y': 9.0, 'z': -3.0},
            'planarDatum': '096522ad-61fa-4796-9b31-e35b0f8d0b26'
        }
    },
    {
        'msg_type': ros_geometry_msgs.TwistStamped,
        'topic': '/good_sensors/vel',
        'msg': ros_geometry_msgs.TwistStamped(
            header=ros_std_msgs.Header(frame_id='floor2'),
            twist=ros_geometry_msgs.Twist(
                linear=ros_geometry_msgs.Vector3(x=1.0, y=0.0, z=0.0),
                angular=ros_geometry_msgs.Vector3(x=0.2, y=0.1, z=0.0))),
        'property': 'velocity',
        'value': {
            'linear': 1,
            'angle': {
                'w': 0.9937606691655042,
                'x': 0.09970865087213879,
                'y': 0.04972948160146044,
                'z': -0.0049895912294619805
            },
            'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
        }
    },
    {
        'msg_type': ros_sensor_msgs.BatteryState,
        'topic': '/good_sensors/bat',
        'msg': ros_sensor_msgs.BatteryState(percentage=12.34),
        'property': 'batteryPercentage',
        'value': pytest.approx(12.34)
    },
    {
        'msg_type': ros_std_msgs.Float32,
        'topic': '/good_sensors/bat_remaining',
        'msg': ros_std_msgs.Float32(data=123456.789),
        'property': 'remainingRunTime',
        'value': pytest.approx(123456.789)
    },
    {
        'msg_type': ros_std_msgs.Float32,
        'topic': '/good_sensors/load',
        'msg': ros_std_msgs.Float32(data=49.99),
        'property': 'loadPercentageStillAvailable',
        'value': pytest.approx(49.99)
    },
    {
        'msg_type': ros_nav_msgs.Path,
        'topic': '/we_b_robots/destinations',
        'msg': ros_nav_msgs.Path(
            header=ros_std_msgs.Header(frame_id='floor2'),
            poses=[
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor2', stamp=ros_builtin_msgs.Time(sec=1624401648)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=42.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.1))
                ),
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor2', stamp=ros_builtin_msgs.Time(sec=1624402598)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=4.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=1.0, z=-3.0, w=0.1))
                ),
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor2', stamp=ros_builtin_msgs.Time(sec=1624403168)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=12.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.4))
                ),
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor1', stamp=ros_builtin_msgs.Time(sec=1624404998)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=0.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.1))
                )
            ]
        ),
        'property': 'destinations',
        'value': [
            {
                'timestamp': '2021-06-22T22:40:48+00:00',
                'x': 42, 'y': 4, 'z': 2,
                'angle': {'w': 0.1, 'x': -1.0, 'y': 9.0, 'z': -3.0},
                'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
            },
            {
                'timestamp': '2021-06-22T22:56:38+00:00',
                'x': 4, 'y': 4, 'z': 2,
                'angle': {'w': 0.1, 'x': -1.0, 'y': 1.0, 'z': -3.0},
                'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
            },
            {
                'timestamp': '2021-06-22T23:06:08+00:00',
                'x': 12, 'y': 4, 'z': 2,
                'angle': {'w': 0.4, 'x': -1.0, 'y': 9.0, 'z': -3.0},
                'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
            },
            {
                'timestamp': '2021-06-22T23:36:38+00:00',
                'x': 0, 'y': 4, 'z': 2,
                'angle': {'w': 0.1, 'x': -1.0, 'y': 9.0, 'z': -3.0},
                'planarDatum': '096522ad-61fa-4796-9b31-e35b0f8d0b26'
            }
        ]
    },
    {
        'msg_type': ros_nav_msgs.Path,
        'topic': '/magic_nav/path',
        'msg': ros_nav_msgs.Path(
            header=ros_std_msgs.Header(frame_id='floor2'),
            poses=[
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor2', stamp=ros_builtin_msgs.Time(sec=1624401648)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=42.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.1))
                ),
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor2', stamp=ros_builtin_msgs.Time(sec=1624402598)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=4.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=1.0, z=-3.0, w=0.1))
                ),
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor2', stamp=ros_builtin_msgs.Time(sec=1624403168)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=12.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.4))
                ),
                ros_geometry_msgs.PoseStamped(
                    header=ros_std_msgs.Header(
                        frame_id='floor1', stamp=ros_builtin_msgs.Time(sec=1624404998)),
                    pose=ros_geometry_msgs.Pose(
                        position=ros_geometry_msgs.Point(x=0.0, y=4.0, z=2.0),
                        orientation=ros_geometry_msgs.Quaternion(x=-1.0, y=9.0, z=-3.0, w=0.1))
                )
            ]
        ),
        'property': 'path',
        'value': [
            {
                'timestamp': '2021-06-22T22:40:48+00:00',
                'x': 42, 'y': 4, 'z': 2,
                'angle': {'w': 0.1, 'x': -1.0, 'y': 9.0, 'z': -3.0},
                'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
            },
            {
                'timestamp': '2021-06-22T22:56:38+00:00',
                'x': 4, 'y': 4, 'z': 2,
                'angle': {'w': 0.1, 'x': -1.0, 'y': 1.0, 'z': -3.0},
                'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
            },
            {
                'timestamp': '2021-06-22T23:06:08+00:00',
                'x': 12, 'y': 4, 'z': 2,
                'angle': {'w': 0.4, 'x': -1.0, 'y': 9.0, 'z': -3.0},
                'planarDatum': '6ec7a6d0-21a9-4f04-b680-e7c640a0687e'
            },
            {
                'timestamp': '2021-06-22T23:36:38+00:00',
                'x': 0, 'y': 4, 'z': 2,
                'angle': {'w': 0.1, 'x': -1.0, 'y': 9.0, 'z': -3.0},
                'planarDatum': '096522ad-61fa-4796-9b31-e35b0f8d0b26'
            }
        ]
    },
    {
        'msg_type': ros_std_msgs.String,
        'topic': '/troubleshooting/errorcodes',
        'msg': ros_std_msgs.String(data='error1,error2,error3'),
        'property': 'errorCodes',
        'value': ['error1', 'error2', 'error3']
    },
    {
        'msg_type': ros_std_msgs.String,
        'topic': '/troubleshooting/errorcodes',
        'msg': ros_std_msgs.String(data='error1'),
        'property': 'errorCodes',
        'value': ['error1']
    },
    {
        'msg_type': ros_std_msgs.String,
        'topic': '/troubleshooting/errorcodes',
        'msg': ros_std_msgs.String(),
        'property': 'errorCodes',
        'value': []
    }
]


def test_massrobotics_amr_node_status_report_callbacks(event_loop):
    rclpy.init()
    # create the node we want to test
    node = MassRoboticsAMRInteropNode(parameter_overrides=[
        Parameter("config_file", value=str(config_file_test))
    ])
    # also create an additional node to publish messages
    helper_node = rclpy.create_node('test_helper_node')

    for test_data in STATUS_REPORT_TESTS:

        publisher = helper_node.create_publisher(
            msg_type=test_data['msg_type'],
            topic=test_data['topic'],
            qos_profile=10)
        publisher.publish(test_data['msg'])

        rclpy.spin_once(helper_node, timeout_sec=.1)
        rclpy.spin_once(node, timeout_sec=.1)

        publisher.destroy()

        result = node.mass_status_report.data[test_data['property']]
        expected = test_data['value']

        if result != expected:
            pytest.fail(f"The obtained result '{result}' doesn't match with the "
                        f"expected output '{expected}'. Test data: {test_data}")

    event_loop.run_until_complete(node._async_send_report(node.mass_status_report))
    rclpy.shutdown()

    # assert connect method has been called once
    assert websockets.connect.call_count == 1
    # assert mocked status published thread has been called once
    assert node._status_publisher_thread.call_count == 1
    # Mass identity report is sent once on Node init
    # and after processing all messages on ``STATUS_REPORT_TESTS``
    assert node._wss_conn.send.call_count == 2
