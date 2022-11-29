# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
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

import ssl
from uuid import uuid4
from paho.mqtt.client import MQTTMessage
from unittest.mock import MagicMock
import pytest
import json

from vda5050_connector_py.utils import get_vda5050_ts
from vda5050_connector_py.mqtt_bridge import generate_vda_order_msg
from vda5050_connector_py.mqtt_bridge import generate_vda_instant_action_msg
from vda5050_connector_py.mqtt_bridge import MQTTBridge
from vda5050_connector_py.utils import convert_ros_message_to_json

from vda5050_msgs.msg import Order
from vda5050_msgs.msg import Node
from vda5050_msgs.msg import Edge
from vda5050_msgs.msg import OrderState
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import Connection
from vda5050_msgs.msg import Visualization
from vda5050_msgs.msg import InstantActions


def get_order(order_id=str(uuid4())):
    return {
        "timestamp": get_vda5050_ts(),
        "version": "2.0.0",
        "manufacturer": "manufacturer_foo",
        "serial_number": "serial_number_123",
        "order_id": order_id,
        "order_update_id": 0,
        "zone_set_id": "zone_set_id_1",
        "nodes": [
            {
                "node_id": "node1",
                "sequence_id": 0,
                "released": True,
                "node_position": {"x": -2.0, "y": -0.5, "theta": 0.0, "map_id": "map"},
                "actions": [
                    {
                        "action_type": "pick",
                        "action_id": "UUID-1",
                        "blocking_type": "SOFT",
                    },
                    {
                        "action_type": "pick",
                        "action_id": "UUID-2",
                        "blocking_type": "SOFT",
                        "action_parameters": [
                            {"key": "k1", "value": "v1"},
                            {"key": "k2", "value": 1},
                            {"key": "k3", "value": 1.0},
                            {"key": "k4", "value": True},
                            {"key": "k5", "value": ["foo", "bar"]},
                        ],
                    },
                ],
            },
            {
                "node_id": "node2",
                "sequence_id": 2,
                "released": True,
                "node_position": {"x": 0.5, "y": 0, "theta": 0.0, "map_id": "map"},
                "actions": [
                    {
                        "action_type": "foo",
                        "action_id": "UUID-3",
                        "blocking_type": "HARD",
                    },
                    {
                        "action_type": "bar",
                        "action_id": "UUID-4",
                        "blocking_type": "NONE",
                    },
                ],
            },
            {
                "node_id": "node3",
                "sequence_id": 4,
                "released": True,
                "node_description": "Lorem ipsum",
                "node_position": {
                    "x": 1.5,
                    "y": 0.5,
                    "theta": 2.0,
                    "map_id": "map",
                    "allowed_deviation_x_y": 0.1,
                    "allowed_deviation_theta": -0.1,
                    "map_description": "Lorem ipsum",
                },
                "actions": [],
            },
            {
                "node_id": "node4",
                "sequence_id": 6,
                "released": True,
                "node_position": {"x": -2.0, "y": -0.5, "theta": 0.0, "map_id": "map"},
                "actions": [],
            },
        ],
        "edges": [
            {
                "edge_id": "edge1",
                "sequence_id": 1,
                "released": True,
                "start_node_id": "node1",
                "end_node_id": "node2",
                "actions": [
                    {
                        "action_type": "pick",
                        "action_id": "UUID-1",
                        "blocking_type": "SOFT",
                    },
                    {
                        "action_type": "pick",
                        "action_id": "UUID-2",
                        "blocking_type": "SOFT",
                        "action_parameters": [
                            {"key": "k1", "value": "v1"},
                            {"key": "k2", "value": 1},
                            {"key": "k3", "value": 1.0},
                            {"key": "k4", "value": True},
                            {"key": "k5", "value": ["foo", "bar"]},
                        ],
                    },
                ],
            },
            {
                "edge_id": "edge2",
                "sequence_id": 3,
                "edge_description": "Lorem ipsum",
                "released": True,
                "start_node_id": "node2",
                "end_node_id": "node3",
                "max_speed": 2,
                "max_height": 10,
                "min_height": 1,
                "orientation": 1.23,
                # TODO: test orientationType when vda5050_msgs package supports it
                # "orientation_type": "",
                "direction": "straight",
                "rotation_allowed": True,
                "max_rotation_speed": 0.789,
                "length": 12,
                "trajectory": {
                    "degree": 3,
                    "knot_vector": [0, 0.2, 0.4, 0.6, 0.8, 1],
                    "control_points": [
                        {"x": 1, "y": 2, "weight": 2},
                        {"x": 2, "y": 4, "weight": 1.5},
                        {"x": 2, "y": 4},
                    ],
                },
                "actions": [],
            },
            {
                "edge_id": "edge3",
                "sequence_id": 5,
                "released": True,
                "start_node_id": "node3",
                "end_node_id": "node4",
                "actions": [],
            },
        ],
    }


def get_instant_action():
    return {
        "timestamp": get_vda5050_ts(),
        "version": "2.0.0",
        "manufacturer": "manufacturer_foo",
        "serial_number": "serial_number_123",
        "actions": [
            {
                "action_type": "action_type_1",
                "action_id": "UUID-1",
                "blocking_type": "NONE",
            },
            {
                "action_type": "action_type_2",
                "action_id": "UUID-2",
                "action_description": "Lorem ipsum",
                "blocking_type": "HARD",
                "action_parameters": [
                    {"key": "k1", "value": "v1"},
                    {"key": "k2", "value": 1},
                    {"key": "k3", "value": 1.0},
                    {"key": "k4", "value": True},
                    {"key": "k5", "value": ["foo", "bar"]},
                ],
            },
        ],
    }


def test_vda5050_mqtt_bridge_generate_vda_order_msg(mocker):
    test_order_id = str(uuid4())
    order = get_order(order_id=test_order_id)
    order_vda_msg = generate_vda_order_msg(order)

    assert type(order_vda_msg) == dict
    assert order_vda_msg["order_id"] == test_order_id
    assert order_vda_msg["version"] == "2.0.0"
    assert order_vda_msg["manufacturer"] == "manufacturer_foo"
    assert order_vda_msg["serial_number"] == "serial_number_123"
    assert order_vda_msg["zone_set_id"] == "zone_set_id_1"

    # Validate nodes data
    nodes = order_vda_msg["nodes"]
    assert all([type(node) == Node for node in nodes]) and len(nodes) == 4

    # First order node has two actions. The second action contains
    # examples of all supported action parameters value types.
    node_1 = nodes[0]
    assert node_1.node_id == "node1"
    assert node_1.sequence_id == 0
    assert node_1.released is True
    assert node_1.node_position.x == -2.0
    assert node_1.node_position.y == -0.5
    assert node_1.node_position.theta == 0.0
    assert node_1.node_position.map_id == "map"
    action_1 = node_1.actions[0]
    assert action_1.action_type == "pick"
    assert action_1.action_id == "UUID-1"
    assert action_1.blocking_type == "SOFT"
    # First action has no ``action_parameters`` defined so it should default to an empty list
    assert action_1.action_parameters == []
    action_2 = node_1.actions[1]
    assert action_2.action_type == "pick"
    assert action_2.action_id == "UUID-2"
    assert action_2.blocking_type == "SOFT"
    action_params = action_2.action_parameters
    assert action_params[0].key == "k1" and action_params[0].value == "v1"
    assert action_params[1].key == "k2" and action_params[1].value == "1"
    assert action_params[2].key == "k3" and action_params[2].value == "1.0"
    assert action_params[3].key == "k4" and action_params[3].value == "True"
    assert action_params[4].key == "k5" and action_params[4].value == "['foo', 'bar']"

    node_2 = nodes[1]
    assert node_2.node_id == "node2"
    assert node_2.sequence_id == 2
    assert node_2.released is True
    assert node_2.node_position.x == 0.5
    assert node_2.node_position.y == 0.0  # checks proper float conversion
    assert node_2.node_position.theta == 0.0
    assert node_2.node_position.map_id == "map"
    action_1 = node_2.actions[0]
    assert action_1.action_type == "foo"
    assert action_1.action_id == "UUID-3"
    assert action_1.blocking_type == "HARD"
    assert action_1.action_parameters == []
    action_2 = node_2.actions[1]
    assert action_2.action_type == "bar"
    assert action_2.action_id == "UUID-4"
    assert action_2.blocking_type == "NONE"
    assert action_2.action_parameters == []

    node_3 = nodes[2]
    assert node_3.node_id == "node3"
    assert node_3.sequence_id == 4
    assert node_3.released is True
    assert node_3.node_description == "Lorem ipsum"
    assert node_3.actions == []
    node_position = node_3.node_position
    assert node_position.x == 1.5
    assert node_position.y == 0.5
    assert node_position.theta == 2.0
    assert node_position.map_id == "map"
    assert node_position.allowed_deviation_x_y == 0.1
    assert node_position.allowed_deviation_theta == -0.1
    assert node_position.map_description == "Lorem ipsum"

    node_4 = nodes[3]
    assert node_4.node_id == "node4"
    assert node_4.sequence_id == 6
    assert node_4.released is True
    assert node_4.actions == []

    # Validate edges data
    edges = order_vda_msg["edges"]
    assert all([type(edge) == Edge for edge in edges]) and len(edges) == 3

    edge_1 = edges[0]
    assert edge_1.edge_id == "edge1"
    assert edge_1.sequence_id == 1
    assert edge_1.released is True
    assert edge_1.start_node_id == "node1"
    assert edge_1.end_node_id == "node2"
    action_1 = edge_1.actions[0]
    assert action_1.action_type == "pick"
    assert action_1.action_id == "UUID-1"
    assert action_1.blocking_type == "SOFT"
    assert action_1.action_parameters == []
    action_2 = node_1.actions[1]
    assert action_2.action_type == "pick"
    assert action_2.action_id == "UUID-2"
    assert action_2.blocking_type == "SOFT"
    action_params = action_2.action_parameters
    assert action_params[0].key == "k1" and action_params[0].value == "v1"
    assert action_params[1].key == "k2" and action_params[1].value == "1"
    assert action_params[2].key == "k3" and action_params[2].value == "1.0"
    assert action_params[3].key == "k4" and action_params[3].value == "True"
    assert action_params[4].key == "k5" and action_params[4].value == "['foo', 'bar']"

    edge_2 = edges[1]
    assert edge_2.edge_id == "edge2"
    assert edge_2.sequence_id == 3
    assert edge_2.edge_description == "Lorem ipsum"
    assert edge_2.released is True
    assert edge_2.start_node_id == "node2"
    assert edge_2.end_node_id == "node3"
    assert edge_2.max_speed == 2.0
    assert edge_2.max_height == 10.0
    assert edge_2.min_height == 1.0
    assert edge_2.orientation == 1.23
    assert edge_2.direction == "straight"
    assert edge_2.rotation_allowed is True
    assert edge_2.max_rotation_speed == 0.789
    assert edge_2.length == 12
    assert edge_2.actions == []
    trajectory = edge_2.trajectory
    assert trajectory.degree == 3.0
    # The knot_vector uses a compact representation for arrays
    # See https://docs.python.org/3/library/array.html
    assert trajectory.knot_vector.tolist() == [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
    cpoints = trajectory.control_points
    assert cpoints[0].x == 1.0 and cpoints[0].y == 2 and cpoints[0].weight == 2.0
    assert cpoints[1].x == 2.0 and cpoints[1].y == 4 and cpoints[1].weight == 1.5
    assert cpoints[2].x == 2.0 and cpoints[2].y == 4 and cpoints[2].weight == 1.0

    edge_3 = edges[2]
    assert edge_3.edge_id == "edge3"
    assert edge_3.sequence_id == 5
    assert edge_3.released is True
    assert edge_3.start_node_id == "node3"
    assert edge_3.end_node_id == "node4"
    assert edge_3.actions == []


def test_vda5050_mqtt_bridge_generate_vda_instant_action_msg(mocker):
    instant_action = get_instant_action()
    instant_action_vda_msg = generate_vda_instant_action_msg(instant_action)

    assert type(instant_action_vda_msg) == dict
    assert instant_action_vda_msg["version"] == "2.0.0"
    assert instant_action_vda_msg["manufacturer"] == "manufacturer_foo"
    assert instant_action_vda_msg["serial_number"] == "serial_number_123"

    actions = instant_action_vda_msg["actions"]

    assert all([type(action) == Action for action in actions]) and len(actions) == 2

    actions[0].action_type == "action_type_1"
    actions[0].action_id == "UUID-1"
    actions[0].blocking_type == "NONE"

    actions[1].action_type == "action_type_2"
    actions[1].action_id == "UUID-2"
    actions[1].action_description == "Lorem ipsum"
    actions[1].blocking_type == "HARD"

    action_params = actions[1].action_parameters
    assert action_params[0].key == "k1" and action_params[0].value == "v1"
    assert action_params[1].key == "k2" and action_params[1].value == "1"
    assert action_params[2].key == "k3" and action_params[2].value == "1.0"
    assert action_params[3].key == "k4" and action_params[3].value == "True"
    assert action_params[4].key == "k5" and action_params[4].value == "['foo', 'bar']"


def test_vda5050_mqtt_bridge_generate_vda_instant_action_msg_on_v1_msg(mocker):
    instant_action = get_instant_action()

    # VDA5050 v1 instant action messages uses `instant_actions` field instead of `actions`
    instant_action["instant_actions"] = instant_action["actions"]
    del instant_action["actions"]

    instant_action_vda_msg = generate_vda_instant_action_msg(instant_action)

    assert type(instant_action_vda_msg) == dict

    actions = instant_action_vda_msg["actions"]

    assert all([type(action) == Action for action in actions]) and len(actions) == 2

    actions[0].action_type == "action_type_1"
    actions[0].action_id == "UUID-1"
    actions[0].blocking_type == "NONE"

    actions[1].action_type == "action_type_2"
    actions[1].action_id == "UUID-2"
    actions[1].action_description == "Lorem ipsum"
    actions[1].blocking_type == "HARD"

    action_params = actions[1].action_parameters
    assert action_params[0].key == "k1" and action_params[0].value == "v1"
    assert action_params[1].key == "k2" and action_params[1].value == "1"
    assert action_params[2].key == "k3" and action_params[2].value == "1.0"
    assert action_params[3].key == "k4" and action_params[3].value == "True"
    assert action_params[4].key == "k5" and action_params[4].value == "['foo', 'bar']"


def test_vda5050_mqtt_bridge_defaults(setup_rclpy, mocker, mock_mqtt_client):
    mqtt_bridge = MQTTBridge()
    assert mqtt_bridge._manufacturer_name == "robots"
    assert mqtt_bridge._serial_number == "robot_1"
    mqtt_client = mqtt_bridge.mqtt_client
    assert mqtt_client.tls_set.call_count == 0
    assert mqtt_client.username_pw_set.call_count == 0

    msg = Connection(
        timestamp="1970-01-01T12:00:00.00Z",
        version="2.0.0",
        manufacturer="robots",
        serial_number="robot_1",
        connection_state=Connection.CONNECTIONBROKEN,
    )
    will_payload = convert_ros_message_to_json(msg)

    mqtt_client.will_set.assert_called_with(
        topic="uagv/v1/robots/robot_1/connection",
        payload=will_payload,
        qos=1,
        retain=True,
    )
    mqtt_client.connect_async.assert_called_with(host="localhost", port=1883)


def test_vda5050_mqtt_bridge_defaults_with_tls(
    setup_rclpy, mocker, monkeypatch, mock_mqtt_client
):
    def mock_read_str_parameter(node, param_name, alternative):
        return {
            "mqtt_address": "fake_localhost",
            "mqtt_username": "username",
            "mqtt_password": "password",
            "manufacturer_name": "robots",
            "serial_number": "robot_1",
        }[param_name]

    mocker.patch(
        "vda5050_connector.mqtt_bridge.read_str_parameter",
        side_effect=mock_read_str_parameter,
    )
    mqtt_bridge = MQTTBridge()
    mqtt_client = mqtt_bridge.mqtt_client
    mqtt_client.tls_set.assert_called_with(
        ca_certs="/etc/ssl/certs/ca-certificates.crt", tls_version=ssl.PROTOCOL_TLSv1_2
    )
    mqtt_client.username_pw_set.assert_called_with(
        username="username", password="password"
    )
    mqtt_client.connect_async.assert_called_with(host="fake_localhost", port=1883)

    # Test TLS ca-cert path environment variable
    monkeypatch.setenv(
        name="VDA5050_CONNECTOR_TLS_CA_CERT", value="/foo/ca-certificates.crt"
    )
    mqtt_bridge = MQTTBridge()
    mqtt_bridge.mqtt_client.tls_set.assert_called_with(
        ca_certs="/foo/ca-certificates.crt", tls_version=ssl.PROTOCOL_TLSv1_2
    )


def test_vda5050_mqtt_bridge_subscriptions(setup_rclpy, mocker, mock_mqtt_client):
    # Test ROS2 subscribers and publishers
    mocker.patch.object(MQTTBridge, "create_subscription")
    mocker.patch.object(MQTTBridge, "create_publisher")
    mqtt_bridge = MQTTBridge()
    mqtt_bridge.create_subscription.assert_any_call(
        msg_type=OrderState,
        topic="/uagv/v1/robots/robot_1/state",
        callback=mqtt_bridge._publish_state,
        qos_profile=10,
    )
    mqtt_bridge.create_subscription.assert_any_call(
        msg_type=Connection,
        topic="/uagv/v1/robots/robot_1/connection",
        callback=mqtt_bridge._publish_connection,
        qos_profile=10,
    )
    mqtt_bridge.create_subscription.assert_any_call(
        msg_type=Visualization,
        topic="/uagv/v1/robots/robot_1/visualization",
        callback=mqtt_bridge._publish_visualization,
        qos_profile=10,
    )

    mqtt_bridge.create_publisher.assert_any_call(
        msg_type=Order, topic="/uagv/v1/robots/robot_1/order", qos_profile=10
    )
    mqtt_bridge.create_publisher.assert_any_call(
        msg_type=InstantActions,
        topic="/uagv/v1/robots/robot_1/instantActions",
        qos_profile=10,
    )

    # Test MQTT callbacks
    mqtt_client = mqtt_bridge.mqtt_client
    assert mqtt_client.on_connect == mqtt_bridge.on_connect_mqtt
    assert mqtt_client.on_message == mqtt_bridge.on_message_mqtt


@pytest.mark.parametrize(
    "malformed_msg",
    [
        (str.encode("")),
        (str.encode("{{}")),
        (str.encode("    ")),
    ],
)
def test_vda5050_mqtt_bridge_on_malformed_mqtt_messages(
    setup_rclpy, mocker, mock_mqtt_client, malformed_msg
):
    mqtt_bridge = MQTTBridge()
    mocker.patch.object(mqtt_bridge, "_order_pub", return_value=MagicMock())
    mocker.patch.object(mqtt_bridge, "_instant_actions_pub", return_value=MagicMock())

    mqtt_message = MQTTMessage(topic=str.encode("foo/bar/order"))
    mqtt_message.payload = malformed_msg
    mqtt_bridge.on_message_mqtt(mqtt_bridge.mqtt_client, ..., mqtt_message)
    mqtt_bridge._order_pub.publish.assert_not_called()
    mqtt_bridge._instant_actions_pub.publish.assert_not_called()

    mqtt_message = MQTTMessage(topic=str.encode("foo/bar/instantActions"))
    mqtt_message.payload = malformed_msg
    mqtt_bridge.on_message_mqtt(mqtt_bridge.mqtt_client, ..., mqtt_message)
    mqtt_bridge._order_pub.publish.assert_not_called()
    mqtt_bridge._instant_actions_pub.publish.assert_not_called()


@pytest.mark.parametrize(
    "invalid_msg",
    [
        ("{}"),
        ("{'foo': 'bar'}"),
        ("{'timestamp': '111'}"),
        ("{'timestamp': '111', 'header_id': 1, 'serial_number': '123'}"),
    ],
)
def test_vda5050_mqtt_bridge_on_invalid_vda5050_mqtt_messages(
    setup_rclpy, mocker, mock_mqtt_client, invalid_msg
):
    mqtt_bridge = MQTTBridge()
    mocker.patch.object(mqtt_bridge, "_order_pub", return_value=MagicMock())
    mocker.patch.object(mqtt_bridge, "_instant_actions_pub", return_value=MagicMock())

    mqtt_message = MQTTMessage(topic=str.encode("foo/bar/order"))
    mqtt_message.payload = str.encode("{}")
    mqtt_bridge.on_message_mqtt(mqtt_bridge.mqtt_client, ..., mqtt_message)
    mqtt_bridge._order_pub.publish.assert_not_called()
    mqtt_bridge._instant_actions_pub.publish.assert_not_called()

    mqtt_message = MQTTMessage(topic=str.encode("foo/bar/instantActions"))
    mqtt_message.payload = invalid_msg
    mqtt_bridge.on_message_mqtt(mqtt_bridge.mqtt_client, ..., mqtt_message)
    mqtt_bridge._order_pub.publish.assert_not_called()
    mqtt_bridge._instant_actions_pub.publish.assert_not_called()


@pytest.mark.parametrize(
    "valid_msg,msg_type",
    [
        (
            json.dumps(
                {
                    "timestamp": "1970-01-01T12:00:00.00Z",
                    "version": "v1",
                    "header_id": 1,
                    "serial_number": "123",
                    "manufacturer": "foo",
                    "nodes": [],
                    "edges": [],
                }
            ),
            "order",
        ),
        (
            json.dumps(
                {
                    "timestamp": "1970-01-01T12:00:00.00Z",
                    "version": "2.0.0",
                    "header_id": 1,
                    "serial_number": "123",
                    "manufacturer": "foo",
                    "nodes": [],
                    "edges": [],
                }
            ),
            "order",
        ),
        (
            json.dumps(
                {
                    "timestamp": "1970-01-01T12:00:00.00Z",
                    "version": "v2",
                    "header_id": 1,
                    "serial_number": "123",
                    "manufacturer": "foo",
                    "actions": [],
                }
            ),
            "instantAction",
        ),
        (
            json.dumps(
                {
                    "timestamp": "1970-01-01T12:00:00.00Z",
                    "version": "v1",
                    "header_id": 1,
                    "serial_number": "123",
                    "manufacturer": "foo",
                    "instantActions": [],
                }
            ),
            "instantAction",
        ),
    ],
)
def test_vda5050_mqtt_bridge_on_valid_vda5050_mqtt_messages(
    setup_rclpy, mocker, mock_mqtt_client, valid_msg, msg_type
):
    mqtt_bridge = MQTTBridge()
    mocker.patch.object(mqtt_bridge, "_order_pub", return_value=MagicMock())
    mocker.patch.object(mqtt_bridge, "_instant_actions_pub", return_value=MagicMock())

    if msg_type == "order":
        mqtt_message = MQTTMessage(topic=str.encode("foo/bar/order"))
        mqtt_message.payload = str.encode(valid_msg)
        mqtt_bridge.on_message_mqtt(mqtt_bridge.mqtt_client, ..., mqtt_message)
        mqtt_bridge._order_pub.publish.assert_called_once()
        mqtt_bridge._instant_actions_pub.publish.assert_not_called()
    elif msg_type == "instantAction":
        mqtt_message = MQTTMessage(topic=str.encode("foo/bar/instantActions"))
        mqtt_message.payload = valid_msg
        mqtt_bridge.on_message_mqtt(mqtt_bridge.mqtt_client, ..., mqtt_message)
        mqtt_bridge._order_pub.publish.assert_not_called()
        mqtt_bridge._instant_actions_pub.publish.assert_called_once()
    else:
        assert False
