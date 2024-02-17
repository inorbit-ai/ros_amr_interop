#!/usr/bin/env python3

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

# Python dependencies
from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string
from paho.mqtt.enums import CallbackAPIVersion
import copy
import json
import ssl
import os

# ROS dependencies / utils
from rclpy.node import Node

from vda5050_connector_py.utils import get_vda5050_mqtt_topic
from vda5050_connector_py.utils import get_vda5050_ros2_topic
from vda5050_connector_py.utils import json_camel_to_snake_case
from vda5050_connector_py.utils import read_str_parameter, read_int_parameter
from vda5050_connector_py.utils import convert_ros_message_to_json
from vda5050_connector_py.utils import get_vda5050_ts

from vda5050_connector_py.vda5050_controller import DEFAULT_PROTOCOL_VERSION

# ROS msgs / srvs / actions
from vda5050_msgs.msg import Action as VDAAction
from vda5050_msgs.msg import ActionParameter as VDAActionParameter
from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import ControlPoint as VDAControlPoint
from vda5050_msgs.msg import Edge as VDAEdge
from vda5050_msgs.msg import InstantActions as VDAInstantActions
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodePosition as VDANodePosition
from vda5050_msgs.msg import Order as VDAOrder
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import Trajectory as VDATrajectory
from vda5050_msgs.msg import Visualization as VDAVisualization

NODE_NAME = "mqtt_bridge"


def generate_vda_order_msg(order):
    """
    Convert an Order message into a ROS2 Order message represented as a dict.

    Args:
    ----
        order (VDAOrder): VDA5050 Order message

    Returns
    -------
        Order dict message for building a ROS2 Order object

    """
    vda_order = copy.deepcopy(order)
    for node in vda_order["nodes"]:
        # Force all numbers to float. Values with no decimals are
        # interpret as integers, causing the validation errors
        for k in ["x", "y", "theta"]:
            node["node_position"][k] = float(node["node_position"][k])
        node["node_position"] = VDANodePosition(**node["node_position"])
        for action in node["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        node["actions"] = [VDAAction(**action) for action in node["actions"]]

    vda_order["nodes"] = [VDANode(**node) for node in vda_order["nodes"]]
    for edge in vda_order["edges"]:
        for action in edge["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        edge["actions"] = [VDAAction(**action) for action in edge["actions"]]

        # Force all numbers to float. Values with no decimals are
        # interpreted as integers, causing the validation errors.
        for k in [
            "max_speed",
            "max_height",
            "min_height",
            "orientation",
            "max_rotation_speed",
            "length",
        ]:
            if k in edge:
                edge[k] = float(edge[k])

        if "trajectory" in edge:
            edge["trajectory"] = VDATrajectory(
                degree=float(edge["trajectory"]["degree"]),
                knot_vector=edge["trajectory"]["knot_vector"],
                control_points=[
                    VDAControlPoint(
                        x=float(cp["x"]),
                        y=float(cp["y"]),
                        weight=float(cp.get("weight", 1)),
                    )
                    for cp in edge["trajectory"]["control_points"]
                ],
            )
    vda_order["edges"] = [VDAEdge(**edge) for edge in vda_order["edges"]]
    # TODO(@leandropineda): Consider returning a ROS2 Order message
    return vda_order


def generate_vda_instant_action_msg(instant_action):
    """
    Convert an Instant Action message into a ROS2 Instant Action message represented as a dict.

    Args:
    ----
        instant_action (VDAInstantActions): VDA5050 Instant Action message

    Returns
    -------
        Instant Action dict message for building a ROS2 Instant Action object

    """
    # Values on the `instant_action` parameter will be modified. Create
    # a copy of the object to avoid overrides
    vda_instant_action = copy.deepcopy(instant_action)

    # HACK: VDA5050 instantActions message schema differs from v1 to v2. In particular,
    # the ``instantActions`` field from v1 has been renamed to ``actions`` on v2.
    is_v1 = "instant_actions" in vda_instant_action.keys()

    instant_actions_field = "instant_actions" if is_v1 else "actions"

    for action in vda_instant_action[instant_actions_field]:
        try:
            # Force all action parameter values to string. VDA5050 supports
            # value types `array`, `boolean`, `number` and `string` but
            # VDAActionParameter expects str values
            for action_parameters in action["action_parameters"]:
                action_parameters["value"] = str(action_parameters["value"])
            action["action_parameters"] = [
                VDAActionParameter(**action_parameter)
                for action_parameter in action["action_parameters"]
            ]
        except KeyError:
            # Action parameters are not required
            pass

    vda_instant_action["actions"] = [
        VDAAction(**action) for action in vda_instant_action[instant_actions_field]
    ]

    # HACK: As the internal representation of all messages
    # is v2 only, remove the v1 ``instant_actions`` field
    if is_v1:
        vda_instant_action.pop("instant_actions")

    # TODO(@leandropineda): Consider returning a ROS2 Order message
    return vda_instant_action


class MQTTBridge(Node):
    """Translates VDA5050 MQTT messages from and to ROS2."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # Declare Node configuration parameter. Use default values if no parameters
        # are defined on launchfile. Provide the parameter when running the launchfile
        # by using ``foo.launch.py mqtt_address:=localhost mqtt_port:=1883 ...``
        mqtt_address = read_str_parameter(self, "mqtt_address", "localhost")
        mqtt_port = read_int_parameter(self, "mqtt_port", 1883)
        mqtt_username = read_str_parameter(self, "mqtt_username", "")
        mqtt_password = read_str_parameter(self, "mqtt_password", "")

        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")

        # Configure MQTT
        self.mqtt_client = mqtt_client.Client(CallbackAPIVersion.VERSION1)
        self.mqtt_client.on_connect = self.on_connect_mqtt
        self.mqtt_client.on_message = self.on_message_mqtt
        self.mqtt_client.on_disconnect = self.on_disconnect_mqtt

        # Enable TLS if username is provided
        if mqtt_username:
            self.mqtt_client.tls_set(
                ca_certs=os.getenv(
                    key="VDA5050_CONNECTOR_TLS_CA_CERT",
                    default="/etc/ssl/certs/ca-certificates.crt",
                ),
                tls_version=ssl.PROTOCOL_TLSv1_2,
            )
            self.mqtt_client.username_pw_set(
                username=mqtt_username, password=mqtt_password
            )

        # Configure will message or last testament message
        will_topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
        )

        # NOTE: will payload cannot be set dynamically or updated
        # without reconnecting, so some values are fixed. For the
        # timestamp, instead of using the time the connector started
        # the ts 0 is used which is easy to identify.
        will_payload = convert_ros_message_to_json(
            VDAConnection(
                header_id=0,
                version=DEFAULT_PROTOCOL_VERSION,
                timestamp="1970-01-01T12:00:00.00Z",
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                connection_state=VDAConnection.CONNECTIONBROKEN,
            )
        )
        self.mqtt_client.will_set(
            topic=will_topic, payload=will_payload, qos=1, retain=True
        )

        # Keep a copy of the last VDA5050 Connection message
        self._last_connection_msg = None

        # Connect to MQTT broker
        self.mqtt_client.connect_async(host=mqtt_address, port=int(mqtt_port))
        self.mqtt_client.loop_start()

        self.on_configure()

        self.logger.info(f"Node {NODE_NAME} has started successfully.")

    def on_connect_mqtt(self, client, userdata, flags, rc):
        """MQTT client connect callback."""
        if rc == 0:
            self.logger.info("Connected to MQTT Broker!")
            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="order",
                )
            )
            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="instantActions",
                )
            )
            self._publish_connection(
                msg=VDAConnection(
                    header_id=0,
                    version=DEFAULT_PROTOCOL_VERSION,
                    timestamp=get_vda5050_ts(),
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    connection_state=VDAConnection.ONLINE,
                )
            )

        else:
            self.logger.error("Failed to connect, return code %d\n", rc)

    def on_message_mqtt(self, client, userdata, msg):
        """MQTT client message callback."""
        try:
            msg_json = json_camel_to_snake_case(msg.payload)
            self.logger.debug(f"Received '{msg_json}' from '{msg.topic}' topic")
        except json.decoder.JSONDecodeError:
            self.logger.error(f"Failed to decode message: '{msg.payload}'")
            return

        try:
            if msg.topic.endswith("order"):
                vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
                self._order_pub.publish(msg=vda_order_msg)
            if msg.topic.endswith("instantActions"):
                vda_instant_actions_message = VDAInstantActions(
                    **generate_vda_instant_action_msg(msg_json)
                )
                self._instant_actions_pub.publish(msg=vda_instant_actions_message)
        except KeyError as ex:
            self.logger.warn(f"Ignoring invalid VDA5050 message: {ex}.")
            return

    def on_disconnect_mqtt(self, client, userdata, rc):
        """MQTT client disconnect callback."""
        if rc != 0:
            self.logger.info(
                f"MQTT client disconnected (rc: {rc}, {error_string(rc)}). Trying to reconnect."
            )
            while not self.mqtt_client.is_connected():
                try:
                    self.mqtt_client.reconnect()
                except OSError:
                    pass
        else:
            self.logger.info("Disconnected from MQTT Broker!")

    def on_configure(self):
        """
        Subscribe to relevant ROS2 topics.

        This method registers callbacks for translating
        ROS2 messages to VDA5050 MQTT messages.
        """
        self.logger.info("Configuring ROS topics")
        self._state_sub = self.create_subscription(
            msg_type=VDAOrderState,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="state",
            ),
            callback=self._publish_state,
            qos_profile=10,
        )

        self._connection_sub = self.create_subscription(
            msg_type=VDAConnection,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="connection",
            ),
            callback=self._publish_connection,
            qos_profile=10,
        )

        self._visualization_sub = self.create_subscription(
            msg_type=VDAVisualization,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="visualization",
            ),
            callback=self._publish_visualization,
            qos_profile=10,
        )

        self._order_pub = self.create_publisher(
            msg_type=VDAOrder,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
            ),
            qos_profile=10,
        )

        self._instant_actions_pub = self.create_publisher(
            msg_type=VDAInstantActions,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
            ),
            qos_profile=10,
        )
        self.logger.info("Finished configuring ROS topics")

    def on_shutdown(self):
        """Perform all necessary teardown steps."""
        self.logger.info("Publishing offline Connection message")

        offline_message = VDAConnection(
            header_id=0,
            version=DEFAULT_PROTOCOL_VERSION,
            timestamp=get_vda5050_ts(),
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            connection_state=VDAConnection.OFFLINE,
        )

        # Use the latest Connection message `header_id` if available
        if self._last_connection_msg:
            offline_message.header_id = self._last_connection_msg.header_id + 1

        self._publish_connection(msg=offline_message)

        self.logger.info("Unsubscribing from MQTT topics")
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
            )
        )
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
            )
        )

        self.mqtt_client.disconnect()

    def _publish_to_topic(self, msg, topic):
        """
        Publish a ROS2 message to an MQTT topic.

        Args:
        ----
            msg (Any): VDA5050 ROS2 message.
            topic (str): topic for publishing the VDA5050 MQTT message.

        """
        json_msg = convert_ros_message_to_json(msg)
        self.logger.debug(f"Publishing MQTT message to topic {topic}: {json_msg}")
        self.mqtt_client.publish(topic, json_msg)

    def _publish_state(self, msg: VDAOrderState):
        """
        Publish ROS2 OrderState message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAOrderState): VDA5050 ROS2 OrderState message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="state",
        )
        self._publish_to_topic(msg, topic)

    def _publish_connection(self, msg: VDAConnection):
        """
        Publish VDA5050 ROS2 Connection message to the corresponding VDA5050 MQTT topic.

        Also updates a local copy of the last published VDA5050 Connection message to keep track
        of the latest ``header_id`` field. This is used to publish an offline Connection message
        when tearing down the node.

        Args:
        ----
            msg (VDAConnection): VDA5050 ROS2 Connection message.

        """
        # Update the last connection message
        self._last_connection_msg = msg
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
        )
        self._publish_to_topic(msg, topic)

    def _publish_visualization(self, msg: VDAVisualization):
        """
        Publish ROS2 Visualization message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAVisualization): VDA5050 ROS2 Visualization message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="visualization",
        )
        self._publish_to_topic(msg, topic)
