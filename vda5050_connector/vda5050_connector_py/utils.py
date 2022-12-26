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

# Various common utility functions.

from datetime import datetime
import re
import json
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rosidl_runtime_py import message_to_ordereddict


def get_vda5050_ts():
    """
    Generate timestamp string using VDA5050 required format.

    The timestamp is in format ISO 8601 (UTC)
    YYYY-MM-DDTHH:mm:ss.ssZ (e.g.“2017-04-15T11:40:03.12Z”)

    Returns
    -------
        str: ISO 8601 UTC timestamp YYYY-MM-DDTHH:mm:ss.ssZ

    """
    d = datetime.utcnow()
    ts = d.isoformat()
    ts = ts[:-3]
    return f"{ts}Z"


def read_bool_parameter(node: Node, param_name: str, alternative: bool) -> bool:
    """Declare and read a bool parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == bool else param.get_parameter_value().bool_value


def read_str_parameter(node: Node, param_name: str, alternative: str) -> str:
    """Declare and read a string parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == str else param.get_parameter_value().string_value


def read_int_parameter(node: Node, param_name: str, alternative: int) -> int:
    """Declare and read a int parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == int else param.get_parameter_value().integer_value


def read_double_parameter(node: Node, param_name: str, alternative: float) -> float:
    """Declare and read a double (float) parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == float else param.get_parameter_value().double_value


def read_str_array_parameter(node: Node, param_name: str, alternative: list) -> list:
    """Declare and read a string array parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == list else param.get_parameter_value().string_array_value


def json_camel_to_snake_case(s):
    """
    Convert camel case JSON message to snake case.

    Converts all JSON message keys recursively from camel case to snake
    case e.g. camelCase to camel_case. Used to transform VDA5050 messages
    from MQTT topics to ROS2 vda5050_msgs

    Args:
    ----
        s (str|bytes): JSON message as string or bytes

    """
    def snake_case_dict(obj):
        """
        Replace dict camelCase keys by its snake case equivalent.

        Args
        ----
        obj (dict): dict object to replace keys

        Returns
        -------
        str: dictionary with all snake case keys

        """
        for key in obj.copy():
            new_key = re.sub(r"(?<!^)(?=[A-Z])", "_", key).lower()
            if new_key != key:
                obj[new_key] = obj[key]
                del obj[key]
        return obj

    return json.loads(s, object_hook=snake_case_dict)


def json_snake_to_camel_case(s):
    """
    Convert snake case JSON message to camel case.

    Converts all JSON message keys recursively from snake case to camel
    case e.g. snake_case to snakeCase. Used to transform ROS2 vda5050_msgs
    messages to VDA5050 MQTT messages.

    Args:
    ----
        s (str|bytes): JSON message as string or bytes

    """
    def to_camel_case(snake_str):
        """
        Convert snake case string to camel case.

        Args
        ----
            snake_str (str): snake case string.

        Returns
        -------
            str: camel case string.

        """
        components = snake_str.split("_")
        # Capitalize the first letter of each component except the first
        # one with the 'title' method and join them together.
        return components[0] + "".join(x.title() for x in components[1:])

    def camel_case_dict(obj):
        """
        Replace dict snake_case keys by its camel case equivalent.

        Args
        ----
            obj (dict): dict object to replace keys

        Returns
        -------
            str: dictionary with all camel case keys

        """
        for key in obj.copy():
            new_key = to_camel_case(key)
            if new_key != key:
                obj[new_key] = obj[key]
                del obj[key]
        return obj

    return json.loads(s, object_hook=camel_case_dict)


def convert_ros_message_to_json(msg):
    """
    Convert a ROS2 message into a JSON for MQTT publishing.

    Args:
    ----
        msg (Any): VDA5050 ROS2 message

    Returns
    -------
        dict: JSON representation of the ROS2 msg

    """
    json_msg = json.dumps(message_to_ordereddict(msg))
    return json.dumps(json_snake_to_camel_case(json_msg))


def get_vda5050_mqtt_topic(
    manufacturer, serial_number, topic, interface_name="uagv", major_version="v1"
):
    """
    Return suggested VDA5050 MQTT topics.

    Even though the topic structure is not strictly defined, the
    VDA5050 standard suggests topic levels as follows:

    ``interfaceName/majorVersion/manufacturer/serialNumber/topic``
    Example: uagv/v2/KIT/0001/order

    Note: Since the ``/`` character is used to define topic hierarchies, it
    must not be used in any of the aforementioned fields. The ``$`` character
    is also used in some MQTT brokers for special internal topics, so it should
    not be used either.

    Args
    ----
        manufacturer (string): Robot manufacturer
        serial_number (string): Unique robot serial number consisting of the following
            characters: ``A-Z a-z 0-9 _ - . :``
        topic (string): Subtopic for communication.
        interface_name (str, optional): Name of the used interface. Defaults to "uagv".
        major_version (str, optional): Major version number, preceded by "v". Defaults to "v1".

    Raises
    ------
        ValueError: on invalid ``topic``
        ValueError: on invalid ``major_version``

    Returns
    -------
        string: MQTT topic

    """
    if topic not in [
        "order",
        "state",
        "visualization",
        "instantActions",
        "connection",
        "factsheet",
    ]:
        raise ValueError(f"Invalid VDA5050 topic: {topic}")

    if len(major_version) != 2 or not major_version.startswith("v"):
        raise ValueError(
            "Invalid protocol major version. Expected an integer"
            f" preceded by 'v', but got {major_version}"
        )
    return f"{interface_name}/{major_version}/{manufacturer}/{serial_number}/{topic}"


def get_vda5050_ros2_topic(
    manufacturer, serial_number, topic, interface_name="uagv", major_version="v1"
):
    """
    Return ROS2 topics used for communication between controller and adapter.

    Note that these topics follow the same structure as the one defined on the
    VDA5050 standard, but preceded by ``/``.

    Example: /uagv/v2/KIT/0001/order


    Args
    ----
        manufacturer (string): Robot manufacturer
        serial_number (string): Unique robot serial number consisting of the following
            characters: ``A-Z a-z 0-9 _ - . :``
        topic (string): Subtopic for communication.
        interface_name (str, optional): Name of the used interface. Defaults to "uagv".
        major_version (str, optional): Major version number, preceded by "v". Defaults to "v1".

    Raises
    ------
        ValueError: on invalid ``topic``
        ValueError: on invalid ``major_version``

    Returns
    -------
        string: MQTT topic

    """
    mqtt_topic = get_vda5050_mqtt_topic(
        manufacturer, serial_number, topic, interface_name, major_version)
    return (
        f"/{mqtt_topic}"
    )
