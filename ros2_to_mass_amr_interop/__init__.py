import websockets
import json
import asyncio
from std_msgs import msg as ros_std_msgs
from geometry_msgs import msg as ros_geometry_msgs
from sensor_msgs import msg as ros_sensor_msgs
from tf2_kdl import PyKDL

from pathlib import Path
from functools import partial

from rclpy.node import Node
from .config import CFG_PARAMETER_LOCAL
from .config import CFG_PARAMETER_ENVVAR
from .config import CFG_PARAMETER_ROS_TOPIC
from .config import CFG_PARAMETER_ROS_PARAMETER  # noqa: F401
from .config import MassAMRInteropConfig

from .messages import MASS_REPORT_UUID
from .messages import IdentityReport
from .messages import StatusReport


class MassAMRInteropNode(Node):
    """
    ROS node implementing WebSocket communication to Mass.

    The node configuration is obtained from a configuration file
    that can be provided externally. Then it subscribes to various
    topics and sends relevant data to a Mass server by using a
    WebSocket connection.

    Attributes
    ----------
        mass_identity_report (IdentityReport): Instance of Mass Identity Report
        mass_status_report (StatusReport): Instance of Mass Status Report

    """

    def __init__(self, **kwargs) -> None:
        super().__init__(node_name=self.__class__.__name__, **kwargs)
        # Get Node logger instance
        self.logger = self.get_logger()

        # Declare Node configuration parameter. Defaults to './config.yaml' if
        # no `config_file` parameter is provided.
        self.declare_parameter('config_file', './config.yaml')
        config_file_param = self.get_parameter(name='config_file')
        config_file_path = config_file_param.get_parameter_value().string_value
        self._config = self._read_config_file(config_file_path=config_file_path)

        # Create websocket connection
        self._uri = self._config.server
        self._wss_conn = websockets.connect(self._uri)
        self.logger.debug(f"Connected to Mass server '{self._uri}'")

        # An instance of both report types is kept internally
        _uuid = self._config.get_parameter_value(MASS_REPORT_UUID)
        self.mass_identity_report = IdentityReport(uuid=_uuid)
        self.mass_status_report = StatusReport(uuid=_uuid)

        # Update status report with local parameters
        for param_name in self._config.parameters_by_source[CFG_PARAMETER_LOCAL]:
            param_value = self._config.get_parameter_value(param_name)
            self.mass_identity_report.update_parameter(name=param_name, value=param_value)

        for param_name in self._config.parameters_by_source[CFG_PARAMETER_ENVVAR]:
            param_value = self._config.get_parameter_value(param_name)
            self.mass_identity_report.update_parameter(name=param_name, value=param_value)

        # Register callbacks for rosTopic parameters
        for param_name in self._config.parameters_by_source[CFG_PARAMETER_ROS_TOPIC]:
            topic_name = self._config.get_parameter_value(name=param_name)
            self.register_mass_adapter(param_name, topic_name)

    def _read_config_file(self, config_file_path):

        config_file_path = Path(config_file_path).resolve()
        if not config_file_path.is_file():
            raise ValueError(f"Configuration file '{config_file_path}' doesn't exist!")

        self.logger.info(f"Using configuration file '{config_file_path}'")
        return MassAMRInteropConfig(str(config_file_path))

    # <May be a good idea to move these outside this class>
    async def _send_report(self, mass_object):
        async with self._wss_conn as websocket:
            await websocket.send(json.dumps(mass_object.data))

    def send_identity_report(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._send_report(self.mass_identity_report))

    def send_status_report(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._send_report(self.mass_status_report))
    # </May be a good idea to move these outside this class>

    def _callback(self, param_name, msg_field, data):
        # Callback method receives an object of certain type depending on the topic
        # message type this callback is registed. For example, if the callback is
        # called for messages of type `String` of topic `/foo/bar`, then `data` is
        # type `String`.

        # TODO: this may grow a lot, so it may be a good idea to implement an adapter

        mass_data = {}

        self.logger.info(f"Parameter '{param_name}', type {type(data)}")

        if isinstance(data, ros_geometry_msgs.PoseStamped):
            self.logger.debug(f"Message {data} type `PoseStamped`")
            pose_position = data.pose.position  # Point
            pose_orientation = data.pose.orientation  # Quaternion
            mass_data["location"] = {
                "x": pose_position.x,
                "y": pose_position.y,
                "z": pose_position.z,
                "angle": {
                    "x": pose_orientation.x,
                    "y": pose_orientation.y,
                    "z": pose_orientation.z,
                    "w": pose_orientation.w
                },
                "planarDatum": "00000000-0000-0000-0000-000000000000"
            }

        if isinstance(data, ros_sensor_msgs.BatteryState):
            self.logger.debug(f"Message {data} type `BatteryState`")

            # ros2 topic pub --once /good_sensors/bat sensor_msgs/msg/BatteryState "{percentage: 91.3}"
            try:
                mass_data['batteryPercentage'] = getattr(data, msg_field)
            except AttributeError:
                self.logger.error(f"Message field '{msg_field}' on message type '{type(data)}' doesn't exist")

        if isinstance(data, ros_geometry_msgs.TwistStamped):
            self.logger.debug(f"Message {data} type `TwistStamped`")

            # ros2 topic pub --once /good_sensors/vel geometry_msgs/msg/TwistStamped 
            # TODO: find why command below doesn't work. It seems
            # that auto headers are not supported on ros2
            # ros2 topic pub --once /good_sensors/vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'value'}, twist.linear: {x: 1, y: 2, z: 3}, twist.angular: {x: 1, y: 1, z: 1}}"
            twist = data.twist
            quat = PyKDL.Rotation.EulerZYX(twist.angular.z, twist.angular.y, twist.angular.x).GetQuaternion()
            mass_data["velocity"] = {
                "linear": 1,  # TODO: calculate linear velocity
                "angle": {
                    "x": quat[0],
                    "y": quat[1],
                    "z": quat[2],
                    "w": quat[3],
                },
                "planarDatum": "00000000-0000-0000-0000-000000000000"
            }

        if param_name in self.mass_identity_report.schema_properties:
            for mass_param_name, mass_param_data in mass_data.items():
                self.mass_identity_report.update_parameter(mass_param_name, mass_param_data)
                self.send_identity_report()
        if param_name in self.mass_status_report.schema_properties:
            for mass_param_name, mass_param_data in mass_data.items():
                self.mass_status_report.update_parameter(mass_param_name, mass_param_data)
                self.send_status_report()

    def register_mass_adapter(self, param_name, topic_name):

        self.logger.debug(f"Registering callback to topic '{topic_name}'")

        topic_type = self._config.get_ros_topic_parameter_type(name=param_name)

        topic_type_module = topic_type.split("/")[0]  # Are they actually modules? What does geometry_msgs in `geometry_msgs/msg/PoseStamped` means?
        topic_type_name = topic_type.split("/")[-1]

        msg_field = self._config.get_ros_topic_parameter_msg_field(name=param_name)

        # TODO: support message types other than std_msgs
        # e.g. geometry_msgs. These have a different name structure
        # and come from a different module `geometry_msgs`
        msgs_types = {
            'geometry_msgs': ros_geometry_msgs,
            'sensor_msgs': ros_sensor_msgs,
            'std_msgs': ros_std_msgs
        }

        try:
            topic_type_t = getattr(msgs_types[topic_type_module], topic_type_name)
        except (AttributeError, KeyError):
            self.logger.error(f"Undefined topic type {topic_type}")
            return False

        self.logger.debug(f"Binding parameter '{param_name}' with topic '{topic_name}'")

        self.subscription = self.create_subscription(
            msg_type=topic_type_t,
            topic=topic_name,
            callback=partial(self._callback, param_name, msg_field),
            qos_profile=10)

        return True
