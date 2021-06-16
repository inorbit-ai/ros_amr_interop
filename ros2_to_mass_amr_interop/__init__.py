import websockets
import json
import asyncio
from time import sleep
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from functools import partial
from tf2_kdl import PyKDL

from std_msgs import msg as ros_std_msgs
from geometry_msgs import msg as ros_geometry_msgs
from sensor_msgs import msg as ros_sensor_msgs

from rclpy.node import Node
from .config import CFG_PARAMETER_LOCAL
from .config import CFG_PARAMETER_ENVVAR
from .config import CFG_PARAMETER_ROS_TOPIC
from .config import CFG_PARAMETER_ROS_PARAMETER  # noqa: F401
from .config import MassAMRInteropConfig

from .config import STATUS_REPORT_INTERVAL

from .messages import MASS_REPORT_UUID
from .messages import IdentityReport
from .messages import StatusReport


class MassAMRInteropNode(Node):
    """
    ROS node implementing WebSocket communication to Mass.

    The node configuration is obtained from a configuration file that can
    be provided externally. On initialization, the node subscribes to various
    topics and sends relevant data to a MassRobotics AMR InterOp standard
    capable server by using a WebSocket connection.

    Instances for both Identity and Status reports are kept internally as Node
    attributes and are updated when relevant data is processed.
    TODO: implement watchdog for node configuration file changes.

    Attributes
    ----------
        mass_identity_report (IdentityReport): Instance of Mass Identity Report
        mass_status_report (StatusReport): Instance of Mass Status Report

    """

    def __init__(self, **kwargs) -> None:
        super().__init__(node_name=self.__class__.__name__, **kwargs)
        # Get Node logger instance
        self.logger = self.get_logger()

        # Declare Node configuration parameter. Defaults to './config.yaml' if no
        # ``config_file`` parameter is provided. Provide the parameter when running
        # the node by using ``--ros-args -p config_file:=/path/to/config.yaml``
        self.declare_parameter('config_file', './config.yaml')
        config_file_param = self.get_parameter(name='config_file')
        config_file_path = config_file_param.get_parameter_value().string_value
        self._config = self._read_config_file(config_file_path=config_file_path)

        # ThreadPool for running other tasks
        self._ex = ThreadPoolExecutor()

        self.loop = asyncio.get_event_loop()
        # Create websocket connection
        self._uri = self._config.server
        self._wss_conn = None
        # perform a synchronous connect
        self.loop.run_until_complete(self._async_connect())

        # Create an instance of both report types
        _uuid = self._config.get_parameter_value(MASS_REPORT_UUID)
        self.mass_identity_report = IdentityReport(uuid=_uuid)
        self.mass_status_report = StatusReport(uuid=_uuid)

        self._process_config()

        self.loop.run_until_complete(self._async_send_report(self.mass_identity_report))

        self.loop.run_in_executor(self._ex, self._status_publisher_thread)

    def _status_publisher_thread(self):
        loop = asyncio.new_event_loop()
        self.logger.debug("Starting status publisher thread")
        def send_status():
            while True:
                self.loop.run_until_complete(self._async_send_report(self.mass_status_report))
                self.logger.debug(f"Status report sent. Waiting ...")
                sleep(STATUS_REPORT_INTERVAL)
        loop.create_task(send_status())

    def _process_config(self):
        """
        Update object state based on configuration.

        Populate Identity Report object with parameters from configuration
        and register callbacks for ROS Topics.

        TODO: re-run on configuration file changes
        TODO: deregister callbacks on configuration changes
        """
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

    async def _async_connect(self):
        self.logger.debug(f"Connecting to server '{self._uri}'")
        self._wss_conn = await websockets.connect(self._uri)
        self.logger.debug(f"Connected to Mass server '{self._uri}'")
        return self._wss_conn

    async def _async_send_report(self, mass_object):
        """
        Send MassRobotics object to MassRobotics server.

        Sends MassRobotics object data to MassRobotics server
        by using WebSockets connection.

        Args:
        ----
            mass_object (:obj:`MassObject`): Identity or Status report

        """
        self.logger.debug(f"Sending object ({type(mass_object)}): {mass_object.data}")
        try:
            await self._wss_conn.ensure_open()
        except (Exception, websockets.exceptions.ConnectionClosed, websockets.exceptions.ConnectionClosedError) as ex:
            self.logger.info(f"Reconnecting to server: {self._uri}")
            await self._async_connect()
        try:
            await self._wss_conn.send(json.dumps(mass_object.data))
        except Exception as ex:
            self.logger.info(f"Error while sending status report: {ex}")

    def _read_config_file(self, config_file_path):
        config_file_path = Path(config_file_path).resolve()
        if not config_file_path.is_file():
            raise ValueError(f"Configuration file '{config_file_path}' doesn't exist!")

        self.logger.info(f"Using configuration file '{config_file_path}'")
        return MassAMRInteropConfig(str(config_file_path))

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

            # ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.8, w: 1}}}" # noqa: E501

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

            # ros2 topic pub --once /good_sensors/bat sensor_msgs/msg/BatteryState "{percentage: 91.3}" # noqa: E501
            try:
                mass_data['batteryPercentage'] = getattr(data, msg_field)
            except AttributeError:
                self.logger.error(f"Message field '{msg_field}' on message of "
                                  f"type '{type(data)}' doesn't exist")

        if isinstance(data, ros_geometry_msgs.TwistStamped):
            self.logger.debug(f"Message {data} type `TwistStamped`")

            # ros2 topic pub --once /good_sensors/vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}}" # noqa: E501

            twist = data.twist

            linear_vel = PyKDL.Vector(
                x=twist.linear.x,
                y=twist.linear.y,
                z=twist.linear.z).Norm()
            quat = PyKDL.Rotation.EulerZYX(
                Alfa=twist.angular.z,
                Beta=twist.angular.y,
                Gamma=twist.angular.x).GetQuaternion()

            mass_data["velocity"] = {
                "linear": linear_vel,
                "angle": {
                    "x": quat[0],
                    "y": quat[1],
                    "z": quat[2],
                    "w": quat[3],
                },
                "planarDatum": "00000000-0000-0000-0000-000000000000"
            }

        # Reports are sent on each callback execution. This will be improved
        # later by using a corutine to publish messages on time intervals.
        if param_name in self.mass_identity_report.schema_properties:
            for mass_param_name, mass_param_data in mass_data.items():
                self.mass_identity_report.update_parameter(mass_param_name, mass_param_data)
            self.loop.run_until_complete(self._async_send_report(self.mass_identity_report))

        if param_name in self.mass_status_report.schema_properties:
            for mass_param_name, mass_param_data in mass_data.items():
                self.mass_status_report.update_parameter(mass_param_name, mass_param_data)
            self.loop.run_until_complete(self._async_send_report(self.mass_status_report))

    def register_mass_adapter(self, param_name, topic_name):
        """
        Register callbacks for parameters with source ROS topic.

        Creates node callbacks for ROS topics based on ROS topic parameters
        defined on the configuration file. The ``msgType`` configuration field
        is used determine Node callback message type. As the ``msgType`` is a
        string, this method tries to determine message type class as it is
        required to register the callback.

        Args
        ----
            param_name (str): configuration parameter name
            topic_name (str): topic name callback will be register

        Returns
        -------
            boolean: wheter callback registration was successful or not

        """
        self.logger.debug(f"Registering callback to topic '{topic_name}'")

        # Topic/message type is expected to contain package name e.g.
        # ``geometry_msgs/msg/Twist``.
        topic_type = self._config.get_ros_topic_parameter_type(name=param_name)
        topic_type_package = topic_type.split("/")[0]
        topic_type_name = topic_type.split("/")[-1]

        # In some cases, a single message of certain type may have fields that map
        # to multiple MassRobotics report fields. For this scenarios, an additional
        # ``msgField`` callback parameter can be provided, which makes the callback
        # to extract a single field from the message that is processed.
        # For example, a message of type ``sensors_msgs/msg/BatteryState`` contains
        # several fields but we are only interested in ``percentage`` as it translates
        # directly to MassRobotics status object field ``batteryPercentage``.
        #   batteryPercentage:
        #     valueFrom:
        #       rosTopic: /good_sensors/bat
        #       msgType: sensor_msgs/msg/BatteryState
        #       msgField: percentage
        msg_field = self._config.get_ros_topic_parameter_msg_field(name=param_name)

        # Map message package with corresponding module
        msgs_types = {
            'geometry_msgs': ros_geometry_msgs,
            'sensor_msgs': ros_sensor_msgs,
            'std_msgs': ros_std_msgs
        }

        # Try to determine callback message type class
        try:
            topic_type_t = getattr(msgs_types[topic_type_package], topic_type_name)
        except (AttributeError, KeyError):
            # If the message type is not supported do not register any callback
            self.logger.error(f"Undefined topic type '{topic_type}'. Ignoring...")
            return False

        self.logger.debug(f"Binding parameter '{param_name}' with topic '{topic_name}'")

        self.subscription = self.create_subscription(
            msg_type=topic_type_t,
            topic=topic_name,
            callback=partial(self._callback, param_name, msg_field),
            qos_profile=10)

        return True
