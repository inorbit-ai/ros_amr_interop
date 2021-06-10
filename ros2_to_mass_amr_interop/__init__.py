import websockets
import json
import asyncio
from std_msgs import msg as ros_std_msgs

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

    def _get_topic_type(self, topic_name):
        """
        Return topic's type.

        Tries to determine topic's type by analyzing all the subscribers
        registered into that topic. It assumes that all topic endpoint
        types are the same so no additional checking is done.

        Args
        ----
            topic_name (str): topic's name.

        Raises
        ------
            RuntimeError: when topic type cannot be determined.

        Returns
        -------
            str: topic type e.g. `std_msgs/msg/String`.

        """
        # TODO: verify if this approach makes sense vs specifying the topic msgs
        # type on the Node configuration. I'm assuming only one TopicEndpointInfo
        # is returned, but it may not be the case. An idea for testing this would
        # be to subscribe two or more nodes to the same topic and verify how many
        # results `get_publishers_info_by_topic` returns. If two or more are
        # returned, there are two possible implementations:
        #  1) currently used, assume all `topic_type`s are the same and pick the first
        #  2) verify that `topic_type`s of all TopicEndpointInfo objects are the same

        self.logger.debug(f"Getting topic type for topic '{topic_name}'")
        topic_subs_info = self.get_publishers_info_by_topic(topic_name)
        self.logger.debug(f"Got topic subscription info: {topic_subs_info}")

        if not len(topic_subs_info):
            raise RuntimeError(f"No publishers found for topic '{topic_name}'")

        return topic_subs_info[0].topic_type

    def _callback(self, param_name, data):
        # Callback method receives an object of certain type depending on the topic
        # message type this callback is registed. For example, if the callback is
        # called for messages of type `String` of topic `/foo/bar`, then `data` is
        # type `String`.

        # TODO: this may grow a lot, so it may be a good idea to implement an adapter

        if param_name in self.mass_identity_report.schema_properties:
            # TODO: Implement ROS2 data type adapter e.g. sensor_msgs/BatteryState to
            # Mass `batteryPercentage`, `remainingRunTime` and `loadPercentageStillAvailable`
            self.mass_identity_report.update_parameter(param_name, data.data)
            self.send_identity_report()
        if param_name in self.mass_status_report.schema_properties:
            # TODO: Implement ROS2 data type adapter e.g. sensor_msgs/BatteryState to
            # Mass `batteryPercentage`, `remainingRunTime` and `loadPercentageStillAvailable`
            self.mass_status_report.update_parameter(param_name, data.data)
            self.send_status_report()
        self.logger.info(f"Parameter '{param_name}', type {type(data)}")

    def register_mass_adapter(self, param_name, topic_name):

        self.logger.debug(f"Registering callback to topic '{topic_name}'")
        try:
            topic_type = self._get_topic_type(topic_name=topic_name)
        except RuntimeError as ex:
            self.logger.error(f"Couldn't determine topic '{topic_name}' type: {ex}")
            return False

        # topic_type_module = topic_type.split("/")[0]
        topic_type_name = topic_type.split("/")[-1]

        # TODO: support message types other than std_msgs
        # e.g. geometry_msgs. These have a different name structure
        # and come from a different module `geometry_msgs`
        try:
            topic_type_t = getattr(ros_std_msgs, topic_type_name)
        except AttributeError:
            self.logger.error(f"Undefined topic type {topic_type}")
            return False

        self.logger.debug(f"Binding parameter '{param_name}' with topic '{topic_name}'")

        self.subscription = self.create_subscription(
            msg_type=topic_type_t,
            topic=topic_name,
            callback=partial(self._callback, param_name),
            qos_profile=10)

        return True
