from collections import defaultdict
import yaml
import logging
import os

# Config files may have non local values that are
# ought to be extracted from a different source
# other than the config file
CFG_PARAMETER_LOCAL = "local"
CFG_PARAMETER_ROS_TOPIC = "rosTopic"
CFG_PARAMETER_ROS_PARAMETER = "rosParameter"
CFG_PARAMETER_ENVVAR = "envVar"

SUPPORTED_EXTERNAL_VALUES = [
    CFG_PARAMETER_ROS_TOPIC,
    CFG_PARAMETER_ROS_PARAMETER,
    CFG_PARAMETER_ENVVAR
]

STATUS_REPORT_INTERVAL = 1


class MassAMRInteropConfig:
    """
    Configuration file parsing and value gathering.

    Parses yaml configuration file and deals with parameters
    with values that are not local i.e. parameter values that
    are obtained from environment variables or ROS2 topics.

    Attributes
    ----------
        server (str): Mass WebSocket server URI
        mappings (:obj:`dict`): parameter name and value mapping

    """

    def __init__(self, path=None) -> None:
        self.logger = logging.getLogger(__class__.__name__)
        _config = self._load(path)

        self.server = _config['server']
        self.mappings = _config['mappings']
        self.parameters_by_source = defaultdict(list)
        self._parse_config(self.mappings)

    def _load(self, path) -> None:
        config = dict()
        with open(path, "r") as fd:
            try:
                config = yaml.safe_load(fd)
            except yaml.YAMLError as ex:
                self.logger.error("Failed to parse YAML config file", ex)

        self.logger.debug(f"Config file '{path}' loaded")

        # Ignoring config file root key value as it's not relevant
        k = next(iter(config))
        config = config[k]
        return config

    def _parse_config(self, mappings):
        for parameter_name in mappings.keys():
            parameter_source = self.get_parameter_source(name=parameter_name)
            self.parameters_by_source[parameter_source].append(parameter_name)

    def get_parameter_source(self, name):
        """
        Return parameter source.

        Args:
        ----
            name (str): parameter name

        Raises
        ------
            ValueError: when parameter does not exist or it is invalid

        Returns
        -------
            str: parameter source

        """
        if isinstance(self.mappings[name], str) or \
                isinstance(self.mappings[name], float) or \
                isinstance(self.mappings[name], int):
            return CFG_PARAMETER_LOCAL

        if isinstance(self.mappings[name], dict):
            # Evaluate is the parameter is non local
            if 'valueFrom' in self.mappings[name]:
                param_source = self.mappings[name]['valueFrom']

                # param_source is a dict whose first key
                # is the external parameter type
                if not isinstance(param_source, dict):
                    raise ValueError(f"Invalid 'valueFrom' configuration: '{name}'")
                param_source = next(iter(param_source))

                if param_source in SUPPORTED_EXTERNAL_VALUES:
                    return param_source

        # If no supported external valueFrom configs were found,
        # assume that the parameter is local if it is an object
        if isinstance(self.mappings[name], dict):
            return CFG_PARAMETER_LOCAL

        raise ValueError(f"Couldn't determine parameter '{name}' source.")

    def get_parameter_value(self, name):
        """
        Return configuration parameter value.

        It also deals with the complexity of getting
        the value from different sources.

        Args:
        ----
            name (Union[str, dict]): configuration parameter value

        Raises
        ------
            ValueError: when parameter value does not exist or it is invalid

        Returns
        -------
            str: parameter value

        """
        param_source = self.get_parameter_source(name)
        self.logger.debug(f"Parameter '{name}' source: {param_source}")

        if param_source == CFG_PARAMETER_LOCAL:
            return self.mappings[name]

        if param_source == CFG_PARAMETER_ENVVAR:
            envvar_name = self.mappings[name]['valueFrom'][param_source]
            param_value = os.getenv(envvar_name)
            if not param_value:
                raise ValueError(f"Empty or undefined environment variable: '{envvar_name}'")
            return param_value

        if param_source == CFG_PARAMETER_ROS_TOPIC:
            return self.mappings[name]['valueFrom'][param_source]

        if param_source == CFG_PARAMETER_ROS_PARAMETER:
            return self.mappings[name]['valueFrom'][param_source]

    def get_ros_topic_parameter_type(self, name):
        return self.mappings[name]['valueFrom']['msgType']

    def get_ros_topic_parameter_topic(self, name):
        return self.mappings[name]['valueFrom']['rosTopic']

    def get_ros_topic_parameter_msg_field(self, name):
        return self.mappings[name]['valueFrom'].get('msgField')
