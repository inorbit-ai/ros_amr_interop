import yaml
import logging
import os

# Config files may have non static values that are
# ought to be extracted from a different source
# other than the config file
CFG_PARAMETER_STATIC = "static"
CFG_PARAMETER_ROS_TOPIC = "rosTopic"
CFG_PARAMETER_ROS_PARAMETER = "rosParameter"
CFG_PARAMETER_ENVVAR = "envVar"

SUPPORTED_EXTERNAL_VALUES = [
    CFG_PARAMETER_ROS_TOPIC,
    CFG_PARAMETER_ROS_PARAMETER,
    CFG_PARAMETER_ENVVAR
]


class MassConfig:
    def __init__(self, path=None) -> None:
        self.logger = logging.getLogger(__class__.__name__)
        _config = self._load(path)

        self.server = _config['server']
        self.mapping = _config['mapping']

    def _load(self, path) -> None:
        config = dict()
        with open(path, "r") as fd:
            try:
                config = yaml.safe_load(fd)
            except yaml.YAMLError as ex:
                self.logger.error("Failed to parse YAML config file", ex)

        self.logger.debug(f"Config file '{path}' loaded")

        # Ignoring config file key
        k = next(iter(config))
        config = config[k]
        return config

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
        if isinstance(self.mapping[name], str):
            return CFG_PARAMETER_STATIC

        if isinstance(self.mapping[name], dict):
            # Evaluate is the parameter is non static
            if 'valueFrom' in self.mapping[name]:
                param_source = self.mapping[name]['valueFrom']

                # param_source is a dict whose first key
                # is the external parameter type
                if not isinstance(param_source, dict):
                    raise ValueError(f"Invalid 'valueFrom' configuration: '{name}'")
                param_source = next(iter(param_source))

                if param_source in SUPPORTED_EXTERNAL_VALUES:
                    return param_source

        # If no supported external valueFrom configs were found,
        # assume that the parameter is static if it is an object
        if isinstance(self.mapping[name], dict):
            return CFG_PARAMETER_STATIC

        raise ValueError("Invalid parameter")

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

        if param_source == CFG_PARAMETER_STATIC:
            return self.mapping[name]

        if param_source == CFG_PARAMETER_ENVVAR:
            envvar_name = self.mapping[name]['valueFrom'][param_source]
            param_value = os.getenv(envvar_name)
            if not param_value:
                raise ValueError(f"Empty or undefined environment variable: '{envvar_name}'")
            return param_value

        if param_source == CFG_PARAMETER_ROS_TOPIC:
            return self.mapping[name]['valueFrom'][param_source]

        if param_source == CFG_PARAMETER_ROS_PARAMETER:
            return self.mapping[name]['valueFrom'][param_source]
