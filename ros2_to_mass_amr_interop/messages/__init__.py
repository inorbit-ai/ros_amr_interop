import logging


class MassObject:
    def __init__(self) -> None:
        pass

    def _validate_schema(self, mass_object_type):
        pass


class IdentityReport(MassObject):
    def __init__(self, uuid, manufacturer_name,
                 robot_model, robot_serial_number,
                 base_robot_envelop, **kwargs) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.data = {}
        self.data['uuid'] = uuid
        self.data['manufacturerName'] = manufacturer_name
        self.data['robotModel'] = robot_model
        self.data['robotSerialNumber'] = robot_serial_number
        self.data['baseRobotEnvelope'] = base_robot_envelop

        # As per Mass example, data format is ISO8601
        # with timezone offset e.g. 2012-04-21T18:25:43-05:00
        self.data['timestamp'] = '2012-04-21T18:25:43-05:00'  # TODO

        # Add other optional identity report properties
        for k, v in kwargs:
            self.data[k] = v

        self._validate_schema()

    def _validate_schema(self):
        pass
