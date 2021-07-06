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


from datetime import datetime
from pathlib import Path
import json
import jsonschema
from datetime import timezone

# MassRobotics AMR Interop required properties
# for both Identity and Status report objects.
# Common properties
MASS_REPORT_UUID = 'uuid'
MASS_REPORT_TIMESTAMP = 'timestamp'
# Identity Report specific
MASS_REPORT_MANUFACTURER_NAME = 'manufacturerName'
MASS_REPORT_ROBOT_MODEL = 'robotModel'
MASS_REPORT_ROBOT_SERIAL_NUMBER = 'robotSerialNumber'
MASS_REPORT_BASE_ROBOT_ENVELOPE = 'baseRobotEnvelope'
# Status Report specific
MASS_REPORT_OPERATIONAL_STATE = 'operationalState'
MASS_REPORT_LOCATION = 'location'


class MassObject:
    """
    Base class for MassRobotics AMR Interop messages.

    Attributes
    ----------
        data (:obj:`dict`): parameter name and value mapping

    """

    def __init__(self, **kwargs) -> None:

        if MASS_REPORT_UUID not in kwargs:
            raise ValueError(f"Missing mandatory IdentityReport parameter {MASS_REPORT_UUID}")

        self.data = {MASS_REPORT_UUID: kwargs[MASS_REPORT_UUID]}
        self.update_timestamp()
        self.schema = self._load_schema()

    def update_timestamp(self):
        # As per Mass example, data format is ISO8601
        # with timezone offset e.g. 2012-04-21T18:25:43-05:00
        self.data[MASS_REPORT_TIMESTAMP] = datetime.now(tz=timezone.utc) \
            .replace(microsecond=0).isoformat()

    def update_parameter(self, name, value):
        """
        Update object timestamp and paramater ``name`` with ``value``.

        The method creates/overrides the property named ``name`` with the
        value ``value`` while also updates the ``timestamp`` property with
        current time, using ISO 8601 format.

        TODO: support ``timestamp`` parameter for stamped messages to favor
        timestamp from source rather than generating a new one.

        Args:
        ----
            name (str): MassRobotics AMR message parameter name.
            value (str): MassRobotics AMR message parameter value.

        """
        self.data[name] = value
        self.update_timestamp()

    def _load_schema(self):
        cwd = Path(__file__).resolve().parent
        jsonschema_def_path = str(cwd / 'schema.json')
        with open(jsonschema_def_path, 'r') as fp:
            schema = json.load(fp)

        # Fail if additional properties are found on the objects.
        # This can be consider as a workaround, the ``additionalProperties``
        # keyword should be set in the original schema.
        for obj_name in ('identityReport', 'statusReport'):
            if 'additionalProperties' not in schema[obj_name]:
                schema[obj_name]['additionalProperties'] = False

        return schema

    def validate_schema(self):
        # TODO: capture relevant exceptions and improve
        # error reporting
        jsonschema.validate(instance=self.data, schema=self.schema)


class IdentityReport(MassObject):
    """
    MassRobotics AMR Interop Identity Report message class.

    Represents Identity Report messages that are sent to compatible receivers.
    Objects are initialized with default values for all mandatory properties
    to avoid schema errors if no mappings are available.

    Attributes
    ----------
        data (:obj:`dict`): parameter name and value mapping

    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        # Identity Report required and optional properties
        # TODO: get them directly from schema file
        self.schema_properties = [
            "uuid", "timestamp", "manufacturerName", "robotModel",
            "robotSerialNumber", "baseRobotEnvelope", "maxSpeed",
            "maxRunTime", "emergencyContactInformation", "chargerType",
            "supportVendorName", "supportVendorContactInformation",
            "productDocumentation", "thumbnailImage", "cargoType",
            "cargoMaxVolume", "cargoMaxWeight"
        ]

        # Initialize Identity Report object with required properties,
        # assigning default values if not provided.
        self.data[MASS_REPORT_MANUFACTURER_NAME] = kwargs.get(
            MASS_REPORT_MANUFACTURER_NAME, "unknown")
        self.data[MASS_REPORT_ROBOT_MODEL] = kwargs.get(
            MASS_REPORT_ROBOT_MODEL, "unknown")
        self.data[MASS_REPORT_ROBOT_SERIAL_NUMBER] = kwargs.get(
            MASS_REPORT_ROBOT_SERIAL_NUMBER, "unknown")
        self.data[MASS_REPORT_BASE_ROBOT_ENVELOPE] = kwargs.get(
            MASS_REPORT_BASE_ROBOT_ENVELOPE, {"x": 0, "y": 0})

        # Add other optional identity report properties
        for property_name, property_value in kwargs.items():
            self.data[property_name] = property_value

        self.validate_schema()


class StatusReport(MassObject):
    """
    MassRobotics AMR Interop Status Report message class.

    Represents Status Report messages that are sent to compatible receivers.
    Objects are initialized with default values for all mandatory properties
    to avoid schema errors if no mappings are available.

    Attributes
    ----------
        data (:obj:`dict`): parameter name and value mapping

    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        # Status Report required and optional properties
        # TODO: get them directly from schema file
        self.schema_properties = [
            "uuid", "timestamp", "operationalState", "location",
            "velocity", "batteryPercentage", "remainingRunTime",
            "loadPercentageStillAvailable", "errorCodes",
            "destinations", "path"
        ]

        # Initialize Status Report object with required properties,
        # assigning default values if not provided.
        self.data[MASS_REPORT_OPERATIONAL_STATE] = kwargs.get(
            MASS_REPORT_OPERATIONAL_STATE, "idle")
        self.data[MASS_REPORT_LOCATION] = kwargs.get(
            MASS_REPORT_LOCATION, {
                "x": 0,
                "y": 0,
                "angle": {
                    "w": 0,
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "planarDatum": "00000000-0000-0000-0000-000000000000"
            })

        # Add other optional identity report properties
        for property_name, property_value in kwargs.items():
            self.data[property_name] = property_value

        self.validate_schema()
