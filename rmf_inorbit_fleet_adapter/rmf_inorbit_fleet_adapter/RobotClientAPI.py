# Copyright 2023 InOrbit, Inc.
#
# This file has been copied and adapted from
# https://github.com/open-rmf/fleet_adapter_template/blob/32f47a451ea67fa299a4fdb1a189d5db8df84b6b/fleet_adapter_template/fleet_adapter_template/RobotClientAPI.py
#
# The following is a summary of the introduced changes:
# - Implemented or modified all methods
# - Modified the intended use of the class to one instance per robot
#
# The following is a copy of the original license note:
#
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from schema import Schema, Optional

from rclpy.impl.rcutils_logger import RcutilsLogger
from .Requester import Requester

from .utils import euc_distance, angle_diff


class RobotAPI:
    """
    The RobotAPI class is a wrapper for API calls to the robot. Here are implemented the
    functions which will be used by the RobotCommandHandle.
    """

    API_KEY_HEADER = "x-auth-inorbit-app-key"
    STOP_COMMAND_PAYLOAD = {'actionId': 'CancelNavGoal-000000'}

    CONFIG_SCHEMA = Schema({
        "max_delay": float,
        "robot_id": str,
        "battery_charge_attribute_id": str,
        "charging_status_attribute_id": str,
        Optional("actions", default={}): {  # Empty yaml dicts are parsed as None. Default actions to empty dict
            Optional("dock", default=None): {
                "action_id": str,
                "dock_name": str,
            }
        }
    })

    def __init__(self, api_key: str, name: str, robot_config: dict, max_lin_speed: float, logger: RcutilsLogger, base_url="https://api.inorbit.ai/robots/", threshold_distance=0.22, threshold_angle=0.2):
        """
        Takes the parameters required to submit requests to InOrbitAPI

        Args:
            api_key (str): InOrbit API KEY
            name (str): Robot name as appears in the adapter config. It is taken with the purpose of providing more descriptive logging
            robot_config (dict): Robot config set in the adapter configuration. The required keys are:
                max_delay (float): Allowed seconds of delay of the current itinerary before it gets interrupted and replanned (will not affect RobotAPI)
                robot_id (str): InOrbit robot id
                battery_charge_attribute_id (str): InOrbit attribute definition for battery charge
                charging_status_attribute_id (str): InOrbit attribute definition for charging status
                actions (dict?): Optional. Information about each defined action. Currently supported actions:
                    dock (dict):
                        action_id (str): Action ID of the docking action
                        dock_name (str): 'dock_name' of the dock corresponding to the action
            max_lin_speed (float): Maximum linear speed defined in the fleet config
            logger (RcutilsLogger): Node logger
            base_url (str): InOrbit API base url. Default: "https://api.inorbit.ai/robots/"
            threshold_distance (float): Distance in meters from position to goal forwards or backwards. Threshold to goal before stop. Default is 0.15m
            threshold_angle (float): Angle in radians from pose angle to goal pose each side. Threshold to goal before stop. Default is 0.13 (7.5°)
        """

        assert api_key, 'api_key is empty or None'
        assert name, 'name is empty or None'
        assert max_lin_speed > 0, 'max_linear_speed is not greater than zero'
        assert logger, 'logger is None'
        assert threshold_angle > 0 and threshold_distance > 0, "Proximity thresholds must be grater than zero"

        self.CONFIG_SCHEMA.validate(robot_config)
        logger.info("RobotAPI configuration is valid")

        self.headers = {self.API_KEY_HEADER: api_key}

        self.name = name
        self.robot_config = robot_config
        self.max_lin_speed = max_lin_speed
        self.logger = logger
        self.dock_enabled = 'dock' in self.robot_config['actions']

        self.requester = Requester(
            base_url=base_url,
            robot_id=self.robot_config['robot_id'],
            headers=self.headers,
            logger=self.logger)

        self.threshold_distance = threshold_distance
        self.threshold_angle = threshold_angle

        # Test connectivity
        self.connected = self.check_connection()
        if self.connected:
            self.logger.info("Successfully able to query API server")
        else:
            self.logger.warn("Unable to query API server")

        # Keep track of last requested navigation pose and process ID
        self.last_requested_pose = None
        self.last_requested_action = None

    def check_connection(self) -> bool:
        ''' Return True if connection to the robot API server is successful'''
        response = self.requester.robot_get_request("")
        if not response:
            self.logger.error("Exception calling InOrbit API")
            return False
        return response.status_code == 200

    def position(self) -> list | None:
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        response = self.requester.robot_get_request("/localization/pose")
        if not response:
            return None
        res_json = response.json()
        if response.status_code != 200:
            return None
        return [res_json['x'], res_json['y'], res_json['theta']]

    def navigate(self, pose) -> bool:
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        self.last_requested_pose = pose
        response = self.requester.robot_post_request(
            "/navigation/waypoints", json={'waypoints': [{'x': pose[0], 'y': pose[1], 'theta': pose[2]}]})
        if not response:
            return False
        res_json = response.json()
        return response.status_code == 200

    def start_docking(self, dock_name: str) -> bool:
        ''' Request the robot to begin the docking process. It will compare the received dock name to the
            dock name linked to the action set for this robot, and fail if they are different.
            Return True if the robot has accepted the request, else False '''

        if not self.dock_enabled:
            return False

        assert dock_name == self.robot_config['actions']['dock'][
            'dock_name'], "Tried to execute docking action into a non matching dock"

        action_id = self.robot_config['actions']['dock']['action_id']
        response = self.requester.robot_post_request(
            "/actions", json={"actionId": action_id, "parameters": {}})

        if not response:
            return False
        res_json = response.json()
        self.logger.info(f"Start docking response: {res_json}")

        return response.status_code == 200

    def is_robot_charging(self) -> bool | None:
        ''' Returns whether the robot is charging '''

        response = self.requester.robot_get_request(
            f"/attributes/{self.robot_config['charging_status_attribute_id']}")
        if not response:
            return None

        res_json = response.json()
        if not isinstance(res_json['value'], str):
            return None

        return res_json['value'].lower() == 'charging'

    def stop(self) -> bool:
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        response = self.requester.robot_post_request(
            "/actions", json=self.STOP_COMMAND_PAYLOAD)
        if not response:
            return False
        res_json = response.json()
        return response.status_code == 200

    def navigation_remaining_duration(self):
        ''' Return a rough estimate of the number of seconds remaining for the robot to reach
            its destination
            Estimated from the distance left at an speed lower than the maximum'''
        try:
            [curr_x, curr_y, _] = self.position()
            [wp_x, wp_y, _] = self.last_requested_pose
            distance_left = euc_distance(curr_x, curr_y, wp_x, wp_y)
        except:
            self.logger.error("Cannot estimate remaining duration")
            return 0.0
        duration = distance_left / self.max_lin_speed
        return duration

    def navigation_completed(self) -> list | None:
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        if self.last_requested_pose is None:
            self.logger.warn(
                "Navigation completed: No last pose, returning True")
            return True
        [ax, ay, atheta] = self.last_requested_pose
        [bx, by, btheta] = self.position()
        res = (euc_distance(ax, ay, bx, by) < self.threshold_distance and
               math.fabs(angle_diff(atheta, btheta)) < self.threshold_angle)
        return res

    def battery_soc(self) -> int | None:
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        response = self.requester.robot_get_request(
            f"/attributes/{self.robot_config['battery_charge_attribute_id']}")
        if not response:
            return None
        res_json = response.json()
        if response.status_code != 200:
            return None
        return res_json['value']
