# Copyright 2023 InOrbit, Inc.

import requests
from requests import Response
from rclpy.impl.rcutils_logger import RcutilsLogger


class Requester:
    """Generates requests to InOrbit REST API"""

    def __init__(self, base_url: str, robot_id: str, headers: dict, logger: RcutilsLogger) -> None:
        """
        Args:
            base_url (str): Base URL of the InOrbit API
            robot_id (str): ID of the robot the requests will be made to
            headers (dict): Request headers
            logger (RcutilsLogger): Node logger
        """

        assert base_url, "Base URL is empty"
        assert robot_id, "No robot_id provided"
        assert isinstance(headers, dict), "Incorrect header format"
        assert logger, "Logger not provided"

        self.base_url = base_url
        self.robot_id = robot_id
        self.headers = headers
        self.logger = logger

    def robot_get_request(self, endpoint: str, json=None) -> Response | None:
        """
        Makes a GET request to '{BASE_URL}{robot_id}{endpoint}' with the class defined headers and returns the response if successful or None if an error occurred

        Args:
            endpoint (str): With leading forward slash
        """

        url = f"{self.base_url}{self.robot_id}{endpoint}"
        try:
            res = requests.get(url, headers=self.headers, json=json)
            if res.status_code >= 300:
                self.logger.warn(
                    f"\nStatus code {res.status_code} GETting {url} with body {json}\nmessage: {res.text}")
            return res
        except Exception as e:
            self.logger.error(f"Exception GETting {url}: {e}")
        return None

    def robot_post_request(self, endpoint: str, json=None) -> Response | None:
        """
        Makes a POST request to '{BASE_URL}{robot_id}{endpoint}' with the class defined headers and returns the response if successful or None if an error occurred

        Args:
            endpoint (str): With leading forward slash
        """

        url = f"{self.base_url}{self.robot_id}{endpoint}"
        try:
            res = requests.post(url, headers=self.headers, json=json)
            if res.status_code >= 300:
                self.logger.warn(
                    f"\nStatus code {res.status_code} POSTing {url} with body {json}\nmessage: {res.text}")
            return res
        except Exception as e:
            self.logger.error(f"Exception POSTing {url}: {e}")
        return None
