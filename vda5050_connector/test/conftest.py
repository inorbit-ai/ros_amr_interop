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

import pytest
import paho.mqtt.client as mqtt

import rclpy
from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction
from vda5050_connector_py.vda5050_controller import DEFAULT_NAV_TO_NODE_ACT_NAME
from vda5050_connector_py.vda5050_controller import DEFAULT_VDA_ACTION_ACT_NAME
from vda5050_connector.srv import GetState
from vda5050_connector_py.vda5050_controller import DEFAULT_GET_STATE_SVC_NAME
from vda5050_connector.srv import SupportedActions
from vda5050_connector_py.vda5050_controller import DEFAULT_SUPPORTED_ACTIONS_SVC_NAME
from rclpy.qos import qos_profile_action_status_default


class MockActionServerNavigateToNode:
    def __init__(self, node):
        self.logger = node.get_logger()
        self.goal_srv = node.create_service(
            NavigateToNode.Impl.SendGoalService,
            f"/vda5050/robots/robot_1/{DEFAULT_NAV_TO_NODE_ACT_NAME}/_action/send_goal",
            self.goal_callback,
        )
        self.cancel_srv = node.create_service(
            NavigateToNode.Impl.CancelGoalService,
            f"/vda5050/robots/robot_1/{DEFAULT_NAV_TO_NODE_ACT_NAME}/_action/cancel_goal",
            self.cancel_callback,
        )
        self.result_srv = node.create_service(
            NavigateToNode.Impl.GetResultService,
            f"/vda5050/robots/robot_1/{DEFAULT_NAV_TO_NODE_ACT_NAME}/_action/get_result",
            self.result_callback,
        )
        self.feedback_pub = node.create_publisher(
            NavigateToNode.Impl.FeedbackMessage,
            f"/vda5050/robots/robot_1/{DEFAULT_NAV_TO_NODE_ACT_NAME}/_action/feedback",
            1,
        )
        self.goal_status_pub = node.create_publisher(
            NavigateToNode.Impl.GoalStatusMessage,
            f"/vda5050/robots/robot_1/{DEFAULT_NAV_TO_NODE_ACT_NAME}/_action/status",
            qos_profile_action_status_default,
        )

    def goal_callback(self, request, response):
        # Save goal ID to send feedback later
        self.goal_id = request.goal_id
        response.accepted = True
        self.logger.info(f"Goal received {response}")
        return response

    def cancel_callback(self, request, response):
        response.goals_canceling.append(request.goal_info)
        return response

    def result_callback(self, request, response):
        self.logger.info(f"Result callback: {response}")
        return response

    def publish_feedback(self, feedback):
        feedback_message = NavigateToNode.Impl.FeedbackMessage(
            goal_id=self.goal_id, feedback=feedback
        )
        self.feedback_pub.publish(feedback_message)


class MockActionServerProcessVDAAction:
    def __init__(self, node):
        self.logger = node.get_logger()
        self.goal_srv = node.create_service(
            ProcessVDAAction.Impl.SendGoalService,
            f"/vda5050/robots/robot_1/{DEFAULT_VDA_ACTION_ACT_NAME}/_action/send_goal",
            self.goal_callback,
        )
        self.cancel_srv = node.create_service(
            ProcessVDAAction.Impl.CancelGoalService,
            f"/vda5050/robots/robot_1/{DEFAULT_VDA_ACTION_ACT_NAME}/_action/cancel_goal",
            self.cancel_callback,
        )
        self.result_srv = node.create_service(
            ProcessVDAAction.Impl.GetResultService,
            f"/vda5050/robots/robot_1/{DEFAULT_VDA_ACTION_ACT_NAME}/_action/get_result",
            self.result_callback,
        )
        self.feedback_pub = node.create_publisher(
            ProcessVDAAction.Impl.FeedbackMessage,
            f"/vda5050/robots/robot_1/{DEFAULT_VDA_ACTION_ACT_NAME}/_action/feedback",
            1,
        )
        self.goal_status_pub = node.create_publisher(
            ProcessVDAAction.Impl.GoalStatusMessage,
            f"/vda5050/robots/robot_1/{DEFAULT_VDA_ACTION_ACT_NAME}/_action/status",
            qos_profile_action_status_default,
        )

    def goal_callback(self, request, response):
        # Save goal ID to send feedback later
        self.goal_id = request.goal_id
        response.accepted = True
        self.logger.info(f"Goal received {response}")
        return response

    def cancel_callback(self, request, response):
        response.goals_canceling.append(request.goal_info)
        return response

    def result_callback(self, request, response):
        self.logger.info(f"Result callback: {response}")
        return response

    def publish_feedback(self, feedback):
        feedback_message = ProcessVDAAction.Impl.FeedbackMessage(
            goal_id=self.goal_id, feedback=feedback
        )
        self.feedback_pub.publish(feedback_message)


@pytest.fixture
def setup_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def adapter_node(setup_rclpy):
    return rclpy.create_node("FakeAdapter")


@pytest.fixture
def action_server_nav_to_node(adapter_node):
    return MockActionServerNavigateToNode(adapter_node)


@pytest.fixture
def action_server_process_vda_action(adapter_node):
    return MockActionServerProcessVDAAction(adapter_node)


@pytest.fixture
def service_get_state(adapter_node):
    return adapter_node.create_service(
        GetState,
        f"/vda5050/robots/robot_1/{DEFAULT_GET_STATE_SVC_NAME}",
        lambda _: adapter_node.get_logger().info("State request"),
    )


@pytest.fixture
def service_supported_actions(adapter_node):
    return adapter_node.create_service(
        SupportedActions,
        f"/vda5050/robots/robot_1/{DEFAULT_SUPPORTED_ACTIONS_SVC_NAME}",
        lambda _: adapter_node.get_logger().info("Supported actions request"),
    )


@pytest.fixture
def mock_mqtt_client(mocker):
    fake_mid = 52
    fake_rc = 0
    mock = mocker.patch.object(mqtt, "Client")
    mock_mqtt_client = mock.return_value
    mock_mqtt_client.subscribe = mocker.MagicMock(return_value=(fake_rc, fake_mid))
    mock_mqtt_client.unsubscribe = mocker.MagicMock(return_value=(fake_rc, fake_mid))
    mock_mqtt_client.publish = mocker.MagicMock(return_value=(fake_rc, fake_mid))
    mock_mqtt_client.connect.return_value = 0
    mock_mqtt_client.reconnect.return_value = 0
    mock_mqtt_client.disconnect.return_value = 0
    return mock_mqtt_client
