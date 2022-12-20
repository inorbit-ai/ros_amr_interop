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

import rclpy
from rclpy.logging import LoggingSeverity

from uuid import uuid4

from vda5050_connector_py.vda5050_controller import VDA5050Controller
from vda5050_connector_py.utils import get_vda5050_ts
from vda5050_connector.action import ProcessVDAAction

from vda5050_msgs.msg import Order
from vda5050_msgs.msg import Node
from vda5050_msgs.msg import Edge
from vda5050_msgs.msg import NodePosition
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import CurrentAction

TEST_ORDER_ID = uuid4()
TEST_ACTION_ID1 = uuid4()
TEST_ACTION_ID2 = uuid4()


def get_test_order():
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=str(TEST_ORDER_ID),
        order_update_id=0,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=0,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    Action(
                        action_type="foo",
                        action_id=str(TEST_ACTION_ID1),
                        action_description="foo",
                        blocking_type="NONE",
                    )
                ],
            ),
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    Action(
                        action_type="bar",
                        action_id=str(TEST_ACTION_ID2),
                        action_description="bar",
                        blocking_type="NONE",
                    )
                ],
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=1,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )


def test_vda5050_controller_node_order_processing(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):

    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # Add a spy to validate used process vda actions parameters
    spy_send_adapter_process_vda_action = mocker.spy(
        node, "send_adapter_process_vda_action"
    )

    # generate an order and let the node process it
    order = get_test_order()
    node.process_order(order)

    assert len(node._current_state.action_states) == 2

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    rclpy.spin_once(node)

    spy_send_adapter_process_vda_action.assert_called_once()

    rclpy.spin_once(adapter_node)

    # Mocked action server publishes feedback to let the controller
    # know the first action is being initialized
    action_server_process_vda_action.publish_feedback(
        feedback=ProcessVDAAction.Feedback(
            current_action=CurrentAction(
                action_id=str(TEST_ACTION_ID1),
                action_description="foo",
                action_status=CurrentAction.INITIALIZING,
                result_description="",
            )
        )
    )

    # Get action state from controller current state
    action_state = next(
        (
            current_action
            for current_action in node._current_state.action_states
            if current_action.action_id == str(TEST_ACTION_ID1)
        )
    )

    assert action_state.action_description == "foo"
    assert action_state.action_status == CurrentAction.INITIALIZING
