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
from rclpy.task import Future
from rclpy.parameter import Parameter

from uuid import uuid4

from vda5050_connector_py.vda5050_controller import VDA5050Controller
from vda5050_connector_py.vda5050_controller import OrderAcceptModes
from vda5050_connector_py.vda5050_controller import OrderRejectErrors
from vda5050_connector_py.utils import get_vda5050_ts
from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import NavigateThroughNodes
from vda5050_msgs.msg import Order
from vda5050_msgs.msg import Node
from vda5050_msgs.msg import Edge
from vda5050_msgs.msg import NodePosition
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import ActionParameter
from vda5050_msgs.msg import CurrentAction


def get_order_new(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
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
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node4",
                sequence_id=6,
                released=True,
                node_position=NodePosition(
                    x=-0.17,
                    y=1.74,
                    theta=-2.6,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node1",
                sequence_id=8,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
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
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge3",
                sequence_id=5,
                released=True,
                start_node_id="node3",
                end_node_id="node4",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge4",
                sequence_id=7,
                released=True,
                start_node_id="node4",
                end_node_id="node1",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )


def get_order_update(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=8,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node2",
                sequence_id=10,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=9,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            )
        ],
    )


def get_order_w_unreleased_new(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
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
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node4",
                sequence_id=6,
                released=True,
                node_position=NodePosition(
                    x=-0.17,
                    y=1.74,
                    theta=-2.6,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node1",
                sequence_id=8,
                released=False,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
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
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge3",
                sequence_id=5,
                released=True,
                start_node_id="node3",
                end_node_id="node4",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge4",
                sequence_id=7,
                released=False,
                start_node_id="node4",
                end_node_id="node1",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )


def get_stitch_orders(order_id=str(uuid4())):
    action1 = Action(
        action_type="foo",
        action_id=str(uuid4()),
        action_description="Foo description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="foo", value="bar")
        ]
    )
    action2 = Action(
        action_type="bar",
        action_id=str(uuid4()),
        action_description="Bar description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="foo", value="bar")
        ]
    )
    action3 = Action(
        action_type="foobar",
        action_id=str(uuid4()),
        action_description="FooBar description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="abc", value="xyz")
        ]
    )
    base_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
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
                    action1
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
                    action2
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

    stitch_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=1,
        nodes=[
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
                    action2
                ],
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action3
                ],
            ),
        ],
        edges=[
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )

    return [base_order, stitch_order]


def test_vda5050_controller_node_new_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):

    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_to_node = mocker.spy(
        node, "send_adapter_navigate_to_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # generate an order and let the node process it
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.NEW)

    rclpy.spin_once(adapter_node)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 0

    # The order has 5 nodes and 4 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.
    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[0], node=order.nodes[1]
    )

    # Future for invoking adapter navigation goal result callback
    future = Future()
    future.set_result(result=NavigateToNode.Result())

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[1], node=order.nodes[2]
    )

    assert len(node._current_state.node_states) == 3
    assert len(node._current_state.edge_states) == 3
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[2], node=order.nodes[3]
    )

    assert len(node._current_state.node_states) == 2
    assert len(node._current_state.edge_states) == 2
    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[3], node=order.nodes[4]
    )
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1
    assert node._current_state.last_node_id == "node4"
    assert node._current_state.last_node_sequence_id == 6

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8


def test_vda5050_controller_node_update_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_to_node = mocker.spy(
        node, "send_adapter_navigate_to_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # Send first new order
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=NavigateToNode.Result())

    # The NEW order contains 5 nodes and 4 edges. The first node (in deviation range)
    # is processed and remove, and 4 nodes are send to navigate to.
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    # Finish initial order

    spy_accept_order.reset_mock()
    spy_send_adapter_navigate_to_node.reset_mock()

    # Send update order
    order = get_order_update(order_id, 1)  # Same order id
    node.process_order(order)
    node._on_active_order()

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.UPDATE)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 1

    # The order has 2 nodes and 1 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.
    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[0], node=order.nodes[1]
    )

    # Future for invoking adapter navigation goal result callback
    future = Future()
    future.set_result(result=NavigateToNode.Result())

    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 10


def test_vda5050_controller_node_stitch_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # Get the base order and the stitch order, both with two nodes
    # and one edge each. The resulting (stitched) order should contain
    # three nodes and two edges:
    # (A -> B) + (B -> C) = A -> B -> C
    # All three nodes have an action.
    [base_order, stitch_order] = get_stitch_orders()

    node.process_order(base_order)

    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate the adapter reached navigation goals
    future = Future()
    result_response = NavigateToNode.Impl.GetResultService.Response()
    result_response.status = 4  # GoalStatus.STATUS_SUCCEEDED
    future.set_result(result=result_response)

    # The base order contains 2 nodes and 1 edge. The first node (in deviation range)
    # is processed and removed, and 1 node is sent to navigate to.
    node._navigate_to_node_result_callback(future)

    assert len(node._current_order.nodes) == 2
    assert len(node._current_order.edges) == 1
    assert len(node._current_state.action_states) == 2

    spy_accept_order.reset_mock()
    # Process the stitching order
    node.process_order(stitch_order)
    spy_accept_order.assert_called_once_with(order=stitch_order, mode=OrderAcceptModes.STITCH)

    assert node._current_state.order_id == stitch_order.order_id
    assert node._current_state.order_update_id == 1

    assert len(node._current_order.nodes) == 3
    assert len(node._current_order.edges) == 2

    # Stitch node (node2, seq=2) was already visited — dedup drops it
    assert len(node._current_state.node_states) == 1
    assert node._current_state.node_states[0].node_id == "node3"
    assert len(node._current_state.edge_states) == 1
    assert len(node._current_state.action_states) == 3

    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    node._navigate_to_node_result_callback(future)
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert len(node._current_state.action_states) == 3

    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4


def test_vda5050_controller_node_reject_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate that the order has been rejected
    spy_reject_order = mocker.spy(node, "_reject_order")

    # UPDATE test fail - lower order_update_id

    # Send first new order
    order_id = str(uuid4())
    order = get_order_new(order_id, 1)
    node.process_order(order)

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=NavigateToNode.Result())

    # The NEW order contains 5 nodes and 4 edges. The first node (in deviation range)
    # is processed and remove, and 4 nodes are send to navigate to.
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    # Finish initial order

    spy_reject_order.reset_mock()

    order = get_order_update(order_id, 0)  # Same order id, lower order_update_id
    node.process_order(order)

    spy_reject_order.assert_called_once_with(
        order=order,
        error=OrderRejectErrors.ORDER_UPDATE_ERROR,
        description="New update id 0 lower than old update id 1",
    )


def test_vda5050_controller_node_new_order_nav_through_nodes(
    mocker,
    adapter_node,
    action_server_nav_through_nodes,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Tests the original node list with the nav through nodes functionality."""
    nav_through_nodes_param = Parameter(
        "enable_nav_through_nodes", type_=Parameter.Type.BOOL, value=True)
    node = VDA5050Controller(parameter_overrides=[nav_through_nodes_param])
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_through_nodes = mocker.spy(
        node, "send_adapter_navigate_through_nodes"
    )

    spy_process_last_edge_node = mocker.spy(
        node, "_process_last_edge_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # generate an order and let the node process it
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.NEW)

    rclpy.spin_once(adapter_node)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 0

    # The order has 5 nodes and 4 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Assert the navigation through nodes goal was sent to the adapter,
    # and that the parameters matches order's released edges
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.

    spy_send_adapter_navigate_through_nodes.assert_called_once_with(
        edges=order.edges[:4], nodes=order.nodes[1:5]
    )

    feedback_msg = NavigateThroughNodes.Impl.FeedbackMessage()
    # Check that a feedback of the current node doesn't affect the current state
    feedback_msg.feedback.last_node = order.nodes[0]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_not_called()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4
    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Next node has been reached and a feedback message is published
    feedback_msg.feedback.last_node = order.nodes[1]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 3
    assert len(node._current_state.edge_states) == 3
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    # Next node has been reached and a feedback message is published
    feedback_msg.feedback.last_node = order.nodes[2]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 2
    assert len(node._current_state.edge_states) == 2
    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4

    # Next node has been reached and a feedback message is published
    feedback_msg.feedback.last_node = order.nodes[3]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1
    assert node._current_state.last_node_id == "node4"
    assert node._current_state.last_node_sequence_id == 6

    # Last node reached as indicated by the result coming through
    # A feedback message shouldn't be published for the final node in a navigation order

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=NavigateThroughNodes.Result())
    node._navigate_through_nodes_result_callback(future)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8


def test_vda5050_controller_node_new_order_nav_through_nodes_unreleased_nodes(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_nav_through_nodes,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Tests with some unreleased nodes on the horizon and navigate through nodes."""
    nav_through_nodes_param = Parameter(
        "enable_nav_through_nodes", type_=Parameter.Type.BOOL, value=True)
    node = VDA5050Controller(parameter_overrides=[nav_through_nodes_param])
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_through_nodes = mocker.spy(
        node, "send_adapter_navigate_through_nodes"
    )

    spy_process_last_edge_node = mocker.spy(
        node, "_process_last_edge_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # generate an order and let the node process it
    order_id = str(uuid4())
    order = get_order_w_unreleased_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.NEW)

    rclpy.spin_once(adapter_node)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 0

    # The order has 5 nodes and 4 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.

    spy_send_adapter_navigate_through_nodes.assert_called_once_with(
        edges=order.edges[:3], nodes=order.nodes[1:4]
    )

    feedback_msg = NavigateThroughNodes.Impl.FeedbackMessage()
    # Check that a feedback of the current node doesn't affect the current state
    feedback_msg.feedback.last_node = order.nodes[0]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_not_called()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4
    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Next node has been reached and a feedback message is published
    feedback_msg.feedback.last_node = order.nodes[1]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 3
    assert len(node._current_state.edge_states) == 3
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    # Next node has been reached and a feedback message is published
    feedback_msg.feedback.last_node = order.nodes[2]
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()

    assert len(node._current_state.node_states) == 2
    assert len(node._current_state.edge_states) == 2
    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4

    # Last node reached as indicated by the result coming through
    # A feedback message shouldn't be published for the final node in a navigation order

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=NavigateThroughNodes.Result())
    node._navigate_through_nodes_result_callback(future)

    spy_process_last_edge_node.assert_called_once()
    spy_process_last_edge_node.reset_mock()
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1
    assert node._current_state.last_node_id == "node4"
    assert node._current_state.last_node_sequence_id == 6

    # This is the final node with a released horizon.
    # Therefore a request should be flagged on the next tick
    node._on_active_order()
    assert node._current_state.new_base_request is True


def test_vda5050_controller_stitch_while_navigating(
    mocker,
    adapter_node,
    action_server_nav_through_nodes,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Test that a stitch order arriving mid-navigation extends the in-flight goal
    via ExtendNavigation instead of sending a new nav goal (which would be rejected)."""
    nav_through_nodes_param = Parameter(
        "enable_nav_through_nodes", type_=Parameter.Type.BOOL, value=True)
    node = VDA5050Controller(parameter_overrides=[nav_through_nodes_param])
    node.logger.set_level(LoggingSeverity.DEBUG)

    spy_send_adapter_navigate_through_nodes = mocker.spy(
        node, "send_adapter_navigate_through_nodes"
    )
    spy_accept_order = mocker.spy(node, "_accept_order")
    spy_extend_nav = mocker.spy(node, "_extend_navigation_response_callback")

    # Mock the extend_nav service client so we don't rely on DDS discovery
    from vda5050_connector.srv import ExtendNavigation
    mock_extend_response = ExtendNavigation.Response(success=True, message="ok")
    mock_extend_future = Future()
    mock_extend_future.set_result(mock_extend_response)
    node._extend_nav_svc_cli = mocker.MagicMock()
    node._extend_nav_svc_cli.call_async = mocker.MagicMock(return_value=mock_extend_future)

    # --- Build a base order with 3 released nodes + 1 unreleased horizon node ---
    order_id = str(uuid4())
    base_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=0,
        nodes=[
            Node(node_id="A", sequence_id=0, released=True,
                 node_position=NodePosition(x=0.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="B", sequence_id=2, released=True,
                 node_position=NodePosition(x=1.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="C", sequence_id=4, released=True,
                 node_position=NodePosition(x=2.0, y=0.0, theta=0.0, map_id="map")),
            # Horizon (unreleased)
            Node(node_id="D", sequence_id=6, released=False,
                 node_position=NodePosition(x=3.0, y=0.0, theta=0.0, map_id="map")),
        ],
        edges=[
            Edge(edge_id="e1", sequence_id=1, released=True,
                 start_node_id="A", end_node_id="B"),
            Edge(edge_id="e2", sequence_id=3, released=True,
                 start_node_id="B", end_node_id="C"),
            # Horizon (unreleased)
            Edge(edge_id="e3", sequence_id=5, released=False,
                 start_node_id="C", end_node_id="D"),
        ],
    )

    # Process the base order
    node.process_order(base_order)
    rclpy.spin_once(node)
    spy_accept_order.assert_called_once_with(order=base_order, mode=OrderAcceptModes.NEW)
    rclpy.spin_once(adapter_node)

    # The controller should navigate through released nodes B and C (2 edges)
    spy_send_adapter_navigate_through_nodes.assert_called_once_with(
        edges=base_order.edges[:2], nodes=base_order.nodes[1:3]
    )
    assert node._nav_through_nodes_last_seq == 4  # seq of node C

    assert node._current_state.last_node_id == "A"
    assert node._current_state.last_node_sequence_id == 0
    assert len(node._current_state.node_states) == 3  # B, C, D (horizon)
    assert len(node._current_state.edge_states) == 3

    # --- Robot reaches node B (feedback) ---
    feedback_msg = NavigateThroughNodes.Impl.FeedbackMessage()
    feedback_msg.feedback.last_node = base_order.nodes[1]  # B
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    assert node._current_state.last_node_id == "B"
    assert node._current_state.last_node_sequence_id == 2
    assert len(node._current_state.node_states) == 2  # C, D

    # --- Stitch order arrives while robot is between B and C ---
    # Extends the base: releases D and adds E
    spy_accept_order.reset_mock()
    spy_send_adapter_navigate_through_nodes.reset_mock()

    stitch_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=1,
        nodes=[
            # Stitch node (must match last released node of old base)
            Node(node_id="C", sequence_id=4, released=True,
                 node_position=NodePosition(x=2.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="D", sequence_id=6, released=True,
                 node_position=NodePosition(x=3.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="E", sequence_id=8, released=True,
                 node_position=NodePosition(x=4.0, y=0.0, theta=0.0, map_id="map")),
        ],
        edges=[
            Edge(edge_id="e3", sequence_id=5, released=True,
                 start_node_id="C", end_node_id="D"),
            Edge(edge_id="e4", sequence_id=7, released=True,
                 start_node_id="D", end_node_id="E"),
        ],
    )

    node.process_order(stitch_order)
    spy_accept_order.assert_called_once_with(order=stitch_order, mode=OrderAcceptModes.STITCH)

    # The controller should NOT send a new navigation goal (would be rejected)
    spy_send_adapter_navigate_through_nodes.assert_not_called()

    # It should have called extend_nav_svc_cli.call_async
    node._extend_nav_svc_cli.call_async.assert_called_once()
    req = node._extend_nav_svc_cli.call_async.call_args[0][0]
    # nodes[0] is the stitch reference node (C), plus 2 new nodes (D, E)
    assert len(req.nodes) == 3
    assert req.nodes[0].node_id == "C"
    assert req.nodes[1].node_id == "D"
    assert req.nodes[2].node_id == "E"
    assert len(req.edges) == 2

    # The response callback should have been called (future was pre-resolved)
    spy_extend_nav.assert_called_once()

    # _nav_through_nodes_last_seq should now cover up to E
    assert node._nav_through_nodes_last_seq == 8
    assert node._current_state.last_node_id == "B"
    assert node._current_state.last_node_sequence_id == 2

    # --- Robot reaches node C (feedback) ---
    feedback_msg.feedback.last_node = base_order.nodes[2]  # C
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    assert node._current_state.last_node_id == "C"
    assert node._current_state.last_node_sequence_id == 4

    # --- Navigation result arrives (goal finishes at the original last_seq=4,
    # but we extended to 8, so result callback should consume up to 8) ---
    # Actually, the result callback consumes up to _nav_through_nodes_last_seq
    # which is now 8 thanks to the extension.
    future = Future()
    result_response = NavigateThroughNodes.Impl.GetResultService.Response()
    result_response.status = 4  # GoalStatus.STATUS_SUCCEEDED
    future.set_result(result=result_response)
    node._navigate_through_nodes_result_callback(future)

    # All nodes up to E (seq 8) should now be consumed
    assert node._current_state.last_node_id == "E"
    assert node._current_state.last_node_sequence_id == 8
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0


def test_vda5050_controller_skipped_feedback_nodes(
    mocker,
    adapter_node,
    action_server_nav_through_nodes,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Test that when intermediate node feedback is missed (C, D), the result
    callback still processes each skipped node individually (C, D, E)."""
    nav_through_nodes_param = Parameter(
        "enable_nav_through_nodes", type_=Parameter.Type.BOOL, value=True)
    node = VDA5050Controller(parameter_overrides=[nav_through_nodes_param])
    node.logger.set_level(LoggingSeverity.DEBUG)

    spy_process_last_edge_node = mocker.spy(node, "_process_last_edge_node")
    spy_process_node = mocker.spy(node, "_process_node")

    # --- Order: A(0) → B(2) → C(4) → D(6) → E(8) ---
    order_id = str(uuid4())
    order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=0,
        nodes=[
            Node(node_id="A", sequence_id=0, released=True,
                 node_position=NodePosition(x=0.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="B", sequence_id=2, released=True,
                 node_position=NodePosition(x=1.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="C", sequence_id=4, released=True,
                 node_position=NodePosition(x=2.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="D", sequence_id=6, released=True,
                 node_position=NodePosition(x=3.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="E", sequence_id=8, released=True,
                 node_position=NodePosition(x=4.0, y=0.0, theta=0.0, map_id="map")),
        ],
        edges=[
            Edge(edge_id="e1", sequence_id=1, released=True,
                 start_node_id="A", end_node_id="B"),
            Edge(edge_id="e2", sequence_id=3, released=True,
                 start_node_id="B", end_node_id="C"),
            Edge(edge_id="e3", sequence_id=5, released=True,
                 start_node_id="C", end_node_id="D"),
            Edge(edge_id="e4", sequence_id=7, released=True,
                 start_node_id="D", end_node_id="E"),
        ],
    )

    # Process the order — robot is at A, nav goal sent for B→C→D→E
    node.process_order(order)
    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    assert node._current_state.last_node_id == "A"
    assert node._current_state.last_node_sequence_id == 0
    assert node._nav_through_nodes_last_seq == 8
    assert len(node._current_state.node_states) == 4  # B, C, D, E
    assert len(node._current_state.edge_states) == 4

    # --- Feedback: robot reaches B (seq 2) ---
    feedback_msg = NavigateThroughNodes.Impl.FeedbackMessage()
    feedback_msg.feedback.last_node = order.nodes[1]  # B
    node._navigate_through_nodes_feedback_callback(feedback_msg)

    assert node._current_state.last_node_id == "B"
    assert node._current_state.last_node_sequence_id == 2
    assert len(node._current_state.node_states) == 3  # C, D, E
    assert len(node._current_state.edge_states) == 3

    # --- No feedback for C or D (missed / proximity threshold not met) ---

    # Reset spies before result callback to count only the skipped-node processing
    spy_process_last_edge_node.reset_mock()
    spy_process_node.reset_mock()

    # Spy on _publish_state to verify each node triggers a separate state message
    spy_publish_state = mocker.spy(node, "_publish_state")

    # --- Result: navigation goal completes (robot arrived at E) ---
    future = Future()
    result_response = NavigateThroughNodes.Impl.GetResultService.Response()
    result_response.status = 4  # GoalStatus.STATUS_SUCCEEDED
    future.set_result(result=result_response)
    node._navigate_through_nodes_result_callback(future)

    # The result callback should have consumed C, D, E individually
    assert spy_process_last_edge_node.call_count == 3
    assert spy_process_node.call_count == 3

    # Each node must publish its own state message — not batched into one
    assert spy_publish_state.call_count == 3

    # Verify _process_node was called in order: C, D, E
    process_node_calls = [call.kwargs.get("node") or call.args[0]
                          for call in spy_process_node.call_args_list]
    assert process_node_calls[0].node_id == "C"
    assert process_node_calls[0].sequence_id == 4
    assert process_node_calls[1].node_id == "D"
    assert process_node_calls[1].sequence_id == 6
    assert process_node_calls[2].node_id == "E"
    assert process_node_calls[2].sequence_id == 8

    # Final state: all nodes consumed, robot at E
    assert node._current_state.last_node_id == "E"
    assert node._current_state.last_node_sequence_id == 8
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0


def test_stitch_preserves_action_statuses(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Verify that a stitch preserves existing action statuses (FINISHED/RUNNING)
    instead of resetting them to WAITING, and only appends truly new actions."""
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    [base_order, stitch_order] = get_stitch_orders()
    action1 = base_order.nodes[0].actions[0]
    action2 = base_order.nodes[1].actions[0]
    action3 = stitch_order.nodes[1].actions[0]

    # Accept the base order
    node.process_order(base_order)
    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate robot reaching node2 via _process_last_edge_node
    node._process_last_edge_node()

    assert len(node._current_state.action_states) == 2

    # Simulate action progression: action1 finished, action2 running
    node._update_action_status(action1.action_id, CurrentAction.FINISHED)
    node._update_action_status(action2.action_id, CurrentAction.RUNNING)

    # Verify pre-stitch state
    action_map = {a.action_id: a for a in node._current_state.action_states}
    assert action_map[action1.action_id].action_status == CurrentAction.FINISHED
    assert action_map[action2.action_id].action_status == CurrentAction.RUNNING

    # Process the stitch order
    node.process_order(stitch_order)

    # After stitch: 3 action_states total
    assert len(node._current_state.action_states) == 3
    action_map = {a.action_id: a for a in node._current_state.action_states}

    # action1 (node1) preserved as FINISHED
    assert action_map[action1.action_id].action_status == CurrentAction.FINISHED

    # action2 (stitch node) preserved as RUNNING — not reset to WAITING
    assert action_map[action2.action_id].action_status == CurrentAction.RUNNING

    # action3 (new node) appended as WAITING
    assert action_map[action3.action_id].action_status == CurrentAction.WAITING


def test_stitch_no_actions_on_stitch_node(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Regression test: stitching when the stitch node has zero actions must not
    wipe existing action_states (guards against the [:-0] == [:0] == [] bug)."""
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    order_id = str(uuid4())

    edge_action = Action(
        action_type="honk", action_id="edge_act_1",
        action_description="Honk on edge", blocking_type="NONE",
    )
    node_c_action = Action(
        action_type="dock", action_id="node_c_act_1",
        action_description="Dock at C", blocking_type="NONE",
    )

    # Base order: A(0) --[e1 w/ edge_action]--> B(2) --[e2 unreleased]--> C(4 unreleased)
    # The horizon keeps has_current_order() True after reaching B.
    base_order = Order(
        header_id=0, timestamp=get_vda5050_ts(), version="1.1.1",
        manufacturer="MANUFACTURER", serial_number="SERIAL_NUMBER",
        order_id=order_id, order_update_id=0,
        nodes=[
            Node(node_id="A", sequence_id=0, released=True,
                 node_position=NodePosition(x=0.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="B", sequence_id=2, released=True,
                 node_position=NodePosition(x=1.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="C", sequence_id=4, released=False,
                 node_position=NodePosition(x=2.0, y=0.0, theta=0.0, map_id="map")),
        ],
        edges=[
            Edge(edge_id="e1", sequence_id=1, released=True,
                 start_node_id="A", end_node_id="B",
                 actions=[edge_action]),
            Edge(edge_id="e2", sequence_id=3, released=False,
                 start_node_id="B", end_node_id="C"),
        ],
    )

    # Stitch order: B(2, no actions) --> C(4, node_c_action)
    stitch_order = Order(
        header_id=0, timestamp=get_vda5050_ts(), version="1.1.1",
        manufacturer="MANUFACTURER", serial_number="SERIAL_NUMBER",
        order_id=order_id, order_update_id=1,
        nodes=[
            Node(node_id="B", sequence_id=2, released=True,
                 node_position=NodePosition(x=1.0, y=0.0, theta=0.0, map_id="map")),
            Node(node_id="C", sequence_id=4, released=True,
                 node_position=NodePosition(x=2.0, y=0.0, theta=0.0, map_id="map"),
                 actions=[node_c_action]),
        ],
        edges=[
            Edge(edge_id="e2", sequence_id=3, released=True,
                 start_node_id="B", end_node_id="C"),
        ],
    )

    # Accept base and navigate to node B
    node.process_order(base_order)
    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)
    node._process_last_edge_node()

    # Mark the edge action as FINISHED
    node._update_action_status(edge_action.action_id, CurrentAction.FINISHED)

    # Pre-stitch: 1 action_state (edge_action, FINISHED)
    assert len(node._current_state.action_states) == 1
    assert node._current_state.action_states[0].action_id == edge_action.action_id
    assert node._current_state.action_states[0].action_status == CurrentAction.FINISHED

    # Process stitch — stitch node B has 0 actions
    node.process_order(stitch_order)

    # Existing action_states must be fully preserved (not wiped)
    # and the new node C's action appended
    assert len(node._current_state.action_states) == 2
    action_map = {a.action_id: a for a in node._current_state.action_states}
    assert action_map[edge_action.action_id].action_status == CurrentAction.FINISHED
    assert action_map[node_c_action.action_id].action_status == CurrentAction.WAITING
