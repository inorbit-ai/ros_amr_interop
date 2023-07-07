#!/usr/bin/env python3

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

# Python dependencies
from enum import Enum
import itertools
import functools

# ROS dependencies / utils
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from vda5050_connector_py.utils import get_vda5050_ros2_topic
from vda5050_connector_py.utils import get_vda5050_ts
from vda5050_connector_py.utils import read_bool_parameter
from vda5050_connector_py.utils import read_double_parameter
from vda5050_connector_py.utils import read_int_parameter
from vda5050_connector_py.utils import read_str_array_parameter
from vda5050_connector_py.utils import read_str_parameter

# ROS msgs / srvs / actions
from vda5050_msgs.msg import Action as VDAAction
from vda5050_msgs.msg import AGVGeometry as VDAAGVGeometry
from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import CurrentAction as VDACurrentAction
from vda5050_msgs.msg import Edge as VDAEdge
from vda5050_msgs.msg import EdgeState as VDAEdgeState
from vda5050_msgs.msg import Envelope2D as VDAEnvelope2D
from vda5050_msgs.msg import Envelope3D as VDAEnvelope3D
from vda5050_msgs.msg import Error as VDAError
from vda5050_msgs.msg import ErrorReference as VDAErrorReference
from vda5050_msgs.msg import Factsheet as VDAFactsheet
from vda5050_msgs.msg import InstantActions as VDAInstantActions
from vda5050_msgs.msg import LoadSet as VDALoadSet
from vda5050_msgs.msg import LoadSpecification as VDALoadSpecification
from vda5050_msgs.msg import MaxArrayLens as VDAMaxArrayLens
from vda5050_msgs.msg import MaxStringLens as VDAMaxStringLens
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodeState as VDANodeState
from vda5050_msgs.msg import Order as VDAOrder
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import PhysicalParameters as VDAPhysicalParameters
from vda5050_msgs.msg import PolygonPoint as VDAPolygonPoint
from vda5050_msgs.msg import ProtocolFeatures as VDAProtocolFeatures
from vda5050_msgs.msg import ProtocolLimits as VDAProtocolLimits
from vda5050_msgs.msg import SafetyState as VDASafetyState
from vda5050_msgs.msg import Timing as VDATiming
from vda5050_msgs.msg import TypeSpecification as VDATypeSpecification
from vda5050_msgs.msg import Visualization as VDAVisualization
from vda5050_msgs.msg import WheelDefinition as VDAWheelDefinition

from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions

from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction

# Constants
DEFAULT_NODE_NAME = "controller"
DEFAULT_NAMESPACE = "vda5050"
DEFAULT_ROBOT_NAME = "robot_1"
DEFAULT_MANUFACTURER_NAME = "robots"
DEFAULT_SERIAL_NUMBER = "robot_1"
DEFAULT_PROTOCOL_VERSION = "2.0.0"

DEFAULT_GET_STATE_SVC_NAME = "adapter/get_state"
DEFAULT_SUPPORTED_ACTIONS_SVC_NAME = "adapter/supported_actions"
DEFAULT_VDA_ACTION_ACT_NAME = "adapter/vda_action"
DEFAULT_NAV_TO_NODE_ACT_NAME = "adapter/nav_to_node"

DEFAULT_STATE_PUB_PERIOD = 5.0  # sec
DEFAULT_CONNECTION_PUB_PERIOD = 15.0  # sec
DEFAULT_VISUALIZATION_PUB_PERIOD = 1.0  # sec
DEFAULT_EXECUTE_ORDER_PERIOD = 0.1  # sec


class ActionErrors(Enum):
    """Action Error types."""

    ACTION_NOT_FOUND = "actionNotFound"
    NO_ORDER_TO_CANCEL = "noOrderToCancel"


class OrderRejectErrors(Enum):
    """Order Processing - Reject Error types."""

    VALIDATION_ERROR = "validationOrder"
    ORDER_UPDATE_ERROR = "orderUpdateError"
    NO_ROUTE_ERROR = "noRouteError"


class OrderAcceptModes(Enum):
    """Order Processing - Accept Modes."""

    NEW = 0
    UPDATE = 1
    STITCH = 2


class VDA5050Controller(Node):
    """ROS2 <> VDA5050 Connector: Controller node."""

    def __init__(self, **kwargs):
        super().__init__(node_name=DEFAULT_NODE_NAME, namespace=DEFAULT_NAMESPACE, **kwargs)

        self.logger = self.get_logger()
        self.on_configure()

        self.logger.info("Node {} has started successfully.".format(DEFAULT_NODE_NAME))

    # Configure

    def on_configure(self):
        """Configure resources needed by this node."""
        self._read_parameters()

        self._cancel_action = None
        self._current_node_actions = []
        self._current_order = VDAOrder(order_id="-1")
        self._current_state = VDAOrderState(
            header_id=0,
            version=self._protocol_version,
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            operating_mode=VDAOrderState.AUTOMATIC,
            safety_state=VDASafetyState(e_stop=VDASafetyState.NONE, field_violation=False),
        )
        self._current_connection = VDAConnection(
            header_id=0,
            version=self._protocol_version,
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
        )
        self._current_visualization = VDAVisualization(
            header_id=0,
            version=self._protocol_version,
            serial_number=self._serial_number,
            manufacturer=self._manufacturer_name,
        )
        self._current_factsheet = VDAFactsheet(
            header_id=0,
            version=self._protocol_version,
            serial_number=self._serial_number,
            manufacturer=self._manufacturer_name,
        )

        # Configure Controller <> Adapter interfaces
        self._configure_action_clients()
        self._configure_service_clients()

        # Configure Master Control <> Controller interfaces
        self._configure_subscriptions()
        self._configure_publishers()
        self._configure_timers()

    def _read_parameters(self):
        """Read and load ROS parameters."""
        # Robot information
        self._robot_name = read_str_parameter(self, "robot_name", DEFAULT_ROBOT_NAME)
        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", DEFAULT_MANUFACTURER_NAME
        )
        self._serial_number = read_str_parameter(self, "serial_number", DEFAULT_SERIAL_NUMBER)
        self._protocol_version = read_str_parameter(
            self, "protocol_version", DEFAULT_PROTOCOL_VERSION
        )
        # ROS interfaces names
        self._get_state_svc_name = read_str_parameter(
            self, "get_state_svc_name", DEFAULT_GET_STATE_SVC_NAME
        )
        self._supported_actions_svc_name = read_str_parameter(
            self, "supported_actions_svc_name", DEFAULT_SUPPORTED_ACTIONS_SVC_NAME
        )
        self._vda_action_act_name = read_str_parameter(
            self, "vda_action_act_name", DEFAULT_VDA_ACTION_ACT_NAME
        )
        self._nav_to_node_act_name = read_str_parameter(
            self, "nav_to_node_act_name", DEFAULT_NAV_TO_NODE_ACT_NAME
        )
        # Timer periods
        self._state_pub_period = read_double_parameter(
            self, "state_pub_period", DEFAULT_STATE_PUB_PERIOD
        )
        self._connection_pub_period = read_double_parameter(
            self, "connection_pub_period", DEFAULT_CONNECTION_PUB_PERIOD
        )
        self._visualization_pub_period = read_double_parameter(
            self, "visualization_pub_period", DEFAULT_VISUALIZATION_PUB_PERIOD
        )
        self._execute_order_period = read_double_parameter(
            self, "execute_order_period", DEFAULT_EXECUTE_ORDER_PERIOD
        )

    # ---- Configure ROS interfaces ----

    def _configure_action_clients(self):
        """Configure Controller <> Adapter ROS Action interfaces."""
        base_interface_name = (
            f"{self.get_namespace()}/{self._manufacturer_name}/{self._robot_name}/"
        )
        # Action client for sending NavigateToNode goals to adapter
        self._navigate_to_node_act_cli = ActionClient(
            node=self,
            action_type=NavigateToNode,
            action_name=base_interface_name + self._nav_to_node_act_name,
        )
        while not self._navigate_to_node_act_cli.wait_for_server(timeout_sec=1.0):
            self.logger.error(
                "NavigateToNode adapter action server not available, waiting again..."
            )
        self._navigate_to_node_goal_handle = None

        # Action client for sending ProcessVDAAction goals to adapter
        self._process_vda_action_act_cli = ActionClient(
            node=self,
            action_type=ProcessVDAAction,
            action_name=base_interface_name + self._vda_action_act_name,
        )
        while not self._process_vda_action_act_cli.wait_for_server(timeout_sec=1.0):
            self.logger.error(
                "ProcessVDAAction adapter action server not available, waiting again..."
            )
        self._process_vda_action_goal_handle_dict = {}

    def _configure_service_clients(self):
        """Configure Controller <> Adapter ROS Service interfaces."""
        base_interface_name = (
            f"{self.get_namespace()}/{self._manufacturer_name}/{self._robot_name}/"
        )
        # Service client to request GetState from the adapter
        self._get_adapter_state_svc_cli = self.create_client(
            srv_type=GetState,
            srv_name=base_interface_name + self._get_state_svc_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        while not self._get_adapter_state_svc_cli.wait_for_service(timeout_sec=1.0):
            self.logger.error("GetState adapter service not available, waiting again...")

        # Service client to request SupportedActions from the adapter
        self._supported_actions_svc_cli = self.create_client(
            srv_type=SupportedActions,
            srv_name=base_interface_name + self._supported_actions_svc_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        while not self._supported_actions_svc_cli.wait_for_service(timeout_sec=1.0):
            self.logger.error("SupportedActions adapter service not available, waiting again...")

    def _configure_subscriptions(self):
        """Configure Master Control to Robot topic msgs."""
        # Process orders coming from the Master Control
        self._process_order_sub = self.create_subscription(
            msg_type=VDAOrder,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
            ),
            callback=self.process_order,
            qos_profile=10,
        )

        # Process instant actions coming from the Master Control
        self._process_instant_actions_sub = self.create_subscription(
            msg_type=VDAInstantActions,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
            ),
            callback=self.process_instant_actions,
            qos_profile=10,
        )

    def _configure_publishers(self):
        """Configure Robot to Master Control topic msgs."""
        # Publish state messages to the Master Control
        self._publish_state_to_mc = self.create_publisher(
            msg_type=VDAOrderState,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="state",
            ),
            qos_profile=10,
        )

        # Publish connection messages to the Master Control
        self._publish_connection_to_mc = self.create_publisher(
            msg_type=VDAConnection,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="connection",
            ),
            qos_profile=10,
        )

        # Publish visualization messages to the Master Control
        self._publish_visualization_to_mc = self.create_publisher(
            msg_type=VDAVisualization,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="visualization",
            ),
            qos_profile=10,
        )

        # Publish visualization messages to the Master Control
        self._publish_factsheet_to_mc = self.create_publisher(
            msg_type=VDAFactsheet,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="factsheet",
            ),
            qos_profile=10,
        )

    def _configure_timers(self):
        """Configure periodic timers for Robot to Master Control msgs."""
        # Publish state msg periodically
        self._state_publisher_timer = self.create_timer(
            timer_period_sec=self._state_pub_period, callback=self._publish_state
        )

        # Publish connection msg periodically
        self._connection_publisher_timer = self.create_timer(
            timer_period_sec=self._connection_pub_period, callback=self._publish_connection
        )

        # Publish visualization msg periodically
        self._visualization_publisher_timer = self.create_timer(
            timer_period_sec=self._visualization_pub_period,
            callback=self._publish_visualization,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Execute order state machine periodically
        self._execute_order_timer = self.create_timer(
            timer_period_sec=self._execute_order_period, callback=self._on_active_order
        )

    # ---- Robot to Master Control publish topics ----

    def _publish_state(self):
        """Publish the current OrderState msg."""
        self.logger.debug(f"Publishing state message {self._current_state}")
        self._current_state.header_id += 1
        self._current_state.timestamp = get_vda5050_ts()
        self._publish_state_to_mc.publish(msg=self._current_state)

    def _publish_connection(self):
        """Publish the current Connection msg."""
        self.logger.debug(f"Publishing connection message {self._current_connection}")
        self._current_connection.header_id += 1
        self._current_connection.timestamp = get_vda5050_ts()
        self._current_connection.connection_state = VDAConnection.ONLINE
        self._publish_connection_to_mc.publish(msg=self._current_connection)

    def _publish_visualization(self):
        """Publish the current Visualization msg."""
        self._current_visualization.header_id += 1
        self._current_visualization.timestamp = get_vda5050_ts()
        self.get_state_from_adapter()  # Need the most recent AGVPosition and velocity
        self._publish_visualization_to_mc.publish(self._current_visualization)

    def _publish_factsheet(self):
        """Publish the current Factsheet msg."""
        self._current_factsheet.header_id += 1
        self._current_factsheet.timestamp = get_vda5050_ts()
        self._publish_factsheet_to_mc.publish(self._current_factsheet)

    # Handle state

    def _update_state(self, partial_state: dict, publish_now: bool = False):
        """
        Update Order State.

        Args:
        ----
            partial_state (dict): Dictionary with field value pairs to update.
            publish_now (bool): True to publish, False otherwise (default False).

        """
        for k, v in partial_state.items():
            setattr(self._current_state, k, v)

        self.logger.debug(f"State updated: '{self._current_state}'.")
        if publish_now:
            self._publish_state()

    # ---- Node, edge and action states helpers ----

    def _get_node_states(self, order: VDAOrder) -> list:
        """
        Create a list of NodeState objects from Order's nodes.

        Args:
        ----
            order (VDAOrder): VDA5050 Order message.

        Returns
        -------
            Sequence[NodeState]: List of NodeState objects.

        """
        return [
            VDANodeState(
                node_id=node.node_id,
                sequence_id=node.sequence_id,
                node_description=node.node_description,
                node_position=node.node_position,
                released=node.released,
            )
            for node in order.nodes
        ]

    def _get_edge_states(self, order: VDAOrder) -> list:
        """
        Create a list of EdgeState objects from Order's edges.

        Args:
        ----
            order (VDAOrder): VDA5050 Order message.

        Returns
        -------
            Sequence[EdgeState]: List of EdgeState objects.

        """
        return [
            VDAEdgeState(
                edge_id=edge.edge_id,
                sequence_id=edge.sequence_id,
                edge_description=edge.edge_description,
                released=edge.released,
                trajectory=edge.trajectory,
            )
            for edge in order.edges
        ]

    def _get_action_states(self, order: VDAOrder) -> list:
        """
        Create a list of CurrentAction objects from Order's base actions.

        All CurrentAction objects are initialized with status WAITING.

        Args:
        ----
            order (VDAOrder): VDA5050 Order message.

        Returns
        -------
            Sequence[CurrentAction]: List of CurrentAction objects.

        """
        # Filter nodes on order base
        base_order_nodes = [node for node in order.nodes if node.released]
        base_order_edges = [edge for edge in order.edges if edge.released]

        # Get actions from nodes and edges
        # ``itertools.chain`` is used for flattening
        node_actions = list(itertools.chain(*[node.actions for node in base_order_nodes]))
        edge_actions = list(itertools.chain(*[edge.actions for edge in base_order_edges]))

        return [
            VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.WAITING,
            )
            for action in node_actions + edge_actions
        ]

    def _delete_action_states(self):
        """Delete instant actions array from current state."""
        self.logger.debug("Deleting action states.")
        self._update_state({"action_states": []})

    def _get_action_status(self, action_id: str) -> str:
        """
        Get the action status on the current state given action's ID.

        Args:
        ----
            action_id (str): Action ID.

        Returns
        -------
            action_status (str): Action status
                ``VDACurrentAction.WAITING``
                ``VDACurrentAction.INITIALIZING``
                ``VDACurrentAction.RUNNING``
                ``VDACurrentAction.PAUSED``
                ``VDACurrentAction.FINISHED``
                ``VDACurrentAction.FAILED``

        """
        action_state = next(
            (
                action
                for action in self._current_state.action_states
                if action.action_id == action_id
            ),
            None,
        )
        return action_state.action_status

    def _update_action_status(self, action_id: str, action_status: str):
        """
        Update action status on the current state given action's ID.

        Args:
        ----
            action_id (str): Action ID
            action_status (str): Action status
                ``VDACurrentAction.WAITING``
                ``VDACurrentAction.INITIALIZING``
                ``VDACurrentAction.RUNNING``
                ``VDACurrentAction.PAUSED``
                ``VDACurrentAction.FINISHED``
                ``VDACurrentAction.FAILED``

        """
        # Get action state from current state
        action_state = next(
            (
                action
                for action in self._current_state.action_states
                if action.action_id == action_id
            ),
            None,
        )
        self.logger.debug(f"Updating action state: {action_state}")

        if not action_state:
            self.logger.error(
                f"Error while processing action state. Couldn't find action with id: '{action_id}'"
            )
            error = VDAError(
                error_type=ActionErrors.ACTION_NOT_FOUND.value,
                description=(
                    f"VDA5050 action with id {action_id} not found to update"
                    f" its state: {action_status}"
                ),
                error_level=VDAError.WARNING,
                error_references=[
                    VDAErrorReference(reference_key="action_id", reference_value=action_id)
                ],
            )
            # Generate error and publish it, then delete it
            current_errors = self._current_state.errors
            self._update_state({"errors": current_errors + [error]}, publish_now=True)
            self._update_state({"errors": current_errors})
            return

        # If found, update action status
        action_state.action_status = action_status

    # ---- Adapter's state ----

    def get_state_from_adapter(self, async_call: bool = False, action: VDAAction = None):
        """
        Request the adapter's state and updates the current state.

        Args:
        ----
            async_call (bool): True to perform an async call to the GetState service.
                False for a sync call (default False).
            action (VDAAction): Optional VDA action required to update its status on
                async calls (default is None).

        """
        if async_call:
            future = self._get_adapter_state_svc_cli.call_async(GetState.Request())
            future.add_done_callback(
                functools.partial(self._get_state_from_adapter_callback, action)
            )
            return

        order_state = self._get_adapter_state_svc_cli.call(GetState.Request())
        self._update_state_from_adapter(order_state)

    def _get_state_from_adapter_callback(self, action: VDAAction, future: Future):
        """
        Receive the async call future result, update the current state and publish it.

        Args:
        ----
            action (VDAAction): VDA action to update its status on async calls.
            future (Future): Service response future.

        """
        if action:
            self._update_action_status(action.action_id, VDACurrentAction.FINISHED)
        self._update_state_from_adapter(future.result())
        self._publish_state()

    def _update_state_from_adapter(self, order_state: VDAOrderState):
        """
        Update current state and visualization msgs based on the adapter's state information.

        Args:
        ----
            order_state (VDAOrderState): Adapter's state to update robot specific information.

        """
        self._current_visualization.agv_position = order_state.state.agv_position
        self._current_visualization.velocity = order_state.state.velocity

        # Remove previous adapter (non-controller) order errors
        current_errors = [
            error
            for error in self._current_state.errors
            if error.error_type
            in [e.value for e in OrderRejectErrors] + [e.value for e in ActionErrors]
        ]
        # Robot specific information filled by the adapter
        self._update_state(
            {
                "agv_position": order_state.state.agv_position,
                "velocity": order_state.state.velocity,
                "loads": order_state.state.loads,
                "driving": order_state.state.driving,
                "paused": order_state.state.paused,
                "distance_since_last_node": order_state.state.distance_since_last_node,
                "battery_state": order_state.state.battery_state,
                "errors": current_errors + order_state.state.errors,
                "informations": order_state.state.informations
            }
        )

    # Instant Actions

    def process_instant_actions(self, instant_actions: VDAInstantActions):
        """
        Process instant actions as per VDA5050 specification.

        This is the instant_actions processing entrypoint. It decides how
        the instant_actions will be processed given current controller state.

        Args:
        ----
            instant_actions (VDAInstantActions): VDA5050 InstantActions message.

        """
        msg_is_valid, error = self.instant_action_msg_is_valid(instant_actions)
        if not msg_is_valid:
            # Generate error and publish it, then delete it
            current_errors = self._current_state.errors
            self._update_state({"errors": current_errors + [error]}, publish_now=True)
            self._update_state({"errors": current_errors})
            return

        header_id = instant_actions.header_id
        self.logger.info(f"Received instant_actions msg with id: '{header_id}'")

        for action in instant_actions.actions:
            self.logger.info(
                f"Processing action '{action.action_id}' of type '{action.action_type}'"
            )

            # Add action to action_states
            action_state = VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.WAITING,
            )
            self._update_state(
                {"action_states": self._current_state.action_states + [action_state]}
            )

            if action.action_type == "cancelOrder":
                self._cancel_action = action
                continue
            elif action.action_type == "stateRequest":
                self._update_action_status(action.action_id, VDACurrentAction.RUNNING)
                self.get_state_from_adapter(async_call=True, action=action)
                continue
            elif action.action_type == "factsheetRequest":
                # Populate the current factsheet msg reading and requesting its info
                self._populate_factsheet()
                self._update_action_status(action.action_id, VDACurrentAction.FINISHED)
                self._publish_factsheet()
                continue

            self.send_adapter_process_vda_action(action)

    def instant_action_msg_is_valid(self, instant_actions: VDAInstantActions):
        """
        Process if a given instant action msg is valid. If not, a proper VDAError is generated.

        Args:
        ----
            instant_actions (VDAInstantActions): Instant action msg to validate.

        Returns
        -------
            True if msg is valid, False otherwise.
            VDAError if msg in invalid.

        """
        # TODO: Proper implementation
        return True, VDAError()

    # ---- Process VDA action send goals ----

    def send_adapter_process_vda_action(self, action: VDAAction):
        """
        Send a VDA action goal to the VDA5050 adapter.

        If the action is being processed (not in WAITING), the function exits.

        Args:
        ----
            action (VDAAction): Action to be executed.

        """
        if self._get_action_status(action.action_id) != VDACurrentAction.WAITING:
            return

        goal_msg = ProcessVDAAction.Goal()
        goal_msg.action = action

        self._process_vda_action_act_cli.wait_for_server()

        # Send goal to action server
        self.logger.info(
            f"VDA Action '{action.action_id}' of type '{action.action_type}' sent to adapter."
        )
        self._process_vda_action_goal_future = self._process_vda_action_act_cli.send_goal_async(
            goal=goal_msg, feedback_callback=self._process_vda_action_feedback_callback
        )

        self._update_action_status(action.action_id, VDACurrentAction.INITIALIZING)
        # Register callback to be executed when the goal is accepted
        self._process_vda_action_goal_future.add_done_callback(
            functools.partial(self._process_vda_action_goal_response_callback, action)
        )

    def _process_vda_action_goal_response_callback(self, action: VDAAction, future: Future):
        """
        Response callback function for process VDA actions goal request.

        Args:
        ----
            action (VDAAction): VDA Action sent as goal.
            future (Future): Action response future.

        """
        _goal_handle = future.result()
        if not _goal_handle.accepted:
            self._update_action_status(
                action_id=action.action_id, action_status=VDACurrentAction.FAILED
            )
            self.logger.info(
                f"VDA Action '{action.action_id}' of type '{action.action_type}'"
                " rejected by the adapter"
            )
            return

        self._process_vda_action_goal_handle_dict[action.action_id] = _goal_handle
        self.logger.info(
            f"VDA Action '{action.action_id}' of type '{action.action_type}'"
            " accepted by the adapter"
        )

        _get_result_future = _goal_handle.get_result_async()
        _get_result_future.add_done_callback(self._process_vda_action_result_callback)

    def _process_vda_action_feedback_callback(self, feedback_msg: ProcessVDAAction.Feedback):
        """
        Feedback callback function for process VDA actions goal request.

        Args:
        ----
            feedback_msg (ProcessVDAAction.Feedback): ProcessVDAAction Action client feedback
            message.

        """
        current_action = feedback_msg.feedback.current_action
        self._update_action_status(current_action.action_id, current_action.action_status)

    def _process_vda_action_result_callback(self, future: Future):
        """
        Process VDA actions goal request.

        Args:
        ----
            future (Future): Action result future.

        """
        action_result: ProcessVDAAction.Result = future.result().result
        current_action: VDACurrentAction = action_result.result
        self._process_vda_action_goal_handle_dict.pop(current_action.action_id)
        self._update_action_status(current_action.action_id, current_action.action_status)
        self.logger.info(f"VDA Action finished. Result: {current_action}")

    # Order

    def process_order(self, order: VDAOrder):
        """
        Process order as per VDA5050 specification.

        This is the order processing entrypoint. It decides how
        the order will be processed given current controller state.

        https://github.com/VDA5050/VDA5050/blob/development/assets/Figure8.png

        Args:
        ----
            order (VDAOrder): VDA5050 Order message.

        """
        self.logger.info(f"Received order with ID: '{order.order_id}'")

        # Validate order msg
        msg_is_valid, error = self.order_msg_is_valid(order)
        if not msg_is_valid:
            self._reject_order(order, OrderRejectErrors.VALIDATION_ERROR, error.error_description)
            return

        accept_mode = None
        reject_error = None
        error_description = ""

        has_current_order = self._has_current_order()
        if order.order_id != self._current_order.order_id:
            # New order graph (Different order_id)
            if not self._first_node_in_deviation_range(order) or has_current_order:
                # Reject order if already have a running order or if first node not in deviation
                # range
                error_description = (
                    "There is an active running order."
                    if has_current_order
                    else "First node not in deviation range"
                )
                reject_error = (
                    OrderRejectErrors.ORDER_UPDATE_ERROR
                    if has_current_order
                    else OrderRejectErrors.NO_ROUTE_ERROR
                )
            else:
                # Accept new order
                accept_mode = OrderAcceptModes.NEW
        else:
            # Same order graph (Same order_id)
            update_id_diff = order.order_update_id - self._current_order.order_update_id
            match_last_new_base_nodes = self._match_base_nodes(order)

            if update_id_diff == 0:
                # Same update id, discard the msg
                self.logger.info(f"Order [{order.order_id}] discarded. Same order update id.")
                return
            elif update_id_diff < 0 or not match_last_new_base_nodes:
                # Reject if update id is lower or if last and new base nodes doesn't match
                error_description = (
                    f"New base start node [{order.nodes[0].node_id}] doesn't match with old base"
                    f" last node [{self._current_order.nodes[0].node_id}]"
                    if not match_last_new_base_nodes
                    else (
                        f"New update id {order.order_update_id} lower than old update id"
                        f" {self._current_order.order_update_id}"
                    )
                )
                reject_error = OrderRejectErrors.ORDER_UPDATE_ERROR
            else:
                # Accept update order
                accept_mode = (
                    OrderAcceptModes.STITCH if has_current_order else OrderAcceptModes.UPDATE
                )

        if reject_error:
            self._reject_order(order=order, error=reject_error, description=error_description)
        else:
            self._accept_order(order=order, mode=accept_mode)

    def order_msg_is_valid(self, order: VDAOrder):
        """
        Check if a given order msg is valid. If not, a proper VDAError is generated.

        Args
        ----
            order (VDAOrder): Order msg to validate

        Returns
        -------
            True if msg is valid, False otherwise, VDAError if msg in invalid.

        """
        # TODO: Proper implementation
        return True, VDAError()

    def _has_current_order(self) -> bool:
        """
        Validate if there is an active order.

        ``actionStates running && nodeStates not empty && edgeStates not empty``

        Returns
        -------
            True if there is an active order, False otherwise.

        """
        # Dont take care of cancelOrder action when evaluating if there is an active order
        action_id_cancel = -1
        if self._canceling_order():
            action_id_cancel = self._cancel_action.action_id
        has_running_actions = any(
            [
                action_state.action_status
                not in [VDACurrentAction.FINISHED, VDACurrentAction.FAILED]
                for action_state in self._current_state.action_states
                if action_state.action_id != action_id_cancel
            ]
        )
        # If there are no actions, but there are node / edge states, there is an active order
        return has_running_actions or (
            len(self._current_state.node_states) and len(self._current_state.edge_states)
        )

    def _first_node_in_deviation_range(self, order: VDAOrder) -> bool:
        """
        Validate if first node is in deviation range.

        Args:
        ----
            order (VDAOrder): New order to validate first node.

        Return:
        ------
            True if first node is in deviation range, False otherwise.

        """
        # TODO: Proper implementation
        # first_node = order.nodes[0]
        # node_position = first_node.node_position
        # allowed_deviation_xy = node_position.allowed_deviation_x_y
        # allowed_deviation_theta = node_position.allowed_deviation_theta

        # HACK: deviation range is ignored ATM. Need to improve function to calculate radius and
        # deviation of theta
        return True

    def _match_base_nodes(self, order: VDAOrder):
        """
        Validate if the last node from the old base matches the first node on the new base.

        Args
        ----
            order (VDAOrder): Incoming order to validate if base nodes match.

        Returns
        -------
            True if last <> first base nodes match, False otherwise.

        """
        # TODO: Proper implementation
        return True

    def _accept_order(self, order: VDAOrder, mode: OrderAcceptModes):
        """
        Process the accepting of an order, updating if its new, update or stitch.

        https://github.com/VDA5050/VDA5050/blob/development/assets/Figure8.png

        Args:
        ----
            order (VDAOrder): Valid order to process.
            mode (OrderAcceptModes): There are 3 different ways of accepting an order:
                NEW: the order is different from current order i.e. order_id != current_order_id.
                UPDATE: when the order's horizon is updated.
                STITCH: order's base is extended. It may include a new horizon.

        """
        self.logger.info(
            f"Order '{order.order_id}' with update id '{order.order_update_id}' accepted!"
        )

        if mode == OrderAcceptModes.STITCH:
            # Accept STITCH order
            self.logger.debug("Clearing horizon and appending new graph to the current base.")

            # Clear horizon on current state
            # Avoid copying the stitching node twice
            self._current_state.node_states = [
                node_state
                for node_state in self._current_state.node_states
                if node_state.released and node_state.sequence_id != order.nodes[0].sequence_id
            ]
            self._current_state.edge_states = [
                edge_state for edge_state in self._current_state.edge_states if edge_state.released
            ]

            # Clear horizon on current order and append new nodes / edges
            # Avoid copying the stitching node twice
            base_order_nodes = [
                node
                for node in self._current_order.nodes
                if node.released and node.sequence_id != order.nodes[0].sequence_id
            ]
            base_order_edges = [edge for edge in self._current_order.edges if edge.released]

            self._current_order.order_update_id = order.order_update_id
            self._current_order.zone_set_id = order.zone_set_id

            self._current_order.nodes = base_order_nodes + order.nodes
            self._current_order.edges = base_order_edges + order.edges

        else:
            # Accept NEW / UPDATE order
            if mode == OrderAcceptModes.NEW:
                # On new orders, delete action states
                self._delete_action_states()

            self._current_order = order

        # Remove previous order errors when accepting a new order
        errors = [
            error
            for error in self._current_state.errors
            if error.error_type not in [e.value for e in OrderRejectErrors]
        ]

        # Update state
        # Only on stitching updates the node and edges base states are kept
        self._update_state(
            {
                "order_id": order.order_id,
                "order_update_id": order.order_update_id,
                "errors": errors,
                "node_states": (mode == OrderAcceptModes.STITCH) * self._current_state.node_states
                + self._get_node_states(order),
                "edge_states": (mode == OrderAcceptModes.STITCH) * self._current_state.edge_states
                + self._get_edge_states(order),
                "action_states": self._current_state.action_states
                + self._get_action_states(order),
                "new_base_request": False,
            }
        )

        # Send the robot to navigate on new / update orders
        if mode != OrderAcceptModes.STITCH:
            # Note: the standard assumes the robot is at the first node of the order.
            # Otherwise, the order gets rejected and this method is not called.
            self._process_node(self._current_order.nodes[0])

    def _reject_order(self, order: VDAOrder, error: OrderRejectErrors, description: str = ""):
        """
        Reject order.

        Args:
        ----
            order (VDAOrder): Order sent
            error (OrderRejectErrors): Errors
                VALIDATION_ERROR
                ORDER_UPDATE_ERROR
                NO_ROUTE_ERROR
            description (str): Order rejection description

        """
        error_references = []
        if error == OrderRejectErrors.ORDER_UPDATE_ERROR:
            # On orderUpdateError send orderUpdateId and orderId as reference

            # TODO: Question: camelCase or snakeCase (order_id or orderId)
            error_references.append(
                VDAErrorReference(reference_key="order_id", reference_value=order.order_id)
            )
            error_references.append(
                VDAErrorReference(
                    reference_key="order_update_id", reference_value=str(order.order_update_id)
                )
            )
        elif error == OrderRejectErrors.NO_ROUTE_ERROR:
            # On noRouteError send 1st node as reference
            error_references.append(
                VDAErrorReference(reference_key="node_id", reference_value=order.nodes[0].node_id)
            )

        order_error = VDAError()
        order_error.error_type = error.value
        order_error.error_description = (
            f"VDA5050 order {order.order_id} with update ID"
            f" {order.order_update_id} rejected [{description}]."
        )
        order_error.error_level = VDAError.WARNING
        order_error.error_references = error_references

        # Update error array and publish it
        # These reject orders will be delete them when a valid order is accepted
        self.logger.warn(f"Order rejected: {order_error.error_description}")
        self._update_state(
            {"errors": self._current_state.errors + [order_error]}, publish_now=True
        )

    def _cancel_order(self):
        """
        Process cancel order as per VDA5050 specification.

        https://github.com/VDA5050/VDA5050/blob/main/assets/Figure9.png

        This cancel order action will stop running NavigateToNode goals and ProcessVDAAction goals,
        delete edge_states and node_states, and set to failed waiting action_states.
        When the cancel order action finishes, it publishes the updated state.
        """
        if not self._has_current_order():
            self.logger.error(
                "cancelOrder action request failed. There is no active order running."
            )
            self._update_action_status(self._cancel_action.action_id, VDACurrentAction.FAILED)
            # The AGV must report a “noOrderToCancel” error with the errorLevel set to warning.
            # The actionId of the instantAction must be passed as an errorReference.
            error = VDAError()
            error.error_type = ActionErrors.NO_ORDER_TO_CANCEL.value
            error.error_description = "There is no active order running."
            error.error_level = VDAError.WARNING
            error.error_references = [
                VDAErrorReference(
                    reference_key="action_id", reference_value=self._cancel_action.action_id
                )
            ]

            # Generate error and publish it, then delete it
            current_errors = self._current_state.errors
            self._update_state({"errors": current_errors + [error]}, publish_now=True)
            self._update_state({"errors": current_errors})
            self._cancel_action = None
            return

        # Set cancelOrder action state to running
        self._update_action_status(self._cancel_action.action_id, VDACurrentAction.RUNNING)

        # Set waiting actions to failed
        for action_state in self._current_state.action_states:
            if action_state.action_status == VDACurrentAction.WAITING:
                self._update_action_status(action_state.action_id, VDACurrentAction.FAILED)

        # Interrupt any running action
        vda_action_goal_handles = self._process_vda_action_goal_handle_dict.values()
        if len(vda_action_goal_handles) > 0:
            for goal_handle in vda_action_goal_handles:
                goal_handle.cancel_goal_async()
            return

        # Interrupt any running navigation goal
        if self._is_navigation_active():
            self._navigate_to_node_goal_handle.cancel_goal_async()
            return

        # Once all the VDA actions and navigation goal requests have finished,
        # the cancel order will be mark as finished

        # Delete remaining node / edge states
        self._update_state({"new_base_request": False, "node_states": [], "edge_states": []})
        self._update_action_status(self._cancel_action.action_id, VDACurrentAction.FINISHED)
        self._current_order = VDAOrder(order_id="-1")
        self._cancel_action = None
        self._current_node_actions = []

        self.logger.info("Finished executing cancelOrder.")

    def _canceling_order(self) -> bool:
        """
        Indicate if there is a request to cancel an order.

        Returns
        -------
            True if a cancel order action was sent, False otherwise.

        """
        return self._cancel_action is not None

    # ---- Process order nodes / edges / actions ----

    def _on_active_order(self):
        """
        Execute order state machine.

        It first checks if there is an instruction to cancel the order.
        It checks if there is a current order. If so, it runs first any current node actions.
        If there are no more actions to execute, it sends it to navigate to the next edge / node,
        if it's not navigating already.
        """
        if self._canceling_order():
            self._cancel_order()
            return

        if not self._has_current_order():
            return

        if len(self._current_node_actions) > 0:
            self._execute_node_actions()
            return

        if not self._is_navigation_active():
            self._process_next_edge()

    def _process_node(self, node: VDANode):
        """
        Process VDA5050 order's node when the robot reaches a given goal.

        Args:
        ----
            node (VDANode): Order's node.

        """
        # Remove node from node_states and update controller state
        self.logger.info(f"Arrived to node: {node.node_id}")
        self._update_state(
            {
                "node_states": [
                    node_state
                    for node_state in self._current_state.node_states
                    if node_state.sequence_id != node.sequence_id
                ],
                "last_node_id": node.node_id,
                "last_node_sequence_id": node.sequence_id,
            }
        )

        self._current_node_actions = node.actions
        self.logger.info(
            f"Executing {len(self._current_node_actions)} actions of node: {node.node_id}."
        )

        if len(self._current_state.node_states) == 0:
            self.logger.info(
                f"Processing last order's node. Order {self._current_order.order_id} finished."
            )

    def _execute_node_actions(self):
        """
        Process the node actions as per VDA5050 specification (Figure 15).

        https://github.com/VDA5050/VDA5050/blob/main/assets/Figure15.png

        The algorithm process first HARD actions, then SOFT and NONE.
        This same function filters any finished / failed action from the current node actions.

        """
        # Remove finished / failed actions
        self._current_node_actions = [
            action
            for action in self._current_node_actions
            if self._get_action_status(action.action_id)
            not in [VDACurrentAction.FINISHED, VDACurrentAction.FAILED]
        ]

        def get_list_type(type):
            return [
                action for action in self._current_node_actions if action.blocking_type == type
            ]

        # Add a list for each type in sequence: HARD, SOFT, NONE
        execution_list = {
            "hard": get_list_type(VDAAction.HARD),
            "soft": get_list_type(VDAAction.SOFT),
            "none": get_list_type(VDAAction.NONE),
        }

        # Execute serial (hard) actions
        if len(execution_list["hard"]) > 0:
            self.send_adapter_process_vda_action(execution_list["hard"][0])
            return

        # Execute parallel (soft and none) actions
        for action in execution_list["soft"] + execution_list["none"]:
            self.send_adapter_process_vda_action(action)

    def _process_next_edge(self):
        """Process VDA5050 order's edge."""
        # Get next edge to be processed by looking for an edge
        # with sequence_id equal to last_node_sequence_id + 1.
        try:
            next_edge = next(
                edge
                for edge in self._current_order.edges
                if edge.sequence_id == self._current_state.last_node_sequence_id + 1
            )
        except StopIteration:
            # This only happens when there is no order or it has finished,
            # but there is an active instant action running.
            # In this case, just exit.
            return

        if not next_edge.released:
            if not self._current_state.new_base_request:
                self.logger.warn("Next edge is part of the horizon. Stopping traversing of nodes.")
                self._update_state({"new_base_request": True}, publish_now=True)
            return

        # After a released edge there is always a released node.
        # Otherwise, the order gets rejected and this method is never called.
        next_node = next(
            node
            for node in self._current_order.nodes
            if node.sequence_id == self._current_state.last_node_sequence_id + 2
        )
        self.logger.info(f"Processing node: {next_node}")

        self.send_adapter_navigate_to_node(edge=next_edge, node=next_node)

    # ---- Navigate to node: send goals ----

    def send_adapter_navigate_to_node(self, edge: VDAEdge, node: VDANode):
        """
        Send navigation goal to the VDA5050 adapter.

        Args:
        ----
            edge (VDAEdge): Order's edge to traverse.
            node (VDANode): Order edge ending node.

        """
        # Create goal message with edge and node parameters
        goal_msg = NavigateToNode.Goal()
        goal_msg.edge = edge
        goal_msg.node = node

        # Wait for NavigateToNode action server to be ready
        self._navigate_to_node_act_cli.wait_for_server()

        # Send goal to action server
        self.logger.info("Navigate to node goal request sent.")
        _send_goal_future = self._navigate_to_node_act_cli.send_goal_async(goal_msg)

        # Register callback to be executed when the goal is accepted
        _send_goal_future.add_done_callback(self._navigate_to_node_goal_response_callback)

    def _navigate_to_node_goal_response_callback(self, future: Future):
        """
        Response callback function for navigate to node goal request.

        Args:
        ----
            future (Future): Action response future.

        """
        self._navigate_to_node_goal_handle = future.result()
        if not self._navigate_to_node_goal_handle.accepted:
            self.logger.error("Navigate to node goal request rejected by adapter. Trying again.")
            self._navigate_to_node_goal_handle = None
            return

        self.logger.info("Navigate to node goal request accepted by adapter.")
        # TODO: Execute edge actions

        # Add callback to handle action result
        _get_result_future = self._navigate_to_node_goal_handle.get_result_async()
        _get_result_future.add_done_callback(self._navigate_to_node_result_callback)

    def _navigate_to_node_result_callback(self, future: Future):
        """
        Process VDA actions goal request.

        This callback is in charge of triggering next order's node execution.

        Args:
        ----
            future (Future): Action result future.

        """
        # TODO: Check when the goal fails
        self._navigate_to_node_goal_handle = None

        # When the order is cancelled, this callback should avoid continuing its logic
        if self._canceling_order():
            return

        last_edge = next(
            edge
            for edge in self._current_order.edges
            if edge.sequence_id == self._current_state.last_node_sequence_id + 1
        )

        last_node = next(
            node
            for node in self._current_order.nodes
            if node.sequence_id == self._current_state.last_node_sequence_id + 2
        )

        # Remove edge from edge states and update controller state
        self._update_state(
            {
                "edge_states": [
                    edge_state
                    for edge_state in self._current_state.edge_states
                    if edge_state.edge_id != last_edge.edge_id
                    and edge_state.sequence_id != last_edge.sequence_id
                ]
            }
        )
        self._process_node(node=last_node)

    def _is_navigation_active(self) -> bool:
        """
        Indicate if the robot was commanded to navigate to a node.

        Returns
        -------
            True if robot is navigating to node, False otherwise.

        """
        return self._navigate_to_node_goal_handle is not None

    # Factsheet

    def _populate_factsheet(self):
        """Populate the factsheet (fs) msg calling the different functions for each field."""
        # If the msg has been already published, populate it.
        # NOTE: Populate one time (data doesn't change).
        if self._current_factsheet.header_id != 0:
            return

        self._current_factsheet.type_specification = self._get_fs_type_specification()
        self._current_factsheet.physical_parameters = self._get_fs_physical_parameters()
        self._current_factsheet.protocol_limits = self._get_fs_protocol_limits()
        self._current_factsheet.protocol_features = self._get_fs_protocol_features()
        self._current_factsheet.agv_geometry = self._get_fs_agv_geometry()
        self._current_factsheet.load_specification = self._get_fs_load_specification()
        self._current_factsheet.localization_parameters = self._get_fs_localization_parameters()

    def _get_fs_type_specification(self) -> VDATypeSpecification:
        """
        Populate the type specification msg for the factsheet msg.

        Read these fields from parameters.
        """
        type_specification = VDATypeSpecification()

        type_specification.series_name = read_str_parameter(
            self, "factsheet.type_specification.series_name", "robot_number"
        )
        type_specification.series_description = read_str_parameter(
            self, "factsheet.type_specification.series_description", ""
        )
        type_specification.agv_kinematic = read_str_parameter(
            self, "factsheet.type_specification.agv_kinematic", VDATypeSpecification.OMNI
        )
        type_specification.agv_class = read_str_parameter(
            self, "factsheet.type_specification.agv_class", VDATypeSpecification.CARRIER
        )
        type_specification.max_load_mass = read_double_parameter(
            self, "factsheet.type_specification.max_load_mass", 0.0
        )
        type_specification.localization_types = read_str_array_parameter(
            self,
            "factsheet.type_specification.localization_types",
            [VDATypeSpecification.REFLECTOR],
        )
        type_specification.navigation_types = read_str_array_parameter(
            self,
            "factsheet.type_specification.navigation_types",
            [VDATypeSpecification.AUTONOMOUS],
        )

        return type_specification

    def _get_fs_physical_parameters(self) -> VDAPhysicalParameters:
        """Populate the physical parameters msg for the factsheet msg."""
        physical_parameters = VDAPhysicalParameters()
        parameters_physical = {
            "speed_min": 0.0,
            "speed_max": 0.0,
            "acceleration_max": 0.0,
            "deceleration_max": 0.0,
            "height_min": 0.0,
            "height_max": 0.0,
            "width": 0.0,
            "length": 0.0,
        }

        for key, val in parameters_physical.items():
            value = read_double_parameter(self, "factsheet.physical_parameters." + key, val)
            setattr(physical_parameters, key, value)

        return physical_parameters

    def _get_fs_protocol_limits(self) -> VDAProtocolLimits:
        """Populate the protocol limits msg for the factsheet msg."""
        protocol_limits = VDAProtocolLimits()

        def _read_string_lens() -> VDAMaxStringLens:
            """Populate the max string lens msg for the protocol limits msg."""
            string_lens = VDAMaxStringLens()
            parameters_string_lens = {
                "msg_len": 0,
                "topic_serial_len": 0,
                "topic_elem_len": 0,
                "id_len": 0,
                "enum_len": 0,
                "load_id_len": 0,
            }

            for key, val in parameters_string_lens.items():
                value = read_int_parameter(
                    self, "factsheet.protocol_limits.max_string_lens." + key, val
                )
                setattr(string_lens, key, value)

            string_lens.id_numerical_only = read_bool_parameter(
                self, "factsheet.protocol_limits.max_string_lens.id_numerical_only", False
            )
            return string_lens

        def _read_array_lens() -> VDAMaxArrayLens:
            """Populate the max array lens msg for the protocol limits msg."""
            array_lens = VDAMaxArrayLens()
            parameters_array_lens = {
                "order_nodes": 0,
                "order_edges": 0,
                "node_actions": 0,
                "edge_actions": 0,
                "actions_parameters": 0,
                "instant_actions": 0,
                "trajectory_knot_vector": 0,
                "trajectory_control_points": 0,
                "state_node_states": 0,
                "state_edge_states": 0,
                "state_loads": 0,
                "state_action_states": 0,
                "state_errors": 0,
                "state_information": 0,
                "error_references": 0,
                "info_references": 0,
            }

            for key, val in parameters_array_lens.items():
                value = read_int_parameter(
                    self, "factsheet.protocol_limits.max_array_lens." + key, val
                )
                setattr(array_lens, key, value)
            return array_lens

        def _read_timing() -> VDATiming:
            """Populate the timing msg for the protocol limits msg."""
            timing = VDATiming()
            parameters_timing = {
                "min_order_interval": 0.0,
                "min_state_interval": 0.0,
                "default_state_interval": 0.0,
                "visualization_interval": 0.0,
            }

            for key, val in parameters_timing.items():
                value = read_double_parameter(self, "factsheet.protocol_limits.timing." + key, val)
                setattr(timing, key, value)
            return timing

        protocol_limits.max_string_lens = _read_string_lens()
        protocol_limits.max_array_lens = _read_array_lens()
        protocol_limits.timing = _read_timing()

        return protocol_limits

    def _get_fs_protocol_features(self) -> VDAProtocolFeatures:
        """
        Populate the protocol features msg for the factsheet msg.

        This methods requests the AGV actions from the SupportedActions adapter service.
        """
        protocol_features = VDAProtocolFeatures()

        # Get supported actions from adapter
        self._supported_actions_svc_cli.wait_for_service()
        response = self._supported_actions_svc_cli.call(SupportedActions.Request())
        protocol_features.agv_actions = response.agv_actions

        return protocol_features

    def _get_fs_agv_geometry(self) -> VDAAGVGeometry:
        """Populate the agv geometry msg for the factsheet msg."""
        agv_geometry = VDAAGVGeometry()

        def _read_wheel_definitions() -> list:
            """
            Populate the wheel definitions msg for the agv geometry msg.

            It reads first a wheel_definitions.ids str array with unique identifiers.
            Then, for each id, the proper wheel definition keys are read.
            """
            wheel_definitions = []
            base_key = "factsheet.agv_geometry.wheel_definitions."
            wheel_definitions_ids = read_str_array_parameter(self, base_key + "ids", [])
            for wheel_id in wheel_definitions_ids:
                wheel_key = base_key + "wheel_" + wheel_id + "."
                wheel_definition = VDAWheelDefinition()

                wheel_definition.type = read_str_parameter(
                    self, wheel_key + "type", VDAWheelDefinition.DRIVE
                )
                wheel_definition.is_active_driven = read_bool_parameter(
                    self, wheel_key + "is_active_driven", True
                )
                wheel_definition.is_active_steered = read_bool_parameter(
                    self, wheel_key + "is_active_steered", False
                )

                # Position
                wheel_definition.position.x = read_double_parameter(
                    self, wheel_key + "position.x", 0.0
                )
                wheel_definition.position.y = read_double_parameter(
                    self, wheel_key + "position.y", 0.0
                )
                wheel_definition.position.theta = read_double_parameter(
                    self, wheel_key + "position.theta", 0.0
                )

                wheel_definition.diameter = read_double_parameter(
                    self, wheel_key + "diameter", 0.0
                )
                wheel_definition.width = read_double_parameter(self, wheel_key + "width", 0.0)
                wheel_definition.center_displacement = read_double_parameter(
                    self, wheel_key + "center_displacement", 0.0
                )
                wheel_definition.constraints = read_str_parameter(
                    self, wheel_key + "constraints", ""
                )
                wheel_definitions.append(wheel_definition)
            return wheel_definitions

        def _read_envelopes2d() -> list:
            """
            Populate the enveloped2d list for the agv geometry msg.

            It reads first a envelopes2d.ids str array with unique identifiers
            Then, for each id, the proper envelopes2d keys are read.

            For polygon_points, the method expects to read a string array of "x, y" values.
            E.G polygon_points: ["1,2", "5,6"]
            """
            envelopes2d = []
            base_key = "factsheet.agv_geometry.envelopes2d."
            envelop2d_ids = read_str_array_parameter(self, base_key + "ids", [])
            for envelop_id in envelop2d_ids:
                envelop2d_key = base_key + "envelop2d_" + envelop_id + "."
                envelop2d = VDAEnvelope2D()

                envelop2d.set = read_str_parameter(self, envelop2d_key + "set", "")
                envelop2d.description = read_str_parameter(self, envelop2d_key + "description", "")
                polygon_points = read_str_array_parameter(
                    self, envelop2d_key + "polygon_points", []
                )
                for polygon_pair in polygon_points:
                    try:
                        polygon_point = VDAPolygonPoint()

                        values = polygon_pair.split(",")
                        polygon_point.x = float(values[0])
                        polygon_point.y = float(values[1])

                        envelop2d.polygon_points.append(polygon_point)
                    except Exception:
                        error_msg = f"Error reading {envelop2d_key}polygon_points pair values."
                        error_msg += (
                            "The format of this field should be ...polygon_points: "
                            "['x1,y1', 'x2,y2']"
                        )
                        self.logger.error(error_msg)
                        continue

                envelopes2d.append(envelop2d)
            return envelopes2d

        def _read_envelopes3d() -> list:
            """
            Populate the enveloped3d list for the agv geometry msg.

            It reads first a envelopes3d.ids str array with unique identifiers.
            Then, for each id, the proper envelopes3d keys are read.
            """
            envelopes3d = []
            base_key = "factsheet.agv_geometry.envelopes3d."
            envelop3d_ids = read_str_array_parameter(self, base_key + "ids", [])
            for envelop_id in envelop3d_ids:
                envelop3d_key = base_key + "envelop3d_" + envelop_id + "."
                envelop3d = VDAEnvelope3D()

                parameters_envelop3d = {
                    "set": "",
                    "format": "",
                    "data": "",
                    "url": "",
                    "description": "",
                }

                for key, val in parameters_envelop3d.items():
                    value = read_str_parameter(self, envelop3d_key + key, val)
                    setattr(envelop3d, key, value)

                envelopes3d.append(envelop3d)
            return envelopes3d

        agv_geometry.wheel_definitions = _read_wheel_definitions()
        agv_geometry.envelopes2d = _read_envelopes2d()
        agv_geometry.envelopes3d = _read_envelopes3d()

        return agv_geometry

    def _get_fs_load_specification(self) -> VDALoadSpecification:
        """Populate the load specification msg for the factsheet msg."""
        load_specification = VDALoadSpecification()

        def _read_load_sets() -> list:
            """
            Populate the load_sets list for the load specification msg.

            It reads first a load_sets.ids str array with unique identifiers
            Then, for each id, the proper load_sets keys are read.
            """
            load_sets = []
            base_key = "factsheet.load_specification.load_sets."
            load_sets_ids = read_str_array_parameter(self, base_key + "ids", [])

            for load_sets_id in load_sets_ids:
                load_set = VDALoadSet()
                load_set_key = base_key + "set_" + load_sets_id + "."

                parameters_load_set = {
                    "set_name": "",
                    "load_type": "",
                    "description": "",
                    "max_weight": 0.0,
                    "min_loadhandling_height": 0.0,
                    "max_loadhandling_height": 0.0,
                    "min_loadhandling_depth": 0.0,
                    "max_loadhandling_depth": 0.0,
                    "min_loadhandling_tilt": 0.0,
                    "max_loadhandling_tilt": 0.0,
                    "agv_speed_limit": 0.0,
                    "agv_acceleration_limit": 0.0,
                    "agv_deceleration_limit": 0.0,
                    "pick_time": 0.0,
                    "drop_time": 0.0,
                }

                for key, val in parameters_load_set.items():
                    if type(val) == str:
                        value = read_str_parameter(self, load_set_key + key, val)
                    else:
                        value = read_double_parameter(self, load_set_key + key, val)
                    setattr(load_set, key, value)

                load_set.load_positions = read_str_array_parameter(
                    self, load_set_key + "load_positions", []
                )

                # Load dimensions
                load_set.load_dimensions.length = read_double_parameter(
                    self, load_set_key + "load_dimensions.length", 0.0
                )
                load_set.load_dimensions.width = read_double_parameter(
                    self, load_set_key + "load_dimensions.width", 0.0
                )
                load_set.load_dimensions.height = read_double_parameter(
                    self, load_set_key + "load_dimensions.height", 0.0
                )

                # Bounding Box Reference
                load_set.bounding_box_reference.x = read_double_parameter(
                    self, load_set_key + "bounding_box_reference.x", 0.0
                )
                load_set.bounding_box_reference.y = read_double_parameter(
                    self, load_set_key + "bounding_box_reference.y", 0.0
                )
                load_set.bounding_box_reference.z = read_double_parameter(
                    self, load_set_key + "bounding_box_reference.z", 0.0
                )
                load_set.bounding_box_reference.theta = read_double_parameter(
                    self, load_set_key + "bounding_box_reference.theta", 0.0
                )

                load_sets.append(load_set)

            return load_sets

        load_specification.load_positions = read_str_array_parameter(
            self, "factsheet.load_specification.load_positions", []
        )
        load_specification.load_sets = _read_load_sets()

        return load_specification

    def _get_fs_localization_parameters(self) -> int:
        """Populate the localization parameter msg for the factsheet msg."""
        # TODO: This seems to be not defined in the VDA5050 schema.
        return 0
