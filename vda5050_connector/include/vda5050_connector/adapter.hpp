// BSD 3-Clause License
//
// Copyright (c) 2022 InOrbit, Inc.
// Copyright (c) 2022 Clearpath Robotics, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the InOrbit, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * Pre-compiler statements
 */
#pragma once

/**
 * C++ Libraries / header
 */
#include <unordered_map>

/**
 * ROS related dependencies / headers
 */
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vda5050_connector/nav_to_node.hpp"
#include "vda5050_connector/state_handler.hpp"
#include "vda5050_connector/utils.hpp"
#include "vda5050_connector/vda_action.hpp"

/**
 * ROS msgs / services
 */
#include "vda5050_connector/srv/get_state.hpp"
#include "vda5050_connector/srv/supported_actions.hpp"

namespace
{
constexpr auto DEFAULT_NODE_NAME = "adapter";
constexpr auto DEFAULT_ROBOT_NAME = "robot_1";
constexpr auto DEFAULT_MANUFACTURER_NAME = "robots";
constexpr auto DEFAULT_SERIAL_NUMBER = "robot_1";
constexpr auto DEFAULT_PLUGIN_PACKAGE = "vda5050_connector";
constexpr auto DEFAULT_ROS_NAMESPACE = "vda5050";

constexpr auto DEFAULT_SUPPORTED_ACTIONS_SVC_NAME = "adapter/supported_actions";
constexpr auto DEFAULT_GET_STATE_SVC_NAME = "adapter/get_state";
constexpr auto DEFAULT_NAV_TO_NODE_ACT_NAME = "adapter/nav_to_node";
constexpr auto DEFAULT_VDA_ACTION_ACT_NAME = "adapter/vda_action";

constexpr auto DEFAULT_VDA_ACTION_CLASS_PLUGIN = "adapter::VDAAction";
constexpr auto DEFAULT_NAV_TO_NODE_CLASS_PLUGIN = "adapter::NavToNode";
constexpr auto DEFAULT_STATE_HANDLER_CLASS_PLUGIN = "adapter::StateHandler";
}  // namespace

namespace adapter
{
using GetState = vda5050_connector::srv::GetState;
using SupportedActions = vda5050_connector::srv::SupportedActions;

/**
 * @brief The Adapter class defines aniInterface for AGV / AMR vendors to
 * implement the communication between a Robot API and the ROS2_VDA5050
 * connector.
 */
class AdapterNode : public rclcpp::Node
{
public:
  template <typename T>
  using UniquePtr = std::unique_ptr<T, std::function<void(T *)>>;

  /**
   * @brief Construct a new Adapter Node with default values
   */
  AdapterNode();

  /**
   * @brief Create a new AdapterNode.
   * @param node_name The name of the adapter node.
   * @param ros_namespace The namespace for the node.
   * @param options Node options.
   */
  AdapterNode(
    const std::string & node_name, const std::string & ros_namespace,
    const rclcpp::NodeOptions & options);

  virtual ~AdapterNode();

  /**
   * @brief Return the robot name.
   */
  std::string get_robot_name() const { return robot_name_; }

protected:
  /**
   * @brief Configure communication interfaces, parameters, variables, etc, for the node to work.
   */
  virtual void on_configure();

  /**
   * @brief Read node related parameters.
   */
  virtual void read_node_parameters();

  /**
   * @brief Read plugin related parameters.
   */
  virtual void read_plugin_parameters();

  /**
   * @brief Read and process state handler parameters to load its plugins.
   */
  virtual void process_state_handler_parameters();

  /**
   * @brief Read and process nav to node parameters to load its plugins.
   */
  virtual void process_nav_to_node_parameters();

  /**
   * @brief Read and process vda action parameters to load its plugins.
   */
  virtual void process_vda_action_parameters();

  /**
   * @brief Call the different state handlers to get their states, and assemble a new updated state.
   */
  virtual void update_current_state();

  // -- Create and add handler instances -- //

  /**
   * @brief Create and add a new state handler.
   * @param handler_name Name of the state handler class to load.\
   * @throw Pluginlib exception if is not possible to create instance of handler_name
   */
  virtual void set_new_state_handler(const std::string & handler_name);

  /**
   * @brief Create and add a new VDA action handler.
   * @param action_name Name of the VDA5050 action.
   * @param action_handler_name Name of the action handler class to load.
   * @throw Pluginlib exception if is not possible to create instance of handler_name
   */
  virtual void set_new_vda_action(
    const std::string & action_name, const std::string & action_handler_name);

  /**
   * @brief Create and add a nav to node handler.
   * @param handler_name Name of the handler class to load.
   * @throw Pluginlib exception if is not possible to create instance of handler_name
   */
  virtual void set_nav_to_node_handler(const std::string & handler_name);

  // -- Get State -- //

  /**
   * @brief Callback for state requests.
   * @param request Empty request message.
   * @param response State message filled with adapter information.
   */
  virtual void get_state_callback(
    const std::shared_ptr<GetState::Request> request,
    const std::shared_ptr<GetState::Response> response);

  // -- Get State -- //

  // -- VDA Action -- //

  /**
   * @brief Callback that decides if a goal should be accepted or rejected.
   * @param uuid Goal ID.
   * @param goal ProcessVDAAction goal.
   * @return Goal response.
   */
  virtual rclcpp_action::GoalResponse vda_action_handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ProcessVDAAction::Goal> goal) const;

  /**
   * @brief Callback that decides if a goal should be attempted to be canceled.
   * @param goal_handle The handle to the goal to cancel.
   */
  virtual rclcpp_action::CancelResponse vda_action_handle_cancel(
    const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle) const;

  /**
   * @brief Callback that triggers when a goal is accepted.
   * @param goal_handle The handle to the accepted goal.
   */
  virtual void vda_action_handle_accepted(
    const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle);

  /**
   * @brief Execute the corresponding VDA action.
   * @param goal_handle The handle to the goal to execute.
   */
  virtual void execute_vda_action(const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle);

  // -- VDA Action -- //

  // -- Navigate to Node -- //

  /**
   * @brief Handle navigate to node action request.
   * @param uuid Goal ID.
   * @param goal A pointer to the nav to node goal.
   * @return Goal response.
   */
  virtual rclcpp_action::GoalResponse nav_to_node_handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToNode::Goal> goal) const;

  /**
   * @brief Callback that decides if a nav to node action should be attempted
   * to be canceled.
   * @param goal_handle The handle to the nav to node action to cancel.
   */
  virtual rclcpp_action::CancelResponse nav_to_node_handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToNode> goal_handle) const;

  /**
   * @brief Callback that triggers when a nav to node action is accepted.
   * @param goal_handle The handle to the accepted nav to node action.
   */
  virtual void nav_to_node_handle_accepted(
    const std::shared_ptr<GoalHandleNavigateToNode> goal_handle);

  /**
   * @brief Execute a nav to node action request.
   * @param goal_handle The handle to the nav to node action.
   */
  virtual void execute_nav_to_node(const std::shared_ptr<GoalHandleNavigateToNode> goal_handle);

  // -- Navigate to Node -- //

  /**
   * @brief Callback for supported actions service.
   * @param request Empty message.
   * @param response Array of supported msgs and scopes allowed per action.
   */
  virtual void supported_actions_callback(
    const std::shared_ptr<SupportedActions::Request> request,
    std::shared_ptr<SupportedActions::Response> response) const;

  // Variables //

  // Unique robot name, manufacturer and serial number
  std::string robot_name_{DEFAULT_ROBOT_NAME};
  std::string manufacturer_name_{DEFAULT_MANUFACTURER_NAME};
  std::string serial_number_{DEFAULT_SERIAL_NUMBER};

  // Plugin package name
  const std::string plugin_package_{DEFAULT_PLUGIN_PACKAGE};

  // ROS interfaces names
  std::string supported_actions_svc_name_{DEFAULT_SUPPORTED_ACTIONS_SVC_NAME};
  std::string get_state_svc_name_{DEFAULT_GET_STATE_SVC_NAME};
  std::string vda_action_act_name_{DEFAULT_VDA_ACTION_ACT_NAME};
  std::string nav_to_node_act_name_{DEFAULT_NAV_TO_NODE_ACT_NAME};

  // Plugin classes names
  const std::string state_handler_class_plugin_{DEFAULT_STATE_HANDLER_CLASS_PLUGIN};
  const std::string vda_action_class_plugin_{DEFAULT_VDA_ACTION_CLASS_PLUGIN};
  const std::string nav_to_node_class_plugin_{DEFAULT_NAV_TO_NODE_CLASS_PLUGIN};

  // Current state
  std::unique_ptr<SafeState> current_state_;

  // -- State Handler-- //
  // Get state
  rclcpp::Service<GetState>::SharedPtr get_state_svc_srv_;

  // Registered state_handlers plugins
  std::vector<UniquePtr<StateHandler>> state_handlers_;

  // Pluginlib StateHandlers class loader
  std::unique_ptr<pluginlib::ClassLoader<StateHandler>> state_handler_loader_;
  // -- State Handler-- //

  // -- VDA Action -- //
  // VDA5050 action
  rclcpp_action::Server<ProcessVDAAction>::SharedPtr vda_action_act_srv_;

  // Registered vda_action handler plugins
  std::unordered_map<std::string, UniquePtr<VDAAction>> vda_actions_;

  // PluginLib VDAAction class loader
  std::unique_ptr<pluginlib::ClassLoader<VDAAction>> vda_action_loader_;
  // -- VDA Action -- //

  // -- Navigate to Node -- //
  // Nav to node
  rclcpp_action::Server<NavigateToNode>::SharedPtr nav_to_node_act_srv_;

  // Registered nav_to_node handler plugin
  UniquePtr<NavToNode> nav_to_node_;

  // PluginLib NavToNode class loader
  std::unique_ptr<pluginlib::ClassLoader<NavToNode>> nav_to_node_loader_;
  // -- Navigate to Node -- //

  rclcpp::Service<SupportedActions>::SharedPtr supported_actions_svc_srv_;
};

}  // namespace adapter
