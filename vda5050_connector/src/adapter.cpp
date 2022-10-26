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
 * C++ Libraries
 */
#include <thread>

/**
 * Package Files
 */
#include "vda5050_connector/adapter.hpp"

namespace adapter
{
AdapterNode::AdapterNode()
: AdapterNode(DEFAULT_NODE_NAME, DEFAULT_ROS_NAMESPACE, rclcpp::NodeOptions())
{
}

AdapterNode::AdapterNode(
  const std::string & node_name, const std::string & ros_namespace,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, ros_namespace, options)
{
  on_configure();
  RCLCPP_INFO(get_logger(), "Node [%s] has started successfully.", node_name.c_str());
}

AdapterNode::~AdapterNode()
{
  state_handlers_.clear();
  vda_actions_.clear();
  nav_to_node_ = nullptr;
}

void AdapterNode::on_configure()
{
  using namespace std::placeholders;

  read_node_parameters();

  // Create current state
  current_state_ = std::make_unique<SafeState>();

  // Create state handler pluginlib loader
  state_handler_loader_ = std::make_unique<pluginlib::ClassLoader<adapter::StateHandler>>(
    plugin_package_, state_handler_class_plugin_);

  // Create vda actions pluginlib loader
  vda_action_loader_ = std::make_unique<pluginlib::ClassLoader<adapter::VDAAction>>(
    plugin_package_, vda_action_class_plugin_);

  // Create nav to node pluginlib loader
  nav_to_node_loader_ = std::make_unique<pluginlib::ClassLoader<adapter::NavToNode>>(
    plugin_package_, nav_to_node_class_plugin_);

  std::string base_interface_name = std::string(get_namespace()) + "/";
  base_interface_name += manufacturer_name_ + std::string("/");
  base_interface_name += robot_name_ + std::string("/");

  // Services
  supported_actions_svc_srv_ = create_service<SupportedActions>(
    base_interface_name + supported_actions_svc_name_,
    std::bind(&AdapterNode::supported_actions_callback, this, _1, _2));

  get_state_svc_srv_ = create_service<GetState>(
    base_interface_name + get_state_svc_name_,
    std::bind(&AdapterNode::get_state_callback, this, _1, _2));

  // Actions
  nav_to_node_act_srv_ = rclcpp_action::create_server<NavigateToNode>(
    this, base_interface_name + nav_to_node_act_name_,
    std::bind(&AdapterNode::nav_to_node_handle_goal, this, _1, _2),
    std::bind(&AdapterNode::nav_to_node_handle_cancel, this, _1),
    std::bind(&AdapterNode::nav_to_node_handle_accepted, this, _1));

  vda_action_act_srv_ = rclcpp_action::create_server<ProcessVDAAction>(
    this, base_interface_name + vda_action_act_name_,
    std::bind(&AdapterNode::vda_action_handle_goal, this, _1, _2),
    std::bind(&AdapterNode::vda_action_handle_cancel, this, _1),
    std::bind(&AdapterNode::vda_action_handle_accepted, this, _1));

  read_plugin_parameters();
}

void AdapterNode::read_node_parameters()
{
  using namespace vda5050_connector::utils;
  robot_name_ = read_str_parameter(this, "robot_name", robot_name_);
  manufacturer_name_ = read_str_parameter(this, "manufacturer_name", manufacturer_name_);
  serial_number_ = read_str_parameter(this, "serial_number", serial_number_);

  supported_actions_svc_name_ =
    read_str_parameter(this, "supported_actions_svc_name", supported_actions_svc_name_);
  get_state_svc_name_ = read_str_parameter(this, "get_state_svc_name", get_state_svc_name_);
  vda_action_act_name_ = read_str_parameter(this, "vda_action_act_name", vda_action_act_name_);
  nav_to_node_act_name_ = read_str_parameter(this, "nav_to_node_act_name", nav_to_node_act_name_);
}

void AdapterNode::read_plugin_parameters()
{
  process_state_handler_parameters();
  process_nav_to_node_parameters();
  process_vda_action_parameters();
}

void AdapterNode::process_state_handler_parameters()
{
  const std::vector<std::string> state_handler_names =
    vda5050_connector::utils::read_str_array_parameter(this, "state_handler_names");
  for (const std::string & handler_name : state_handler_names) {
    set_new_state_handler(handler_name);
  }
}

void AdapterNode::process_nav_to_node_parameters()
{
  const std::string handler_name =
    vda5050_connector::utils::read_str_parameter(this, "nav_to_node.handler");
  if (handler_name == "") {
    RCLCPP_WARN(
      get_logger(),
      "The 'handler' parameter for the nav_to_node key was not provided. Unable to create the "
      "plugin instance.");
    return;
  }
  set_nav_to_node_handler(handler_name);
}

void AdapterNode::process_vda_action_parameters()
{
  using namespace vda5050_connector::utils;
  const std::vector<std::string> vda_action_handler_names =
    read_str_array_parameter(this, "vda_action_handlers");
  for (const std::string & handler_name : vda_action_handler_names) {
    const std::string action_name_key = to_snake_case(handler_name);
    const std::string handler_name_handler = read_str_parameter(this, action_name_key + ".handler");

    if (handler_name_handler == "") {
      RCLCPP_ERROR(
        get_logger(),
        "The 'handler' parameter for the [%s] key was not provided. Unable to create the VDA "
        "action.",
        action_name_key.c_str());
      continue;
    }

    set_new_vda_action(handler_name, handler_name_handler);

    // Read description and result
    const std::string description = read_str_parameter(this, action_name_key + ".description");
    const std::string result_description =
      read_str_parameter(this, action_name_key + ".result_description");

    vda_actions_.at(handler_name)
      ->set_supported_info(handler_name, description, result_description);

    // Read supported scopes
    const std::string scope_key = action_name_key + ".scopes";

    const bool instant = read_bool_parameter(this, scope_key + ".instant");
    const bool node = read_bool_parameter(this, scope_key + ".node");
    const bool edge = read_bool_parameter(this, scope_key + ".edge");

    vda_actions_.at(handler_name)->set_supported_scopes(instant, node, edge);

    // Read supported parameter definitions
    const std::vector<std::string> vda_action_parameters =
      read_str_array_parameter(this, action_name_key + ".parameters");
    for (const std::string & parameter : vda_action_parameters) {
      const std::string parameter_key = action_name_key + ".parameter_" + parameter;

      const std::string data_type =
        read_str_parameter(this, parameter_key + ".data_type", ActionParameterDefinition::OBJECT);
      const std::string description = read_str_parameter(this, parameter_key + ".description");
      bool is_optional = read_bool_parameter(this, parameter_key + ".is_optional");

      vda_actions_.at(handler_name)
        ->set_supported_parameter(parameter, data_type, description, is_optional);
    }
  }
}

void AdapterNode::update_current_state()
{
  // Clear information, error and load arrays
  current_state_->clear();
  for (auto & state_handler : state_handlers_) {
    state_handler->execute();
  }
}

void AdapterNode::set_new_state_handler(const std::string & handler_name)
{
  state_handlers_.emplace_back(state_handler_loader_->createUniqueInstance(handler_name));
  state_handlers_.back()->compose(this, current_state_.get(), get_robot_name());
  state_handlers_.back()->configure();
  RCLCPP_INFO(get_logger(), "Created state handler with plugin name [%s].", handler_name.c_str());
}

void AdapterNode::set_new_vda_action(
  const std::string & action_name, const std::string & action_handler_name)
{
  vda_actions_[action_name] = vda_action_loader_->createUniqueInstance(action_handler_name);
  vda_actions_.at(action_name)->compose(this, current_state_.get(), get_robot_name());
  vda_actions_.at(action_name)->configure();
  RCLCPP_INFO(
    get_logger(), "Created action [%s] handler plugin from class name [%s].", action_name.c_str(),
    action_handler_name.c_str());
}

void AdapterNode::set_nav_to_node_handler(const std::string & handler_name)
{
  nav_to_node_ = nav_to_node_loader_->createUniqueInstance(handler_name);
  nav_to_node_->compose(this, current_state_.get(), get_robot_name());
  nav_to_node_->configure();
  RCLCPP_INFO(
    get_logger(), "Created nav to node handler plugin from class name [%s].", handler_name.c_str());
}

void AdapterNode::execute_vda_action(const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle)
{
  std::string action_name = goal_handle->get_goal()->action.action_type;

  try {
    vda_actions_.at(action_name)->reset(goal_handle->get_goal()->action, goal_handle);
    vda_actions_.at(action_name)->execute();
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(), "Error executing VDA action, action_type [%s] is invalid or unsupported. [%s].",
      action_name.c_str(), e.what());

    auto result = std::make_shared<ProcessVDAAction::Result>();
    result->result.action_id = goal_handle->get_goal()->action.action_id;
    result->result.action_description = goal_handle->get_goal()->action.action_description;
    result->result.action_status =
      adapter::VDAAction::action_state_str(adapter::VDAAction::STATES::FAILED);
    result->result.result_description = "Action invalid or unsupported.";

    goal_handle->abort(result);
  }
}

void AdapterNode::execute_nav_to_node(const std::shared_ptr<GoalHandleNavigateToNode> goal_handle)
{
  try {
    nav_to_node_->reset(goal_handle->get_goal()->node, goal_handle);
    nav_to_node_->execute();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while navigating to node: [%s].", e.what());
  }
}

void AdapterNode::supported_actions_callback(
  const std::shared_ptr<SupportedActions::Request> request,
  std::shared_ptr<SupportedActions::Response> response) const
{
  (void)request;

  for (auto & vda_action_pair : vda_actions_) {
    response->agv_actions.push_back(vda_action_pair.second->get_supported_action_info());
  }
  RCLCPP_INFO(get_logger(), "Supported actions service request accepted.");
}

void AdapterNode::get_state_callback(
  const std::shared_ptr<GetState::Request> request,
  const std::shared_ptr<GetState::Response> response)
{
  (void)request;

  update_current_state();
  response->state = current_state_->get();
}

rclcpp_action::GoalResponse AdapterNode::vda_action_handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ProcessVDAAction::Goal> goal) const
{
  using STATES = VDAAction::STATES;
  (void)uuid;
  (void)goal;
  const std::string action_name = goal->action.action_type;
  STATES state = STATES::INITIALIZING;

  try {
    state = vda_actions_.at(action_name)->get_action_state();
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Error while getting action state, action_type [%s] is invalid or unsupported. [%s].",
      action_name.c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(), "Received VDA5050 action goal request of type [%s].", action_name.c_str());
  if (state == STATES::INITIALIZING || state == STATES::RUNNING || state == STATES::PAUSED) {
    RCLCPP_INFO(get_logger(), "VDA5050 action rejected. There is running action of the same type.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "VDA5050 action accepted.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AdapterNode::vda_action_handle_cancel(
  const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle) const
{
  const std::string action_name = goal_handle->get_goal()->action.action_type;
  RCLCPP_INFO(
    get_logger(), "Received request to cancel VDA5050 action of type [%s].", action_name.c_str());
  if (vda_actions_.at(action_name)->cancel()) {
    RCLCPP_INFO(get_logger(), "Request to cancel VDA action accepted.");
    return rclcpp_action::CancelResponse::ACCEPT;
  } else {
    RCLCPP_INFO(get_logger(), "Request to cancel VDA action denied.");
    return rclcpp_action::CancelResponse::REJECT;
  }
}

void AdapterNode::vda_action_handle_accepted(
  const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle)
{
  std::thread{std::bind(&AdapterNode::execute_vda_action, this, std::placeholders::_1), goal_handle}
    .detach();
}

rclcpp_action::GoalResponse AdapterNode::nav_to_node_handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToNode::Goal> goal) const
{
  (void)goal;
  RCLCPP_INFO(get_logger(), "Received navigation goal request with ID [%d].", uuid.at(0));
  if (nav_to_node_->is_driving()) {
    RCLCPP_INFO(
      get_logger(), "Navigation goal [%d] has been rejected. There is an active goal executing.",
      uuid.at(0));
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AdapterNode::nav_to_node_handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToNode> goal_handle) const
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Received request to cancel navigation goal.");
  if (!nav_to_node_->cancel()) {
    RCLCPP_INFO(get_logger(), "Unable to cancel navigation goal.");
    return rclcpp_action::CancelResponse::REJECT;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AdapterNode::nav_to_node_handle_accepted(
  const std::shared_ptr<GoalHandleNavigateToNode> goal_handle)
{
  std::thread{
    std::bind(&AdapterNode::execute_nav_to_node, this, std::placeholders::_1), goal_handle}
    .detach();
}

}  // namespace adapter
