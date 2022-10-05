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
 * C++ Libraries / header
 */
#include "vda5050_connector/adapter.hpp"

/**
 * ROS msgs / services
 */
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/order_state.hpp"

namespace test
{
using namespace adapter;
class AdapterNodeTest : public AdapterNode
{
public:
  AdapterNodeTest() = default;
  AdapterNodeTest(
    const std::string & node_name, const std::string & ros_namespace,
    const rclcpp::NodeOptions & options)
  : AdapterNode(node_name, ros_namespace, options){};

  // Getters of handler reference containers
  std::vector<UniquePtr<StateHandler>> & get_state_handlers() { return state_handlers_; }
  std::unordered_map<std::string, UniquePtr<VDAAction>> & get_vda_actions() { return vda_actions_; }
  UniquePtr<NavToNode> & get_nav_to_node() { return nav_to_node_; }

  // Callbacks
  std::vector<vda5050_msgs::msg::AGVAction> get_supported_actions() const
  {
    auto response = std::make_shared<SupportedActions::Response>();
    supported_actions_callback(std::make_shared<SupportedActions::Request>(), response);
    return response->agv_actions;
  }

  // Update state by calling state handler execute functions
  void call_update_current_state() { update_current_state(); }

  // Global state
  vda5050_msgs::msg::OrderState get_current_state() const { return current_state_->get(); }

  // Execute vda action
  VDAAction::STATES execute_vda_action(const vda5050_msgs::msg::Action & action)
  {
    const std::string action_name = action.action_type;
    vda_actions_.at(action_name)->reset(action, nullptr);
    vda_actions_.at(action_name)->execute();
    return vda_actions_.at(action_name)->get_action_state();
  }
};
}  // namespace test
