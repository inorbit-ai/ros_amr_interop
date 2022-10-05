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
#include <gtest/gtest.h>

#include "common/adapter_test.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test
{
class AdapterTest : public ::testing::Test
{
public:
  static constexpr auto p_state_handler_names = "state_handler_names";
  static constexpr auto p_nav_to_node_handler = "nav_to_node.handler";
  static constexpr auto p_vda_action_handlers = "vda_action_handlers";
};

TEST_F(AdapterTest, HandlerExecution)
{
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  std::vector<rclcpp::Parameter> node_parameters;
  node_parameters.push_back(rclcpp::Parameter(
    p_state_handler_names,
    std::vector<std::string>{"test::StubStateHandlerPub", "test::StubStateHandlerData"}));

  node_parameters.push_back(
    rclcpp::Parameter(p_vda_action_handlers, std::vector<std::string>{"actionData"}));
  node_parameters.push_back(rclcpp::Parameter("action_data.handler", "test::StubVDAActionData"));

  node_options.parameter_overrides(node_parameters);

  // Default constructor
  std::unique_ptr<AdapterNodeTest> adapter_node = nullptr;
  ASSERT_NO_THROW(
    adapter_node = std::make_unique<AdapterNodeTest>("adapter_test", "vda5050_test", node_options));

  // Check plugins are loaded successfully
  ASSERT_EQ(static_cast<int>(adapter_node->get_state_handlers().size()), 2);
  ASSERT_EQ(static_cast<int>(adapter_node->get_vda_actions().size()), 1);

  // Check handlers could modify global current state on configure
  auto order_state = adapter_node->get_current_state();
  EXPECT_EQ(static_cast<int>(order_state.informations.size()), 1);
  EXPECT_FLOAT_EQ(static_cast<int>(order_state.battery_state.battery_charge), 100.0);

  // Check handlers could modify global current state on execute
  ASSERT_NO_THROW(adapter_node->call_update_current_state());
  order_state = adapter_node->get_current_state();
  // Update current state should clear previous loads, error and informations
  EXPECT_EQ(static_cast<int>(order_state.informations.size()), 1);
  EXPECT_FLOAT_EQ(static_cast<int>(order_state.battery_state.battery_charge), 90.0);

  // Test vda action execute state machine: Transitions and call of state funcs
  // Use actionData vda action to test transition between states
  // The state machine has to transition first to INITIALIZING automatically, calling its function.
  // The initialize function will read the transition param, and update to be on PAUSED state.
  // The pause function will be called, updating to FINISHED state.
  // The finish function will change a data member on the global state, to later finish the execution.
  vda5050_msgs::msg::Action action;
  action.action_type = "actionData";
  vda5050_msgs::msg::ActionParameter param_transition;
  param_transition.key = "transition";
  param_transition.value = "paused";
  action.action_parameters.push_back(param_transition);

  EXPECT_EQ(adapter_node->execute_vda_action(action), VDAAction::STATES::FINISHED);
  order_state = adapter_node->get_current_state();
  EXPECT_EQ(order_state.paused, false);

  // Test fail case
  action = vda5050_msgs::msg::Action();
  action.action_type = "actionData";
  EXPECT_EQ(adapter_node->execute_vda_action(action), VDAAction::STATES::FAILED);
}
}  // namespace test

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
