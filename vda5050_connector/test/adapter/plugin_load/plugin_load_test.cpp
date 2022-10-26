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

/**
 * ROS msgs / services
 */
#include "vda5050_msgs/msg/agv_action.hpp"

namespace test
{
class AdapterTest : public ::testing::Test
{
public:
  static constexpr auto p_state_handler_names = "state_handler_names";
  static constexpr auto p_nav_to_node_handler = "nav_to_node.handler";
  static constexpr auto p_vda_action_handlers = "vda_action_handlers";
};

/**
 * @brief Test adapter node load stub handler plugins.
 * It should load the giveng handler parameters and load the stubHandlers.
 * It should read the vda aciton supported info and configure it properly to be requested.
 */
TEST_F(AdapterTest, HandlerLoadHandlers)
{
  using AGVAction = vda5050_msgs::msg::AGVAction;
  using ActionParameterDefinition = vda5050_msgs::msg::ActionParameterDefinition;

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  std::vector<rclcpp::Parameter> node_parameters;
  node_parameters.push_back(rclcpp::Parameter(
    p_state_handler_names,
    std::vector<std::string>{"test::StubStateHandlerPub", "test::StubStateHandlerData"}));

  node_parameters.push_back(rclcpp::Parameter(p_nav_to_node_handler, "test::StubNavToNode"));

  node_parameters.push_back(
    rclcpp::Parameter(p_vda_action_handlers, std::vector<std::string>{"actionData", "actionSub"}));

  node_parameters.push_back(rclcpp::Parameter("action_data.handler", "test::StubVDAActionData"));

  node_parameters.push_back(rclcpp::Parameter("action_sub.handler", "test::StubVDAActionSub"));
  node_parameters.push_back(rclcpp::Parameter("action_sub.description", "VDA Action test"));
  node_parameters.push_back(rclcpp::Parameter("action_sub.scopes.instant", true));
  node_parameters.push_back(rclcpp::Parameter("action_sub.scopes.edge", true));
  node_parameters.push_back(
    rclcpp::Parameter("action_sub.parameters", std::vector<std::string>{"param_1", "param_2"}));
  node_parameters.push_back(rclcpp::Parameter("action_sub.parameter_param_1.data_type", "STRING"));
  node_parameters.push_back(
    rclcpp::Parameter("action_sub.parameter_param_1.description", "Parameter test 1"));
  node_parameters.push_back(rclcpp::Parameter("action_sub.parameter_param_1.is_optional", true));
  node_parameters.push_back(
    rclcpp::Parameter("action_sub.parameter_param_2.description", "Parameter test 2"));
  node_options.parameter_overrides(node_parameters);

  std::unique_ptr<AdapterNodeTest> adapter_node = nullptr;
  ASSERT_NO_THROW(
    adapter_node = std::make_unique<AdapterNodeTest>("adapter_test", "vda5050_test", node_options));

  // Check plugins are loaded successfully
  ASSERT_EQ(static_cast<int>(adapter_node->get_state_handlers().size()), 2);
  ASSERT_EQ(static_cast<int>(adapter_node->get_vda_actions().size()), 2);
  ASSERT_FALSE(adapter_node->get_nav_to_node() == nullptr);

  // Check configure functions where called correctly by checking if node publishers were created
  EXPECT_EQ(static_cast<int>(adapter_node->count_publishers("/vda5050_test/state_handler_pub")), 1);
  EXPECT_EQ(static_cast<int>(adapter_node->count_publishers("/vda5050_test/nav_to_node_pub")), 1);
  EXPECT_EQ(static_cast<int>(adapter_node->count_subscribers("/vda5050_test/vda_action_sub")), 1);

  // Check handlers could modify global current state on configure
  auto order_state = adapter_node->get_current_state();
  EXPECT_EQ(static_cast<int>(order_state.informations.size()), 1);
  EXPECT_FLOAT_EQ(order_state.distance_since_last_node, 10.0);
  EXPECT_EQ(order_state.paused, true);

  // Check supported actions is filled correctly
  const auto supported_actions = adapter_node->get_supported_actions();
  ASSERT_EQ(static_cast<int>(supported_actions.size()), 2);

  // Check Supported action information of actionSub
  const auto agv_action_sub = supported_actions.at(0).action_type == "actionSub"
                                ? supported_actions.at(0)
                                : supported_actions.at(1);
  EXPECT_EQ(agv_action_sub.action_type, "actionSub");
  EXPECT_EQ(agv_action_sub.action_description, "VDA Action test");

  const auto scopes = std::vector<std::string>{AGVAction::INSTANT, AGVAction::EDGE};
  EXPECT_EQ(agv_action_sub.action_scopes, scopes);

  const auto agv_action_params_test = agv_action_sub.action_parameters;
  ASSERT_EQ(static_cast<int>(agv_action_params_test.size()), 2);
  EXPECT_EQ(agv_action_params_test.at(0).key, "param_1");
  EXPECT_EQ(agv_action_params_test.at(0).value_data_type, ActionParameterDefinition::STRING);
  EXPECT_EQ(agv_action_params_test.at(0).description, "Parameter test 1");
  EXPECT_EQ(agv_action_params_test.at(0).is_optional, true);

  EXPECT_EQ(agv_action_params_test.at(1).key, "param_2");
  EXPECT_EQ(agv_action_params_test.at(1).value_data_type, ActionParameterDefinition::OBJECT);
  EXPECT_EQ(agv_action_params_test.at(1).description, "Parameter test 2");
  EXPECT_EQ(agv_action_params_test.at(1).is_optional, false);
}
using vectorParam = std::vector<rclcpp::Parameter>;
class AdapterThrowTest : public ::testing::TestWithParam<vectorParam>
{
};

/**
 * @brief Test adapter node throw instances when loading unknown handler plugins.
 * It should throw a PluginlibException when plugin lib not found or fail to loaded
 */
TEST_P(AdapterThrowTest, HandlerLoadThrows)
{
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(GetParam());
  EXPECT_THROW(
    AdapterNodeTest("adapter_test", "vda5050_test", node_options), pluginlib::PluginlibException);
}

// Test each handler case: StateHandler, NavToNode and VDAAction
INSTANTIATE_TEST_SUITE_P(
  HandlerLoadThrowsCases, AdapterThrowTest,
  ::testing::Values(
    vectorParam{rclcpp::Parameter(
      "state_handler_names", std::vector<std::string>{"test::StubStateHandlerWrong"})},
    vectorParam{rclcpp::Parameter("nav_to_node.handler", "test::StubNavToNodeWrong")},
    vectorParam{
      rclcpp::Parameter("vda_action_handlers", std::vector<std::string>{"action_test"}),
      rclcpp::Parameter("action_test.handler", "test::StubVDAActionWrong")}),
  [](const ::testing::TestParamInfo<AdapterThrowTest::ParamType> & info) {
    const std::string name = info.param.at(0).get_name();
    return name == "nav_to_node.handler" ? std::string("nav_to_node") : name;
  });
}  // namespace test

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
