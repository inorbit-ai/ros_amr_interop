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
protected:
  static constexpr auto p_robot_name = "robot_name";
  static constexpr auto p_manufacturer_name = "manufacturer_name";
  static constexpr auto p_serial_number = "serial_number";

  static constexpr auto p_supported_actions_svc_name = "supported_actions_svc_name";
  static constexpr auto p_get_state_svc_name = "get_state_svc_name";
  static constexpr auto p_vda_action_act_name = "vda_action_act_name";
  static constexpr auto p_nav_to_node_act_name = "nav_to_node_act_name";
};

/**
 * @brief Test adapter node instance building with default parameters.
 * It should set node parameters with the defaults.
 * It should not load instance of any plugin on the handle containers.
 */
TEST_F(AdapterTest, NodeCreationDefault)
{
  // Default constructor
  std::unique_ptr<AdapterNodeTest> adapter_node = nullptr;
  ASSERT_NO_THROW(adapter_node = std::make_unique<AdapterNodeTest>());

  // Check node parameters load non-empty default values
  EXPECT_EQ(adapter_node->get_robot_name(), "robot_1");
  EXPECT_STREQ(adapter_node->get_name(), "adapter");
  EXPECT_STREQ(adapter_node->get_namespace(), "/vda5050");
  EXPECT_EQ(adapter_node->get_parameter(p_manufacturer_name).as_string(), "robots");
  EXPECT_EQ(adapter_node->get_parameter(p_serial_number).as_string(), "robot_1");

  // Check adapter interface communication names load non-empty default values
  EXPECT_EQ(
    adapter_node->get_parameter(p_supported_actions_svc_name).as_string(),
    "adapter/supported_actions");
  EXPECT_EQ(adapter_node->get_parameter(p_get_state_svc_name).as_string(), "adapter/get_state");
  EXPECT_EQ(adapter_node->get_parameter(p_vda_action_act_name).as_string(), "adapter/vda_action");
  EXPECT_EQ(adapter_node->get_parameter(p_nav_to_node_act_name).as_string(), "adapter/nav_to_node");

  // Check emptyness of handler containers
  EXPECT_EQ(static_cast<int>(adapter_node->get_state_handlers().size()), 0);
  EXPECT_EQ(static_cast<int>(adapter_node->get_vda_actions().size()), 0);
  EXPECT_EQ(adapter_node->get_nav_to_node(), nullptr);
}

/**
 * @brief Test adapter node instance building with node parameters.
 * It should set node parameters with the ones provided.
 * It should not load instance of any plugin on the handle containers.
 */
TEST_F(AdapterTest, NodeCreationParameters)
{
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  std::vector<rclcpp::Parameter> node_parameters;
  node_parameters.push_back(rclcpp::Parameter(p_robot_name, "robot_test"));
  node_parameters.push_back(rclcpp::Parameter(p_manufacturer_name, "robot_tests"));
  node_parameters.push_back(rclcpp::Parameter(p_serial_number, "robot_1"));
  node_parameters.push_back(rclcpp::Parameter(p_supported_actions_svc_name, "supported_actions"));
  node_parameters.push_back(rclcpp::Parameter(p_get_state_svc_name, "get_state"));
  node_parameters.push_back(rclcpp::Parameter(p_vda_action_act_name, "nav_to_node"));
  node_parameters.push_back(rclcpp::Parameter(p_nav_to_node_act_name, "vda_action"));
  node_options.parameter_overrides(node_parameters);

  // Constructor with parameters
  std::unique_ptr<AdapterNodeTest> adapter_node = nullptr;
  ASSERT_NO_THROW(
    adapter_node = std::make_unique<AdapterNodeTest>("adapter_test", "vda5050_test", node_options));

  // Check parameters have loaded correctly
  EXPECT_EQ(adapter_node->get_robot_name(), "robot_test");
  EXPECT_STREQ(adapter_node->get_name(), "adapter_test");
  EXPECT_STREQ(adapter_node->get_namespace(), "/vda5050_test");
  EXPECT_EQ(adapter_node->get_parameter(p_manufacturer_name).as_string(), "robot_tests");
  EXPECT_EQ(adapter_node->get_parameter(p_serial_number).as_string(), "robot_1");
  EXPECT_EQ(
    adapter_node->get_parameter(p_supported_actions_svc_name).as_string(), "supported_actions");
  EXPECT_EQ(adapter_node->get_parameter(p_get_state_svc_name).as_string(), "get_state");
  EXPECT_EQ(adapter_node->get_parameter(p_vda_action_act_name).as_string(), "nav_to_node");
  EXPECT_EQ(adapter_node->get_parameter(p_nav_to_node_act_name).as_string(), "vda_action");

  // Check emptyness of handler containers
  EXPECT_EQ(static_cast<int>(adapter_node->get_state_handlers().size()), 0);
  EXPECT_EQ(static_cast<int>(adapter_node->get_vda_actions().size()), 0);
  EXPECT_EQ(adapter_node->get_nav_to_node(), nullptr);

  // Check if services have been created and have correct name
  auto services_info = adapter_node->get_service_names_and_types();
  EXPECT_NE(
    services_info.find("/vda5050_test/robot_tests/robot_test/supported_actions"),
    services_info.end());
  EXPECT_NE(
    services_info.find("/vda5050_test/robot_tests/robot_test/get_state"), services_info.end());
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
