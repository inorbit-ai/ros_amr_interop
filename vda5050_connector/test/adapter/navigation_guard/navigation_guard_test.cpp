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

#include <chrono>
#include <string>

#include "common/adapter_test.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace test
{
using namespace std::chrono_literals;

TEST(NavigationGuardTest, SecondGoalRejectedWhileFirstIsActive)
{
  // creates the adapter with a stub whose execute() never finishes the goal,
  // so an accepted goal stays active
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {rclcpp::Parameter("nav_to_node.handler", "test::StubNavToNode")});
  auto adapter_node =
    std::make_shared<AdapterNodeTest>("adapter_test", "vda5050_test", node_options);

  // creates a client node playing the controller
  auto client_node = std::make_shared<rclcpp::Node>("navigation_guard_client");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(client_node);

  // connects to the adapter's nav to node action server
  const auto action_name = std::string("/vda5050_test/") + DEFAULT_MANUFACTURER_NAME + "/" +
                           DEFAULT_ROBOT_NAME + "/" + DEFAULT_NAV_TO_NODE_ACT_NAME;
  auto client = rclcpp_action::create_client<adapter::NavigateToNode>(client_node, action_name);
  ASSERT_TRUE(client->wait_for_action_server(5s));

  // sends two goals back to back, without waiting for the first response
  auto first_goal = client->async_send_goal(adapter::NavigateToNode::Goal());
  auto second_goal = client->async_send_goal(adapter::NavigateToNode::Goal());

  ASSERT_EQ(executor.spin_until_future_complete(first_goal, 5s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(
    executor.spin_until_future_complete(second_goal, 5s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_NE(first_goal.get(), nullptr);
  EXPECT_EQ(second_goal.get(), nullptr);
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
