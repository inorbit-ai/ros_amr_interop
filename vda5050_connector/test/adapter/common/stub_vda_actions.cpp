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
#include "pluginlib/class_list_macros.hpp"
#include "vda5050_connector/nav_to_node.hpp"
#include "vda5050_connector/state_handler.hpp"
#include "vda5050_connector/vda_action.hpp"

/**
 * ROS msgs / services
 */
#include "geometry_msgs/msg/vector3.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/info.hpp"
#include "vda5050_msgs/msg/order_state.hpp"

namespace test
{
class StubVDAActionSub : public adapter::VDAAction
{
public:
  StubVDAActionSub() = default;
  void configure() override
  {
    vda_action_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      "vda_action_sub", 10, [](const geometry_msgs::msg::Vector3 & data) { (void)data; });
  }
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vda_action_sub_;
};

class StubVDAActionData : public adapter::VDAAction
{
public:
  StubVDAActionData() = default;
  void configure() override
  {
    current_state_->set_parameter(&vda5050_msgs::msg::OrderState::paused, true);
  }
  void update_action_state(STATES action_state) override { action_state_ = action_state; }
  void initialize() override
  {
    auto get_transition =
      [](std::vector<vda5050_msgs::msg::ActionParameter> parameters) -> std::string {
      for (const auto & parameter : parameters) {
        if (parameter.key == "transition") {
          return std::string(parameter.value);
        }
      }
      return std::string("failed");
    };
    const std::string transition = get_transition(action_msg_.action_parameters);
    if (transition == "paused") {
      update_action_state(STATES::PAUSED);
    } else {
      update_action_state(STATES::FAILED);
    }
  }

  void pause() override { update_action_state(STATES::FINISHED); }
  void finish() override
  {
    current_state_->set_parameter(&vda5050_msgs::msg::OrderState::paused, false);
  }
};
}  // namespace test

PLUGINLIB_EXPORT_CLASS(test::StubVDAActionSub, adapter::VDAAction)
PLUGINLIB_EXPORT_CLASS(test::StubVDAActionData, adapter::VDAAction)
