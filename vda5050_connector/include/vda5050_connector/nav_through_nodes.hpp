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

#pragma once

#include "vda5050_connector/handler.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "vda5050_connector/action/navigate_through_nodes.hpp"
#include "vda5050_msgs/msg/edge.hpp"
#include "vda5050_msgs/msg/node.hpp"

namespace adapter
{
using NavigateThroughNodes = vda5050_connector::action::NavigateThroughNodes;
using GoalHandleNavigateThroughNodes = rclcpp_action::ServerGoalHandle<NavigateThroughNodes>;

class NavThroughNodes : public Handler
{
public:
  virtual void reset(
    const std::vector<vda5050_msgs::msg::Edge> & edges_msg,
    const std::vector<vda5050_msgs::msg::Node> & nodes_msg,
    const std::shared_ptr<GoalHandleNavigateThroughNodes> goal_handle)
  {
    edges_msg_ = edges_msg;
    nodes_msg_ = nodes_msg;
    goal_handle_ = goal_handle;

    feedback_.reset(new NavigateThroughNodes::Feedback);
    result_.reset(new NavigateThroughNodes::Result);
  }

  virtual void configure() = 0;

  virtual void execute() = 0;

  virtual bool cancel() = 0;

  void update_driving_state(bool driving)
  {
    current_state_->set_parameter(&SafeState::OrderState::driving, driving);
  }

  bool is_driving() const { return current_state_->get().driving; }

protected:
  std::shared_ptr<GoalHandleNavigateThroughNodes> goal_handle_;
  std::shared_ptr<NavigateThroughNodes::Feedback> feedback_;
  std::shared_ptr<NavigateThroughNodes::Result> result_;

  std::vector<vda5050_msgs::msg::Edge> edges_msg_;
  std::vector<vda5050_msgs::msg::Node> nodes_msg_;
};

}  // namespace adapter
