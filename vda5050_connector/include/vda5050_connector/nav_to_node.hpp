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
#include "vda5050_connector/handler.hpp"

/**
 * ROS related dependencies / headers
 */
#include "rclcpp_action/rclcpp_action.hpp"

/**
 * ROS msgs / services
 */
#include "vda5050_connector/action/navigate_to_node.hpp"
#include "vda5050_msgs/msg/node.hpp"

namespace adapter
{
using NavigateToNode = vda5050_connector::action::NavigateToNode;
using GoalHandleNavigateToNode = rclcpp_action::ServerGoalHandle<NavigateToNode>;

/**
 * @brief The NavToNode handler is in charge of sending the robot to navigate to a given pose.
 */

class NavToNode : public Handler
{
public:
  /**
   * @brief Reset the nav to node handler.
   * @param node_msg VDA5050 message with the new node information.
   * @param goal_handle Pointer to the navigate to node goal.
   */
  virtual void reset(
    const vda5050_msgs::msg::Node & node_msg,
    const std::shared_ptr<GoalHandleNavigateToNode> goal_handle)
  {
    node_msg_ = node_msg;
    goal_handle_ = goal_handle;

    feedback_.reset(new NavigateToNode::Feedback);
    result_.reset(new NavigateToNode::Result);
  }

  /**
   * @brief Configure the nav to node handler (i.e. ROS interfaces, read parameters, etc).
   */
  virtual void configure() = 0;

  /**
   * @brief State machine for processing the navigate to node goal request.
   */
  virtual void execute() = 0;

  /**
   * @brief Cancel the current navigation goal.
   * @return true if successfully cancelled, false otherwise.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Update the current driving state.
   */
  void update_driving_state(bool driving)
  {
    current_state_->set_parameter(&SafeState::OrderState::driving, driving);
  }

  /**
   * @brief Get the driving state.
   * @return true if the robot is driving, false otherwise.
   */
  bool is_driving() const { return current_state_->get().driving; }

protected:
  // Goal handle for ROS2 action
  std::shared_ptr<GoalHandleNavigateToNode> goal_handle_;
  std::shared_ptr<NavigateToNode::Feedback> feedback_;
  std::shared_ptr<NavigateToNode::Result> result_;

  // VDA5050 Node message
  vda5050_msgs::msg::Node node_msg_;
};

}  // namespace adapter
