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
#include "vda5050_connector/action/navigate_through_nodes.hpp"
#include "vda5050_connector/srv/extend_navigation.hpp"
#include "vda5050_msgs/msg/edge.hpp"
#include "vda5050_msgs/msg/node.hpp"

namespace adapter
{
using NavigateThroughNodes = vda5050_connector::action::NavigateThroughNodes;
using GoalHandleNavigateThroughNodes = rclcpp_action::ServerGoalHandle<NavigateThroughNodes>;
using ExtendNavigation = vda5050_connector::srv::ExtendNavigation;

/**
 * @brief The NavThroughNodes handler is in charge of sending the robot to navigate through
 * a sequence of nodes and edges.
 */

class NavThroughNodes : public Handler
{
public:
  /**
   * @brief Reset the nav through nodes handler.
   * @param edges_msg VDA5050 messages with the edge information.
   * @param nodes_msg VDA5050 messages with the node information.
   * @param goal_handle Pointer to the navigate through nodes goal.
   */
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

  /**
   * @brief Configure the nav through nodes handler (i.e. ROS interfaces, read parameters, etc).
   */
  virtual void configure() = 0;

  /**
   * @brief State machine for processing the navigate through nodes goal request.
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
  /**
   * @brief Set up the ExtendNavigation service server.
   * Call from configure() after node_ and robot_name_ are available.
   * Builds the service name from namespace, manufacturer_name param, and robot_name_.
   */
  void setupExtendNavigationService();

  /**
   * @brief Called after the navigation has been extended with new edges/nodes.
   * Override to react to the extension (e.g. rebuild task queues, extend planners).
   * @param old_edge_count The number of edges before the extension.
   */
  virtual void onNavigationExtended(size_t /*old_edge_count*/) {}

  // Goal handle for ROS2 action
  std::shared_ptr<GoalHandleNavigateThroughNodes> goal_handle_;
  std::shared_ptr<NavigateThroughNodes::Feedback> feedback_;
  std::shared_ptr<NavigateThroughNodes::Result> result_;

  // VDA5050 Edge and Node messages
  std::vector<vda5050_msgs::msg::Edge> edges_msg_;
  std::vector<vda5050_msgs::msg::Node> nodes_msg_;

private:
  rclcpp::Service<ExtendNavigation>::SharedPtr extend_navigation_srv_;

  /**
   * @brief Callback for the ExtendNavigation service.
   * Validates the request, appends new edges/nodes to the internal vectors,
   * and calls onNavigationExtended() for subclass-specific handling.
   */
  void extendNavigationCallback(
      const std::shared_ptr<ExtendNavigation::Request> request,
      std::shared_ptr<ExtendNavigation::Response> response);
};

}  // namespace adapter
