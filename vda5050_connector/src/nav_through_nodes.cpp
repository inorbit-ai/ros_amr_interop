// BSD 3-Clause License
//
// Copyright (c) 2022 InOrbit, Inc.
// Copyright (c) 2022 Clearpath Robotics, Inc.
// Copyright (c) 2026 Quantillion Technologies
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

#include "vda5050_connector/nav_through_nodes.hpp"

#include <rclcpp/logging.hpp>

void adapter::NavThroughNodes::setupExtendNavigationService()
{
  std::string manufacturer_name = node_->get_parameter("manufacturer_name").as_string();
  std::string base_interface_name = std::string(node_->get_namespace()) + "/"
      + manufacturer_name + "/" + robot_name_ + "/";

  extend_navigation_srv_ = node_->create_service<ExtendNavigation>(
      base_interface_name + "adapter/extend_navigation",
      std::bind(&NavThroughNodes::extendNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void adapter::NavThroughNodes::extendNavigationCallback(
    const std::shared_ptr<ExtendNavigation::Request> request,
    std::shared_ptr<ExtendNavigation::Response> response)
{
  if (!goal_handle_)
  {
    response->success = false;
    response->message = "No active navigation goal";
    return;
  }

  if (request->edges.empty() || request->nodes.size() < 2)
  {
    response->success = false;
    response->message = "Need at least 1 edge and 2 nodes (stitch + target)";
    return;
  }

  if (request->edges.size() != request->nodes.size() - 1)
  {
    response->success = false;
    response->message = "Size mismatch: edges must equal nodes - 1";
    return;
  }

  size_t old_edge_count = 0;
  {
    std::unique_lock lock(navigation_mutex_);

    if (nodes_msg_.empty())
    {
      response->success = false;
      response->message = "Cannot stitch: no existing navigation nodes available";
      RCLCPP_WARN(node_->get_logger(), "NavThroughNodes: %s", response->message.c_str());
      return;
    }

    // Validate stitch node: nodes[0] must match the last node we already have
    const auto& stitch_node = request->nodes[0];
    const auto& expected_node = nodes_msg_.back();
    if (stitch_node.node_id != expected_node.node_id
        || stitch_node.sequence_id != expected_node.sequence_id)
    {
      response->success = false;
      response->message = "Stitch node mismatch: expected node_id='"
          + expected_node.node_id + "' seq="
          + std::to_string(expected_node.sequence_id)
          + ", got node_id='" + stitch_node.node_id + "' seq="
          + std::to_string(stitch_node.sequence_id);
      RCLCPP_WARN(node_->get_logger(), "NavThroughNodes: %s", response->message.c_str());
      return;
    }

    old_edge_count = edges_msg_.size();

    // Append only the new edges and target nodes (skip stitch node)
    edges_msg_.insert(edges_msg_.end(), request->edges.begin(), request->edges.end());
    nodes_msg_.insert(nodes_msg_.end(), request->nodes.begin() + 1, request->nodes.end());
  }

  onNavigationExtended(old_edge_count);

  size_t new_count = request->nodes.size() - 1;
  response->success = true;
  response->message = "Extended navigation with " + std::to_string(new_count) + " new nodes";

  RCLCPP_INFO(node_->get_logger(),
              "NavThroughNodes: %s", response->message.c_str());
}
