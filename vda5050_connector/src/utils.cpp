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
#include "vda5050_connector/utils.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace vda5050_connector
{
namespace utils
{
bool read_bool_parameter(
  rclcpp::Node * node, const std::string & param_name, const bool default_val)
{
  bool temp;
  node->declare_parameter<bool>(param_name, default_val);
  node->get_parameter(param_name, temp);
  return temp;
}

std::string read_str_parameter(
  rclcpp::Node * node, const std::string & param_name, const std::string & default_val)
{
  std::string temp;
  node->declare_parameter<std::string>(param_name, default_val);
  node->get_parameter(param_name, temp);
  return temp;
}

std::vector<std::string> read_str_array_parameter(
  rclcpp::Node * node, const std::string & param_name, const std::vector<std::string> & default_val)
{
  std::vector<std::string> temp;
  node->declare_parameter<std::vector<std::string>>(param_name, default_val);
  node->get_parameter(param_name, temp);
  return temp;
}

std::string to_snake_case(const std::string & camel_case_msg)
{
  std::string snake_case_msg;
  for (const auto & letter : camel_case_msg) {
    snake_case_msg += std::isupper(letter) ? "_" : "";
    snake_case_msg += std::tolower(letter);
  }
  return snake_case_msg;
}

geometry_msgs::msg::PoseStamped get_pose_from_node(const vda5050_msgs::msg::Node & node)
{
  geometry_msgs::msg::PoseStamped pose_stamped;

  pose_stamped.header.frame_id = node.node_position.map_id;

  pose_stamped.pose.position.x = node.node_position.x;
  pose_stamped.pose.position.y = node.node_position.y;

  tf2::Quaternion q;
  q.setRPY(0, 0, node.node_position.theta);
  pose_stamped.pose.orientation.x = q.getX();
  pose_stamped.pose.orientation.y = q.getY();
  pose_stamped.pose.orientation.z = q.getZ();
  pose_stamped.pose.orientation.w = q.getW();
  return pose_stamped;
}
}  // namespace utils
}  // namespace vda5050_connector
