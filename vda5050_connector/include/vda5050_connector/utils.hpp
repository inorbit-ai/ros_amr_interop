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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/**
 * ROS msgs / srvs
 */
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vda5050_msgs/msg/node.hpp"

namespace vda5050_connector
{
namespace utils
{
/**
 * Declare and read a bool parameter.
 * @param node Node pointer.
 * @param param_name Parameter name to search.
 * @param default_val Default value to set if param_name is not found (default: false).
 */
bool read_bool_parameter(
  rclcpp::Node *, const std::string & param_name, const bool default_val = false);

/**
 * Declare and read a string parameter.
 * @param node Node pointer.
 * @param param_name Parameter name to search.
 * @param default_val Default value to set if param_name is not found (default: "").
 */
std::string read_str_parameter(
  rclcpp::Node *, const std::string & param_name, const std::string & default_val = "");

/**
 * Declare and read a string array parameter.
 * @param node Node pointer.
 * @param param_name Parameter name to search.
 * @param default_val Default value to set if param_name is not found (default: {}).
 */
std::vector<std::string> read_str_array_parameter(
  rclcpp::Node *, const std::string & param_name,
  const std::vector<std::string> & default_val = {});

/**
 * @brief Converts a camelCase string into snake case.
 * @param camelCaseMsg camelCase single word.
 * @return snake_case single word.
 */
std::string to_snake_case(const std::string & camel_case_msg);

/**
 * @brief Get a pose stamped msg from a VDANode msg object.
 * @param node VDA5050 node to get the pose from.
 * @return Pose stamped msg.
 */
geometry_msgs::msg::PoseStamped get_pose_from_node(const vda5050_msgs::msg::Node & node);
}  // namespace utils
}  // namespace vda5050_connector
