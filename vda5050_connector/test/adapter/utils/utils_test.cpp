// BSD 3-Clause License
//
// Copyright (c) 2026 InOrbit, Inc.
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

#include "vda5050_connector/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "vda5050_msgs/msg/node.hpp"

namespace
{
vda5050_msgs::msg::Node node_at(double theta)
{
  vda5050_msgs::msg::Node node;
  node.node_position.x = 1.0;
  node.node_position.y = 2.0;
  node.node_position.theta = theta;
  node.node_position.map_id = "map";
  return node;
}
}  // namespace

/**
 * @brief A node with theta gives that yaw.
 */
TEST(GetPoseFromNode, UsesTheta)
{
  auto pose = vda5050_connector::utils::get_pose_from_node(node_at(M_PI / 2));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_NEAR(pose.pose.orientation.z, std::sin(M_PI / 4), 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, std::cos(M_PI / 4), 1e-9);
}

/**
 * @brief A node without theta (NaN) falls back to yaw 0 instead of a NaN quaternion.
 */
TEST(GetPoseFromNode, NoThetaFallsBackToYawZero)
{
  auto pose =
    vda5050_connector::utils::get_pose_from_node(node_at(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
}
