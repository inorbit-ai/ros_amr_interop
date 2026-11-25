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

/**
 * C++ Libraries / header
 */
#include <chrono>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "vda5050_connector/nav_through_nodes.hpp"

namespace test
{
/**
 * @brief Keeps the goal executing until it is canceled.
 */
class StubNavThroughNodes : public adapter::NavThroughNodes
{
public:
  StubNavThroughNodes() = default;
  void configure() override { setupExtendNavigationService(); }
  void execute() override
  {
    const auto goal_handle = goal_handle_;
    while (!goal_handle->is_canceling()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    goal_handle->canceled(result_);
  }
  bool cancel() override { return true; }
};

}  // namespace test

PLUGINLIB_EXPORT_CLASS(test::StubNavThroughNodes, adapter::NavThroughNodes)
