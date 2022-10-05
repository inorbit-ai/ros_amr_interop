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
#include <iostream>
#include <shared_mutex>
#include <utility>

/**
 * ROS related dependencies / headers
 */
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_connector/visibility_control.h"

/**
 * ROS msgs / services
 */
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/info.hpp"
#include "vda5050_msgs/msg/load.hpp"
#include "vda5050_msgs/msg/order_state.hpp"

namespace adapter
{
/**
 * @brief The Handler class is the plugin encapsulation for the CPP Adapter implementation.
 */

class SafeState
{
public:
  using OrderState = vda5050_msgs::msg::OrderState;

  /**
   * @brief Return the OrderState.
   * Multiple threads/readers can read the counter's value at the same time.
   */
  OrderState get() const
  {
    std::shared_lock lock(mutex);
    return order_state_;
  }

  /**
   * @brief Set a value in a order_state's member.
   * Only one thread/writer can modify the order_state.
   * @tparam Type of vda5050_msgs/OrderState member.
   * @tparam Type of value to store on member.
   * @param member Member address to modify on current_state. (e.g &OrderState::driving or &OrderState::agv_position).
   * @param value Value to set on order_state's member (e.g false or vda5050_msgs/AGVPosition).
   */
  template <class T, class U>
  void set_parameter(T OrderState::*member, U && value)
  {
    try {
      std::unique_lock lock(mutex);
      order_state_.*member = std::forward<U>(value);
    } catch (const std::exception & e) {
      std::cout << "Wrong assignment of value on order_state member: " << e.what() << std::endl;
    }
  }

  /**
   * @brief Add a new information msg into the informations array of the order state
   * Only one thread/writer can modify the order_state.
   */
  void add_information(const vda5050_msgs::msg::Info & info)
  {
    std::unique_lock lock(mutex);
    order_state_.informations.push_back(info);
  }

  /**
* @brief Add a new load msg into the loads array of the order state
* Only one thread/writer can modify the order_state.
*/
  void add_load(const vda5050_msgs::msg::Load & load)
  {
    std::unique_lock lock(mutex);
    order_state_.loads.push_back(load);
  }

  /**
* @brief Add a new error msg into the errors array of the order state
* Only one thread/writer can modify the order_state.
*/
  void add_error(const vda5050_msgs::msg::Error & error)
  {
    std::unique_lock lock(mutex);
    order_state_.errors.push_back(error);
  }

  /**
   * @brief Clear the order state arrays (load, informations and errors).
   * Only one thread/writer can clear the order_state.
   */
  void clear()
  {
    std::unique_lock lock(mutex);
    order_state_.loads.clear();
    order_state_.informations.clear();
    order_state_.errors.clear();
  }

  /**
   * @brief Reset the complete OrderState msg.
   * Only one thread/writer can reset the order_state.
   */
  void reset()
  {
    std::unique_lock lock(mutex);
    order_state_ = OrderState();
  }

private:
  OrderState order_state_;
  mutable std::shared_mutex mutex;
};

class Handler
{
public:
  Handler() {}

  virtual ~Handler() {}

  /**
   * @brief Fill the handler with parameters from the adapter node.
   * @param node Pointer to the node.
   * @param current_state Pointer to the global current state.
   * @param robot_name A string with the robot name.
   */
  virtual void compose(
    rclcpp::Node * node, SafeState * current_state, const std::string & robot_name)
  {
    if (node == nullptr || current_state == nullptr) {
      throw std::runtime_error(
        "Either node or current_state pass pointers are nullptr when composing handler.");
    }
    node_ = node;
    current_state_ = current_state;
    robot_name_ = robot_name;
  }

  /**
   * @brief Configure the handler plugin (i.e. ROS interfaces, read parameters, etc).
   *
   */
  virtual void configure() = 0;

  /**
   * @brief Execute the handler plugin (i.e. StateMachine, process, etc).
   */
  virtual void execute() = 0;

protected:
  // Node parameters
  rclcpp::Node * node_{nullptr};
  SafeState * current_state_{nullptr};
  std::string robot_name_;
};

}  // namespace adapter
