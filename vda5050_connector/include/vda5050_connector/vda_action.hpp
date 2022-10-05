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
#include <array>
#include <initializer_list>

#include "vda5050_connector/handler.hpp"

/**
 * ROS related dependencies / headers
 */
#include "rclcpp_action/rclcpp_action.hpp"

/**
 * ROS msgs / srvs
 */
#include "vda5050_connector/action/process_vda_action.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/action_parameter_definition.hpp"
#include "vda5050_msgs/msg/agv_action.hpp"
#include "vda5050_msgs/msg/current_action.hpp"

namespace adapter
{
using ActionParameterDefinition = vda5050_msgs::msg::ActionParameterDefinition;
using AGVAction = vda5050_msgs::msg::AGVAction;
using ProcessVDAAction = vda5050_connector::action::ProcessVDAAction;
using GoalHandleProcessVDAAction = rclcpp_action::ServerGoalHandle<ProcessVDAAction>;

/**
 * @brief The VDAAction handler is in charge of managing VDA5050 actions.
 */

class VDAAction : public Handler
{
public:
  // VDA5050 action states
  enum class STATES { WAITING = 0, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED };

  /**
   * @brief Return a string representing the action_state.
   * @param action_state to convert to string.
   */
  static std::string action_state_str(STATES action_state)
  {
    using CurrentAction = vda5050_msgs::msg::CurrentAction;
    std::array<std::string, 6> action_states_str{
      CurrentAction::WAITING, CurrentAction::INITIALIZING, CurrentAction::RUNNING,
      CurrentAction::PAUSED,  CurrentAction::FINISHED,     CurrentAction::FAILED};
    return action_states_str.at((int)action_state);
  }

  /**
   * @brief Reset the VDA action handler.
   * @param action_msg VDA5050 action message.
   * @param goal_handle Handle to the action to reset.
   */
  virtual void reset(
    const vda5050_msgs::msg::Action & action_msg,
    const std::shared_ptr<GoalHandleProcessVDAAction> goal_handle)
  {
    action_msg_ = action_msg;
    goal_handle_ = goal_handle;

    action_state_ = STATES::WAITING;

    feedback_.reset(new ProcessVDAAction::Feedback);
    result_.reset(new ProcessVDAAction::Result);

    current_action_msg_ = vda5050_msgs::msg::CurrentAction();
    current_action_msg_.action_id = action_msg_.action_id;
    current_action_msg_.action_description = action_msg_.action_description;
  }

  /**
   * @brief Configure the VDA action handler (i.e. ROS interfaces, read parameters, etc).
   */
  virtual void configure() = 0;

  /**
   * @brief State machine for processing VDA Actions.
   */
  virtual void execute()
  {
    STATES action_state = get_action_state();
    bool executing = true;
    while (executing) {
      switch (action_state) {
        case STATES::WAITING:
          update_action_state(STATES::INITIALIZING);  // Initiates action_state machine
          break;
        case STATES::INITIALIZING:
          initialize();
          break;
        case STATES::RUNNING:
          run();
          break;
        case STATES::PAUSED:
          pause();
          break;
        case STATES::FAILED:
          fail();
          executing = false;
          break;
        case STATES::FINISHED:
          finish();
          executing = false;
          break;
        default:
          break;
      }

      action_state = get_action_state();
    }
  }

  /**
   * @brief Get the current action_state.
   * @return STATES action_state.
   */
  STATES get_action_state() const { return action_state_; }

  /**
   * @brief Update the state of get action and publish its feedback.
   * @param action_state Current action_state.
   */
  virtual void update_action_state(STATES action_state)
  {
    action_state_ = action_state;
    current_action_msg_.action_status = action_state_str(action_state_);
    result_->result = current_action_msg_;
    feedback_->current_action = current_action_msg_;
    goal_handle_->publish_feedback(feedback_);
  }

  /**
   * @brief Set the information relative to the action support.
   * @param type VDA action type.
   * @param description Description of the action.
   * @param result_description Description of the action's result.
   */
  void set_supported_info(
    const std::string & type, const std::string & description,
    const std::string & result_description)
  {
    support_action_.action_type = type;
    support_action_.action_description = description;
    support_action_.result_description = result_description;
  }

  /**
   * @brief Set the supported scopes ("INSTANT", "NODE", "EDGE").
   * @param instant true if instant scope is supported.
   * @param node true if node scope is supported.
   * @param edge true if edge scope is supported.
   */
  void set_supported_scopes(const bool instant, const bool node, const bool edge)
  {
    support_action_.action_scopes.clear();
    if (instant) {
      support_action_.action_scopes.push_back(AGVAction::INSTANT);
    }

    if (node) {
      support_action_.action_scopes.push_back(AGVAction::NODE);
    }

    if (edge) {
      support_action_.action_scopes.push_back(AGVAction::EDGE);
    }
  }

  /**
   * @brief Set a new supported parameter definition.
   * @param key Parameter key name.
   * @param data_type value data type: BOOL, NUMBER, INTEGER, FLOAT, STRING, OBJECT, ARRAY.
   * @param description Parameter description.
   * @param is_optional true if the parameter is optional.
   */
  void set_supported_parameter(
    const std::string & key, const std::string & data_type, const std::string & description,
    const bool is_optional)
  {
    ActionParameterDefinition parameter_definition;
    parameter_definition.key = key;
    parameter_definition.value_data_type = data_type;
    parameter_definition.description = description;
    parameter_definition.is_optional = is_optional;

    support_action_.action_parameters.push_back(parameter_definition);
  }

  /**
   * @brief Get the supported action info object.
   * @return AGVAction with information about the action's support.
   */
  AGVAction get_supported_action_info() const { return support_action_; }

  // STATE functions

  /**
   * @brief State function: initializing.
   */
  virtual void initialize() { update_action_state(STATES::INITIALIZING); }

  /**
   * @brief State function: running.
   */
  virtual void run() { update_action_state(STATES::RUNNING); }

  /**
   * @brief State function: paused.
   */
  virtual void pause() { update_action_state(STATES::PAUSED); }

  /**
   * @brief State function: finished.
   */
  virtual void finish() { update_action_state(STATES::FINISHED); }

  /**
   * @brief State function: failed.
   */
  virtual void fail() { update_action_state(STATES::FAILED); }

  /**
   * @brief Cancel the current VDA action.
   * @return true if the action was cancelled.
   */
  virtual bool cancel()
  {
    update_action_state(STATES::FAILED);
    return true;
  }

protected:
  // Current action state
  STATES action_state_{STATES::WAITING};

  // Support action information
  AGVAction support_action_;

  // Goal handle for ROS2 action
  std::shared_ptr<GoalHandleProcessVDAAction> goal_handle_;

  std::shared_ptr<ProcessVDAAction::Feedback> feedback_;
  std::shared_ptr<ProcessVDAAction::Result> result_;

  // VDA5050 action messages
  vda5050_msgs::msg::Action action_msg_;
  vda5050_msgs::msg::CurrentAction current_action_msg_;
};

}  // namespace adapter
