// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_behaviors/plugins/back_up.hpp"

namespace nav2_behaviors
{

void BackUp::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 只在有明确线速度时更新符号，避免停止指令覆盖符号
  if (std::fabs(msg->linear.x) > 1e-3) {
    last_linear_x_sign_ = (msg->linear.x > 0.0) ? 1.0 : -1.0;
  }
}

Status BackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  // 延迟初始化订阅（node_ 在基类构造后才可用）
  if (!cmd_vel_sub_) {
    cmd_vel_sub_ = node_.lock()->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&BackUp::cmdVelCallback, this, std::placeholders::_1));
  }

  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Backing up in Y and Z not supported, will only move in X.");
    return Status::FAILED;
  }

  // 用最近速度的符号决定"倒车"方向（与上次运动方向相反）
  command_x_ = -last_linear_x_sign_ * std::fabs(command->target.x);
  command_speed_ = -last_linear_x_sign_ * std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUp, nav2_core::Behavior)
