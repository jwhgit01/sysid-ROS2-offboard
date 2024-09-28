/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Test Manual Mode";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _actuator_controls = std::make_shared<px4_ros2::DirectActuatorsSetpointType >(*this);
  }

  static constexpr int kMaxNumServos = px4_msgs::msg::ActuatorServos::NUM_CONTROLS;
  static constexpr int kMaxNumMotors = px4_msgs::msg::ActuatorMotors::NUM_CONTROLS;

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    Eigen::Matrix<float, kMaxNumServos, 1> servo_commands;
    Eigen::Matrix<float, kMaxNumMotors, 1> motor_commands;
    servo_commands(0) = _manual_control_input->roll();
    servo_commands(1) = _manual_control_input->pitch();
    servo_commands(2) = _manual_control_input->yaw();
    motor_commands(0) = _manual_control_input->throttle();
    _actuator_controls->updateServos(servo_commands);
    _actuator_controls->updateMotors(motor_commands);
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> _actuator_controls;
};
