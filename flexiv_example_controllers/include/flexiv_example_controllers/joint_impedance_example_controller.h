// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace flexiv_example_controllers {

class JointImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

 private:
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};

  std::array<double, 7> planned_trajectory{0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0};
  double time_total_trajectory = 5.0; // second

  // impedance controller parameters
  static constexpr double kDeltaTauMax{1.0};
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double angle_{0.0};
  double vel_current_{0.0};

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
};

}  // namespace franka_example_controllers
