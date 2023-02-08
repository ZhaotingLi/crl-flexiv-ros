// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <flexiv_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace flexiv_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  // use the position joint interface of robot_hardware module
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);    // defined in joint_position_example_controller.h
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -0.69, 0, 1.57, 0, 0.69, 0}};
  // std::array<double, 7> q_start{{0, -M_PI_4, 0.25 * M_PI_4, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    std::cout<< i << " position_joint_handles_: " << position_joint_handles_[i].getPosition() <<std::endl;

    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

    //   double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;

    elapsed_time_ += period;

    double magnitude_delta = 0.2;  // {0.2, 0.45, 0.6 }analyse the relationship between the discrepency of the external torque and dq
    double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * magnitude_delta;


    double time_now = 1 / 2.0 * elapsed_time_.toSec();
    double time_0_to_12 = int(time_now) % 12 + time_now - int(time_now) ;
    if(time_0_to_12 < 1){
    delta_angle = M_PI / 16 * (1 - std::cos(M_PI * time_now)) * magnitude_delta;
    }else if(time_0_to_12 < 2){
    delta_angle = M_PI / 8 * magnitude_delta;
    }else if(time_0_to_12 < 3){
    delta_angle = M_PI / 8 * magnitude_delta + M_PI / 16 * (1 - std::cos(M_PI * (time_now-2))) * magnitude_delta;
    }else if(time_0_to_12 < 4){
    delta_angle = M_PI / 4 * magnitude_delta;
    }else if(time_0_to_12 < 5){
    delta_angle = M_PI / 4 * magnitude_delta + M_PI / 16 * (1 - std::cos(M_PI * (time_now - 4))) * magnitude_delta;
    }else if(time_0_to_12 < 6){
    delta_angle = M_PI * 3 / 8 * magnitude_delta;
    }else if(time_0_to_12 < 7){
    delta_angle = M_PI / 4 * magnitude_delta + M_PI / 16 * (1 - std::cos(M_PI * (time_now - 5))) * magnitude_delta;
    }else if(time_0_to_12 < 8){
    delta_angle = M_PI * 2 / 8 * magnitude_delta;
    }else if(time_0_to_12 < 9){
    delta_angle = M_PI / 8 * magnitude_delta + M_PI / 16 * (1 - std::cos(M_PI * (time_now - 7))) * magnitude_delta;
    }else if(time_0_to_12 < 10){
    delta_angle = M_PI * 1 / 8 * magnitude_delta;
    }else if(time_0_to_12 < 11){
    delta_angle = M_PI / 16 * (1 - std::cos(M_PI * (time_now - 9))) * magnitude_delta;
    }else{
    delta_angle = 0;
    }
    
    for (size_t i = 0; i < 7; ++i) {
        if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
        } else {
        // position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
        position_joint_handles_[i].setCommand(initial_pose_[i]);
        }
    }

  // stay static so that it's easier to find the external torque
//   for (size_t i = 0; i < 7; ++i) {
//     position_joint_handles_[i].setCommand(initial_pose_[i]);
//   }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(flexiv_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
