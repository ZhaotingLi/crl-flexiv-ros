// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <flexiv_example_controllers/joint_impedance_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace flexiv_example_controllers {

bool JointImpedanceExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  // use the position joint interface of robot_hardware module
  effort_joint_interface_ = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointImpedanceExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointImpedanceExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointImpedanceExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }


  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }


  effort_joint_handles_.resize(7);    // defined in joint_position_example_controller.h
  for (size_t i = 0; i < 7; ++i) {
    try {
      effort_joint_handles_[i] = effort_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -0.69, 0, 1.57, 0, 0.69, 0}};
  // std::array<double, 7> q_start{{0, -M_PI_4, 0.25 * M_PI_4, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    std::cout<< i << " position_joint_handles_: " << effort_joint_handles_[i].getPosition() <<std::endl;

    if (std::abs(effort_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  robot_model_sub_ = node_handle.subscribe("/robot_model", 1, &JointImpedanceExampleController::robotmodel_callback, this);
  gravity_tau.resize(7);
  coriolis_tau.resize(7);
  // tau_des.resize(7);

  return true;
}


// not used currently, plan to use in future.
void JointImpedanceExampleController::robotmodel_callback(const flexiv_msgs::RobotModel& msg){
  // receive the model info, which is published in flexiv_hardware_interface: robot_model_pub_
  for(int i = 0; i < 7; i ++){
    gravity_tau(i) = msg.gravity[i];
    coriolis_tau(i) = msg.coriolis_tau[i];
    tau_des[i] = msg.tau_des[i];
  }
  // std::cout<<"model info received"<<std::endl;
}

void JointImpedanceExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = effort_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

// // one joint movement for the collection of friction compensation data
void JointImpedanceExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

    std::array<double, 7> q_des;   // desired joint state

    elapsed_time_ += period;

    // double time_now = elapsed_time_.toSec();
    // double time_0_to_20 = int(time_now) % 20 + time_now - int(time_now) ;


    // if(time_total_trajectory > time_0_to_20){
    //     for (size_t i = 0; i < 7; ++i) {
    //       q_des[i] = (planned_trajectory[i] - initial_pose_[i]) * time_0_to_20 / time_total_trajectory + initial_pose_[i];
    //     }
    // }else if(time_0_to_20 < 2 * time_total_trajectory){
    //     for (size_t i = 0; i < 7; ++i) {
    //       q_des[i] = planned_trajectory[i];
    //     }
    // }else if(time_0_to_20 < 3 * time_total_trajectory){
    //     for (size_t i = 0; i < 7; ++i) {
    //       q_des[i] = (initial_pose_[i] - planned_trajectory[i]) * (time_0_to_20 - 2 * time_total_trajectory) / time_total_trajectory + planned_trajectory[i];
    //     }
    // }else{
    //     for (size_t i = 0; i < 7; ++i) {
    //       q_des[i] = initial_pose_[i];
    //     }
    // }
    
    //// desired joint state, during this test, just keep the home position
    for (size_t i = 0; i < 7; ++i) {
        if (i == 4) {
          // q_des[i] = initial_pose_[i] + delta_angle;
          q_des[i] = initial_pose_[i];
        } else {
        // position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
          q_des[i] = initial_pose_[i];
        }
    }

    // read current joint position
    std::array<double, 7> q_now;
    for (size_t i = 0; i < 7; ++i) {
      q_now[i] = effort_joint_handles_[i].getPosition();
    }

    // read current joint velocity
    std::array<double, 7> dq_now;
    for (size_t i = 0; i < 7; ++i) {
      dq_now[i] = effort_joint_handles_[i].getVelocity();
    }

    // caculate current joint command
    std::array<double, 7> tau_d_calculated;
    for (size_t i = 0; i < 7; ++i) {
      tau_d_calculated[i] = k_gains_[i] * (q_des[i] - q_now[i]) 
                             + d_gains_[i] * (0 - dq_now[i]); //+ gravity_tau(i) + coriolis_tau(i);
    }

    
    for (size_t i = 0; i < 7; ++i) {
      effort_joint_handles_[i].setCommand(tau_d_calculated[i]);
    }
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(flexiv_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerBase)
