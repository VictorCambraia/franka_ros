// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include "franka_example_controllers/JacobianHMP.h"

#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

using namespace DQ_robotics;
using namespace Eigen;

namespace franka_example_controllers {

class JointVelocitySide2SideController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  void cb_update_hmp(const std_msgs::String::ConstPtr& msg);
  void cb_stop_robot(const std_msgs::Int32::ConstPtr& msg);

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  
  ros::Duration elapsed_time_;

  // Previous global variables in the other code
  std::string str_poses_human_;
  int refresh_pose_ = 0;
  int stop_robot_;

  // Franka Emika Panda serial manipulator
  DQ_SerialManipulatorMDH franka_ = FrankaEmikaPandaRobot::kinematics();

  DQ_QPOASESSolver solver_;
  DQ_ClassicQPController translation_controller_ = DQ_ClassicQPController(&franka_, &solver_);
  // DQ_ClassicQPController translation_controller_ = DQ_ClassicQPController();

  DQ_QPOASESSolver solver_comp_;
  DQ_ClassicQPController translation_controller_comp_ = DQ_ClassicQPController(&franka_, &solver_comp_);

  DQ_QPOASESSolver solver_stop_;
  DQ_ClassicQPController pose_controller_ = DQ_ClassicQPController(&franka_, &solver_stop_);

  // Maybe I could get this values directly from libfranka, but anyway
  VectorXd q_minus_;
  VectorXd q_plus_;

  VectorXd vel_minus_;
  VectorXd vel_plus_;

  VectorXd accel_minus_;
  VectorXd accel_plus_;

  // Define the floor
  DQ n_floor_;
  DQ d_floor_;
  DQ pi_floor_;

  // Define parameters for the VFI
  // nd = 1;
  double nd_;
  double d_safe_floor_;

  // Define the pose of the camera
  DQ pose_camera_;

  // Initialize the variables that we be used later
  double tau_;
  int counter_; // count the number of cycles
  int decide_td_; //aux variable to choose the td
  DQ td_, td1_, td2_; // the desired position of the robot

  // I actually think that we dont need to define d_safe here, but okay
  VectorXd d_safe_hmp_;
  double K_error_value_;
  // Probably not the most correct way of doing the declaration
  JacobianHMP J_hmp_ = JacobianHMP(d_safe_hmp_, K_error_value_);
  MatrixXd poses_human_;
  VectorXd deviation_joints_;

  // Define the subscribers
  ros::NodeHandle n_pred_;
  ros::Subscriber sub_prediction_;

  ros::NodeHandle n_stop_;
  ros::Subscriber sub_stop_robot_;

  // Timers
  double time_prev_;

  // Delete later
  int counter_refresh_;

  // Filter variables
  VectorXd dq_filtered_;
  double K_filter_;

};

}  // namespace franka_example_controllers
