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

#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

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

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  
  ros::Duration elapsed_time_;

  // Franka Emika Panda serial manipulator
  DQ_SerialManipulatorMDH franka_ = FrankaEmikaPandaRobot::kinematics();

  DQ_QPOASESSolver solver_;
  DQ_ClassicQPController translation_controller_ = DQ_ClassicQPController(&franka_, &solver_);
  // DQ_ClassicQPController translation_controller_ = DQ_ClassicQPController();

  DQ_QPOASESSolver solver_stop_;
  DQ_ClassicQPController pose_controller_ = DQ_ClassicQPController(&franka_, &solver_);

  // Maybe I could get this values directly from libfranka, but anyway
  VectorXd q_minus_;
  VectorXd q_plus_;

  VectorXd vel_minus_;
  VectorXd vel_plus_;

  // Define the floor
  DQ n_floor_;
  DQ d_floor_;
  DQ pi_floor_;

  // Define the pose of the camera
  DQ pose_camera_;

  // Initialize the variables that we be used later
  double tau_;
  int counter_; // count the number of cycles
  int decide_td_; //aux variable to choose the td
  DQ td_, td1_, td2_; // the desired position of the robot

};

}  // namespace franka_example_controllers
