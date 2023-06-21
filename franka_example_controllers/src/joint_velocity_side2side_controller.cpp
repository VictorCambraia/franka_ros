// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_side2side_controller.h>

#include <cmath>
#include <thread>
#include <fstream>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

namespace franka_example_controllers {

bool JointVelocitySide2SideController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  // ------------------ HERE I WILL ADD MY CODE OF INITIALIZATION -----------------------
  // I need to check how to get the initial pose....

  // What variables I have available?
  // From the class I have velocity joint interface_, and velocity joint handles_
  // With this variables I can get: state_interface, with this one I can get state_handle, and with this one the robot joint values

  //------------------- Robot definition--------------------------
  //---------- Franka Emika Panda serial manipulator
  // franka_ = FrankaEmikaPandaRobot::kinematics();

  DQ franka_pose = 1 + E_*(0.035*k_);
  DQ new_base_robot = (franka_.get_base_frame())*franka_pose*(1+0.5*E_*(-0.07*k_));
  // DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(DQ(1));
  franka_.set_reference_frame(new_base_robot);
  // //---------------------------------------------------------------

  // Defined previously in the .h file
  // translation_controller_ = DQ_ClassicQPController(&franka_, &solver_);
  translation_controller_.set_gain(2);
  translation_controller_.set_damping(1);
  translation_controller_.set_control_objective(DQ_robotics::Translation);

  // pose_controller_ = DQ_ClassicQPController(&franka_, &solver_stop_);
  pose_controller_.set_gain(1);
  pose_controller_.set_damping(1);
  pose_controller_.set_control_objective(DQ_robotics::Pose);

  // Maybe I could get this values directly from libfranka, but anyway
  // HEREEE CHANGES THESE VALUES AND RUN IN THE COPPELIA FIRST

  ROS_INFO_STREAM(" AQUIIII    1  ");

  q_minus_ = VectorXd(7);
  q_plus_ = VectorXd(7);
  q_minus_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_plus_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

  ROS_INFO_STREAM(" AQUIIII    2  ");

  vel_minus_ = VectorXd(7);
  vel_plus_ = VectorXd(7);
  vel_minus_ << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
  vel_plus_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

  ROS_INFO_STREAM(" AQUIIII    3  ");

  // Define the floor
  n_floor_ = 1*k_;
  d_floor_ = DQ(0);
  pi_floor_ = n_floor_ + E_*d_floor_;

  // Define the pose of the camera
  pose_camera_ = 1 + 0.5*E_*(0*i_ -0.7*j_ + 0*k_);

  // Initialize the variables that we be used later
  tau_ = 0.01; //It works as a delta_t
  counter_ = 0; // count the number of cycles
  decide_td_ = 0; //aux variable to choose the td
  td1_ =  0.3*i_ + 0.6*j_ + 0.5*k_;
  td2_ = -0.3*i_ + 0.6*j_ + 0.5*k_; // the desired position of the robot
  td_ = td1_;

  ROS_INFO_STREAM(" AQUIIII    4  ");

  return true;
}

void JointVelocitySide2SideController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  ROS_INFO_STREAM(" AQUIIII    5  ");
}

void JointVelocitySide2SideController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // ros::Duration time_max(8.0);
  // double omega_max = 0.1;
  // double cycle = std::floor(
  //     std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
  //                        time_max.toSec()));
  // double omega = cycle * omega_max / 2.0 *
  //                (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

  //   // COMO QUE DEBUGA ESSA MERDAAAAA
  //   // aparentemente pode colocar printf sim
  //   // if(elapsed_time_.sec % 2 == 0){
  //   //     ROS_INFO_STREAM("O omega eh " << omega);
  //   // }
    
  //   double vel = 0.01;

  // ------------------ HERE I WILL ADD MY CODE OF UPDATE -----------------------

  counter_++;

  // ROS_INFO_STREAM(" AQUIIII    6  ");

  int i;
  int n_space = franka_.get_dim_configuration_space();
  ROS_INFO_STREAM(" AQUIIII    7  ");
  VectorXd q(n_space); //joint values
  franka::RobotState robot_state = state_handle_->getRobotState();
  // Not sure if it is going to work
  ROS_INFO_STREAM(" AQUIIII    7.5  ");
  for(i=0;i<n_space;i++){
    q[i] = robot_state.q_d[i];
  }
  // ROS_INFO_STREAM(" AQUIIII    8  ");
  int joint_counter; //aux variable for the for
  // Iterate it for every joint of the robot
  DQ t, x;
  

  x = franka_.fkm(q);
  t = translation(x);
  MatrixXd Jx = franka_.pose_jacobian(q);
  // Get the robot's translation Jacobian
  MatrixXd Jt = franka_.translation_jacobian(Jx, x);

  MatrixXd W_q(14,7);
  W_q <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
  VectorXd w_q(14);
  w_q << -1*(q_minus_ - q), 1*(q_plus_ - q);

  // Define the inequalities regarding the max and min velocities
  MatrixXd W_vel(14,7);
  W_vel <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
  VectorXd w_vel(14);
  w_vel << -1*vel_minus_, 1*vel_plus_;

  MatrixXd A;
  VectorXd b;

  // ROS_INFO_STREAM(" AQUIIII    9  ");
  
  A.resize(W_q.rows() + W_vel.rows(), W_q.cols());
  b.resize(w_q.size() + w_vel.size());

  A << W_q, W_vel;
  b << w_q, w_vel;

  // Maybe add the inequality regarding the floor....
  // Update the linear inequalities in the controller
  translation_controller_.set_inequality_constraint(A, b);
  // Get the next control signal [rad/s]
  // We put as objective the current position, so the robot try to stop
  VectorXd u = translation_controller_.compute_setpoint_control_signal(q,vec4(td_));  

  // Check the error
  VectorXd e = vec4(t - td_);
  if(e.norm() < 0.05){
    if(decide_td_ == 0){
      td_ = td2_;
      decide_td_ = 1;
    }
    else{
      td_ = td1_;
      decide_td_ = 0;
    }
  }

  for(i=0;i<n_space;i++){
    velocity_joint_handles_[i].setCommand(u[i]);
  }

  // for (auto joint_handle : velocity_joint_handles_) {
  //   // joint_handle.setCommand(omega);
  //   joint_handle.setCommand(vel);
  // }
  ROS_INFO_STREAM(" AQUIIII    10    ");
  ROS_INFO_STREAM(" Valor de t eh    " << t);
}

void JointVelocitySide2SideController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocitySide2SideController,
                       controller_interface::ControllerBase)
