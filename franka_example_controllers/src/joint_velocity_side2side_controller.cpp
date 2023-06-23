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

// #include "franka_example_controllers/JacobianHMP.h"

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
  
  // Initialize the previous global variables
  stop_robot_ = 0;
  refresh_pose_ = 0;

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
  translation_controller_.set_gain(5);
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

  accel_minus_ = VectorXd(7);
  accel_plus_ = VectorXd(7);
  // accel_minus_ << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
  // accel_plus_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

  accel_minus_ << -15, -7.5, -10, -12.5, -15, -20, -20;
  accel_plus_ << 15, 7.5, 10, 12.5, 15, 20, 20;

  ROS_INFO_STREAM(" AQUIIII    3  ");

  // Define the floor
  n_floor_ = 1*k_;
  d_floor_ = DQ(0);
  pi_floor_ = n_floor_ + E_*d_floor_;
  nd_ = 0.5;
  d_safe_floor_ = 0.1;

  // Define the pose of the camera
  double ang_degree = -90;
  DQ rotation_camera = cos(ang_degree/2*(pi/180)) + sin(ang_degree/2*(pi/180))*(1*k_);
  DQ translation_camera = -0.35*i_ -0.2*j_ + 0.23*k_;
  // pose_camera_ = 1 + 0.5*E_*(0*i_ -0.7*j_ + 0*k_);
  pose_camera_ = rotation_camera + 0.5*E_*translation_camera*rotation_camera;

  // Initialize the variables that we be used later
  tau_ = 0.01; //It works as a delta_t
  counter_ = 0; // count the number of cycles
  decide_td_ = 0; //aux variable to choose the td
  td1_ = 0.6*i_ + 0.1*j_ + 0.4*k_;
  td2_ = 0.6*i_  -0.3*j_ + 0.4*k_; // the desired position of the robot
  td_ = td1_;

  // Define the safety parameters for the human
  d_safe_hmp_ = VectorXd(3);
  d_safe_hmp_ << 0.3, 0.4, 0.6;

  //If you want to change the K_error factor from 0.2 to other value, write it here
  double K_error = 0.1;

  // Get the object from the class that will return the jacobian for us
  J_hmp_ = JacobianHMP(d_safe_hmp_, K_error);

  ROS_INFO_STREAM("O d safe arm ficou sendo  " << J_hmp_.d_safe_arm);

  //Initialize the variable that will store the human poses
  int n_rows = J_hmp_.num_poses*J_hmp_.num_joints_per_pose;
  int n_cols = J_hmp_.num_dim;
  poses_human_ = 100*MatrixXd::Ones(n_rows, n_cols);
  deviation_joints_ = VectorXd::Zero(n_rows);

  counter_refresh_ = 0;
  // ROS_INFO_STREAM(" AQUIIII    4  ");

  // HERE Initialize the subscribers and publishers(if there is one)
  sub_prediction_ = n_pred_.subscribe("prediction_human", 1000, &JointVelocitySide2SideController::cb_update_hmp, this);
  sub_stop_robot_ = n_stop_.subscribe("stop_robot", 1000, &JointVelocitySide2SideController::cb_stop_robot, this);


  return true;
}

void JointVelocitySide2SideController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  time_prev_ = ros::Time::now().toSec();
  // ROS_INFO_STREAM(" AQUIIII    5  ");
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

  // ros::Time time_ros;
  // double time_now, time_diff;
  // time_now = ros::Time::now().toSec();
  // time_diff = time_now - time_prev_;
  // time_prev_ = time_now;
  // if(counter_%1000 == 0){
  //   ROS_INFO_STREAM(" o peroiodo foi  " << time_diff*1000 << "  \n");
  // }

  counter_++;

  if(refresh_pose_ == 1){
    // Check if this is going to work
    counter_refresh_++;
    // if(counter_refresh_ % 90 == 0){
    if(counter_refresh_ % 1 == 0){
      std::tie(poses_human_, deviation_joints_) = J_hmp_.transform_camera_points_2matrix(str_poses_human_, pose_camera_);
    }
    refresh_pose_ = 0;
    // std::cout << "AQUIII 2  " << std::endl;
  }

  // DELETE LATER THIS LITTLE PART
  MatrixXd pose_human_single(9,3);
  VectorXd deviation_joints_single(9);

  for(int j = 0; j < 9; j++){
    pose_human_single.row(j) << poses_human_.row(j);
    deviation_joints_single[j] = deviation_joints_[j];
  }
  // END OF THE DELETE PART


  // ROS_INFO_STREAM(" AQUIIII    6  ");

  int i;
  int n_space = franka_.get_dim_configuration_space();
  // ROS_INFO_STREAM(" AQUIIII    7  ");
  VectorXd q(n_space); //joint values
  VectorXd dq(n_space); //joint velocity values
  franka::RobotState robot_state = state_handle_->getRobotState();
  // Not sure if it is going to work
  // ROS_INFO_STREAM(" AQUIIII    7.5  ");
  for(i=0;i<n_space;i++){
    q[i] = robot_state.q_d[i];
    dq[i] = robot_state.dq_d[i];
  }

  MatrixXd A(0,0);
  VectorXd b(0);
  MatrixXd A_copy(0,0);
  VectorXd b_copy(0);

  DQ t, x;
  // ROS_INFO_STREAM(" AQUIIII    8  ");
  int joint_counter; //aux variable for the for
  // Iterate it for every joint of the robot
  for(joint_counter = n_space-1; joint_counter<n_space; joint_counter++){

    MatrixXd Jx = franka_.pose_jacobian(q,joint_counter);

    x = franka_.fkm(q, joint_counter);
    t = translation(x);
    // Get the robot's translation Jacobian
    MatrixXd Jt = franka_.translation_jacobian(Jx, x);

    // // Get the distance Jacobian from the cube/sphere (Lets see if it works without defining the size)
    // MatrixXd Jp_p(1, n);
    // Jp_p << franka.point_to_point_distance_jacobian(Jt, t, p_sphere), MatrixXd::Zero(1, n-1-joint_counter);
    // // Jp_p << franka.point_to_point_distance_jacobian(Jt, t, p_sphere);

    // // Get the distance to the sphere 
    // double d_p_p = double(norm(t-p_sphere));
    // double d_error = pow(d_p_p,2) - pow(d_safe,2);

    // std::cout << "AQUI 5  " << std::endl;
    // if(counter_%2000 == 0){
    //   // std::cout << "        JOINT NUMERO      " << joint_counter << std::endl;
    //   ROS_INFO_STREAM("        JOINT NUMERO      " << joint_counter << "\n");
    // }

    // The Jacobian for one or more poses
    MatrixXd Jp_p_aux;
    VectorXd d_error;
    // std::tie(Jp_p_aux, d_error) = J_hmp.get_jacobian_human(franka, Jt,t, points_hmp);
    // std::tie(Jp_p_aux, d_error) = J_hmp_.get_jacobian_human(franka_, Jt,t, poses_human_, deviation_joints_);
    // DELETE LATER
    // std::tie(Jp_p_aux, d_error) = J_hmp_.get_jacobian_human(franka_, Jt,t, pose_human_single, deviation_joints_single); //DELETE LATER

    //UNCOMMENT LATER
    std::tie(Jp_p_aux, d_error) = J_hmp_.get_3jacobians_human(franka_, Jt,t, poses_human_, deviation_joints_);

    // std::tie(Jp_p_aux, d_error) = J_hmp_.get_3jacobians_human(franka_, Jt,t, pose_human_single, deviation_joints_single); //DELETE LATER

    MatrixXd Jp_p(Jp_p_aux.rows(),n_space);
    Jp_p << Jp_p_aux, MatrixXd::Zero(Jp_p_aux.rows(), n_space-1-joint_counter);

    // std::cout << "AQUI 5.5  " << std::endl;

    // Get the distance Jacobian to the floor
    MatrixXd Jt_pi(1,n_space);
    Jt_pi << franka_.point_to_plane_distance_jacobian(Jt, t, pi_floor_), MatrixXd::Zero(1, n_space-1-joint_counter);

    
    // Get the distance to the floor 
    double d_p_floor = double(dot(t,n_floor_)-d_floor_);
    double d_error_floor = d_p_floor - d_safe_floor_;   

    // Define now the inequalities regarding the VFI from the human
    MatrixXd Ap_p = -Jp_p;
    VectorXd bp_p(d_error.size());
    bp_p << nd_*d_error;

    // Define now the inequalities regarding the VFI from the floor
    MatrixXd Ap_floor = -Jt_pi;
    VectorXd bp_floor(1);
    bp_floor << nd_*d_error_floor;

    //Define the linear inequality matrix and the linear inequality vector
    MatrixXd A_aux(Ap_p.rows() + Ap_floor.rows(), Ap_p.cols());
    A_aux << Ap_p, Ap_floor;

    VectorXd b_aux(bp_p.size() + bp_floor.size());
    b_aux << bp_p, bp_floor;

    A_copy.resize(A.rows(),A.cols());
    A_copy = A;

    b_copy.resize(b.size());
    b_copy = b;

    A.resize(A_copy.rows() + A_aux.rows(), A_aux.cols());
    b.resize(b_copy.size() + b_aux.size());

    if(A_copy.size() == 0){
        A << A_aux;
    }
    else{
        A << A_copy, A_aux;
    }

    if(b_copy.size() == 0){
        b << b_aux;
    }
    else{
        b << b_copy, b_aux;
    }                
  }

  MatrixXd W_q(14,7);
  W_q <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
  VectorXd w_q(14);
  w_q << -1*(q_minus_ - q), 1*(q_plus_ - q);

  // Maybe there is a better way to do this, but I dont know
  // I could also create a function to do that in a simpler way
  A_copy.resize(A.rows(),A.cols());
  A_copy = A;
  b_copy.resize(b.size());
  b_copy = b;

  A.resize(A_copy.rows() + W_q.rows(), A_copy.cols());
  b.resize(b_copy.size() + w_q.size());
  A << A_copy, W_q;
  b << b_copy, w_q;

  // Define the inequalities regarding the max and min velocities
  MatrixXd W_vel(14,7);
  W_vel <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
  VectorXd w_vel(14);
  w_vel << -1*vel_minus_, 1*vel_plus_;

  A_copy.resize(A.rows(),A.cols());
  A_copy = A;
  b_copy.resize(b.size());
  b_copy = b;

  A.resize(A_copy.rows() + W_vel.rows(), A_copy.cols());
  b.resize(b_copy.size() + w_vel.size());
  A << A_copy, W_vel;
  b << b_copy, w_vel;

  // Define the inequalities regarding the max and min acceleration
  ros::Time time_ros;
  double time_now, time_diff;
  time_now = ros::Time::now().toSec();
  time_diff = time_now - time_prev_;
  time_prev_ = time_now;

  // MatrixXd W_accel(14,7);
  // W_accel <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
  // VectorXd w_accel(14);
  // w_accel << -1*(time_diff*accel_minus_ + dq), (time_diff*accel_plus_ + dq);

  // A_copy.resize(A.rows(),A.cols());
  // A_copy = A;
  // b_copy.resize(b.size());
  // b_copy = b;

  // A.resize(A_copy.rows() + W_accel.rows(), A_copy.cols());
  // b.resize(b_copy.size() + w_accel.size());
  // A << A_copy, W_accel;
  // b << b_copy, w_accel;

  VectorXd u(n_space);
  // If there is some error/exception, mainly regarding the solver not finding a solution...
  try{
    if(stop_robot_ == 1){
        if(counter_%4000 == 0){
          ROS_INFO_STREAM(" STOP THE ROBOTTTT " << u);
        }
      // u << VectorXd::Zero(n);
      // Probably it shouldn't be a runtime error, but okay. It is just to merge the stop_robt with the solver error 
      throw std::runtime_error("Something is blocking the camera");
    }
    else{
      // if(counter_%4000 == 0){
      //   ROS_INFO_STREAM(" A AMTRIX A EH " << A << " \n");
      // }
      // Update the linear inequalities in the controller
      translation_controller_.set_inequality_constraint(A, b);
      // Get the next control signal [rad/s]
      u << translation_controller_.compute_setpoint_control_signal(q,vec4(td_));  

      // if(counter_%2000 == 0){
      //   ROS_INFO_STREAM(" I AM HEREEEE   " << u);
      // }
    } 
  }
  catch(std::exception& e){
    // std::cout << e.what() << std::endl;
    // std::cout << "HEREEEE \n\n HEREEEEE '\n\n" << std::endl;

    ROS_INFO_STREAM("             " << e.what() << "\n");

    MatrixXd A_stop(1,1);
    VectorXd b_stop(1);
    
    A_stop.resize(W_q.rows() + W_vel.rows(), W_q.cols());
    b_stop.resize(w_q.size() + w_vel.size());

    A_stop << W_q, W_vel;
    b_stop << w_q, w_vel;

    // Maybe add the inequality regarding the floor....
    // Update the linear inequalities in the controller
    pose_controller_.set_inequality_constraint(A_stop, b_stop);
    // Get the next control signal [rad/s]
    // We put as objective the current position, so the robot try to stop
    u << pose_controller_.compute_setpoint_control_signal(q,vec8(x));  
  }

  // Check the error
  VectorXd e = vec4(t - td_);
  if(e.norm() < 0.05){
    if(decide_td_ == 0){
      // td_ = td2_;
      td_ = td1_;
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
  // if((counter_%2000) -1 == 0){
  //   ROS_INFO_STREAM(" AQUIIII    10    ");
  //   ROS_INFO_STREAM(" Valor de t eh    " << t);
  //   ROS_INFO_STREAM(" Valor de r do robo eh   " << rotation(x));
  //   ROS_INFO_STREAM(" Valor de r da camera eh " << rotation(pose_camera_));
  // }

  // time_now = ros::Time::now().toSec()*1000;
  // time_diff = time_now - time_prev;
  // if(counter_%2000 == 0){
  //   std::cout << "  \n" << time_diff << "      " << time_now << "   \n";
  // }  
                 
}

void JointVelocitySide2SideController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void JointVelocitySide2SideController::cb_update_hmp(const std_msgs::String::ConstPtr& msg){
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    str_poses_human_ = msg->data.c_str();
    refresh_pose_ = 1;
    
    // std::cout << "The size is   " << str_poses_human.length()  << std::endl;
    // std::cout << str_poses_human.substr(0,100) << std::endl;
}

void JointVelocitySide2SideController::cb_stop_robot(const std_msgs::Int32::ConstPtr& msg){
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    stop_robot_ = int(msg->data);
    std::cout << stop_robot_ << std::endl;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocitySide2SideController,
                       controller_interface::ControllerBase)
