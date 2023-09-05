// Controller developed by Victor Cambraia N. de Oliveira based on the structure presented by the Franka Emika code

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
  translation_controller_.set_gain(5);
  translation_controller_.set_damping(1);
  translation_controller_.set_control_objective(DQ_robotics::Translation);

  translation_controller_comp_.set_gain(5);
  translation_controller_comp_.set_damping(1);
  translation_controller_comp_.set_control_objective(DQ_robotics::Translation);

  pose_controller_.set_gain(1);
  pose_controller_.set_damping(1);
  pose_controller_.set_control_objective(DQ_robotics::Pose);
  
  q_minus_ = VectorXd(7);
  q_plus_ = VectorXd(7);
  q_minus_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_plus_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

  vel_minus_ = VectorXd(7);
  vel_plus_ = VectorXd(7);
  vel_minus_ << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
  vel_plus_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

  accel_minus_ = VectorXd(7);
  accel_plus_ = VectorXd(7);
  accel_minus_ << -15, -7.5, -10, -12.5, -15, -20, -20;
  accel_plus_ << 15, 7.5, 10, 12.5, 15, 20, 20;

  // Define the floor
  n_floor_ = 1*k_;
  d_floor_ = DQ(0);
  pi_floor_ = n_floor_ + E_*d_floor_;
  nd_ = 1.0;
  // nd_ = 1.0;
  d_safe_floor_ = 0.2;

  // Define the pose of the camera
  double ang_degree = -90;
  DQ rotation_camera = cos(ang_degree/2*(pi/180)) + sin(ang_degree/2*(pi/180))*(1*k_);
  DQ translation_camera = -0.03*i_ -0.23*j_ + 0.38*k_;
  pose_camera_ = rotation_camera + 0.5*E_*translation_camera*rotation_camera;

  // Initialize the variables that we be used later
  tau_ = 0.01; //It works as a delta_t
  counter_ = 0; // count the number of cycles
  decide_td_ = 0; //aux variable to choose the td

  // Desired positions of the manipulator
  td1_ = 0.6*i_ + 0.5*j_ + 0.4*k_;
  td2_ = 0.4*i_  -0.5*j_ + 0.4*k_; // the desired position of the robot
  td_ = td1_;

  // TESTS AND RESULTS
  // Define the safety parameters for the human ARM, TORSO, HEAD RESPECTIVELY
  d_safe_hmp_ = VectorXd(3);
  d_safe_hmp_ << 0.04, 0.05, 0.62;
  // d_safe_hmp_ << 0.04, 0.05, 0.65;

  //If you want to change the K_error factor from 0.2 to other value, write it here
  double K_error = 0.2;
  // double K_error = 0.00; //For the No prediction Case

  // Get the object from the class that will return the jacobian for us
  J_hmp_ = JacobianHMP(d_safe_hmp_, K_error);

  //Initialize the variable that will store the human poses
  int n_rows = J_hmp_.num_poses*J_hmp_.num_joints_per_pose;
  int n_cols = J_hmp_.num_dim;
  poses_human_ = 100*MatrixXd::Ones(n_rows, n_cols);
  deviation_joints_ = VectorXd::Zero(n_rows);

  counter_refresh_ = 0;

  // Initialize the variables of the filter
  dq_filtered_ = VectorXd::Zero(7);
  K_filter_ = 50;

  // Initialize the variables to log the data
  log_data_ = 0;

  // HERE Initialize the subscribers and publishers(if there is one)
  sub_prediction_ = n_pred_.subscribe("prediction_human", 1000, &JointVelocitySide2SideController::cb_update_hmp, this);
  sub_stop_robot_ = n_stop_.subscribe("stop_robot", 1000, &JointVelocitySide2SideController::cb_stop_robot, this);

  return true;
}

void JointVelocitySide2SideController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  time_prev_ = ros::Time::now().toSec();
}

void JointVelocitySide2SideController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // ------------------ HERE I WILL ADD MY CODE OF UPDATE -----------------------

  counter_++;

  if(refresh_pose_ == 1){
    counter_refresh_++;
    if(counter_refresh_ % 1 == 0){
      std::tie(poses_human_, deviation_joints_) = J_hmp_.transform_camera_points_2matrix(str_poses_human_, pose_camera_);
    }
    refresh_pose_ = 0;
  }

  // THIS IS ACTUALLY USEFUL FOR GETTING RESULTS
  MatrixXd pose_human_single(9,3);
  VectorXd deviation_joints_single(9);
  int j;
  for(j = 0; j < 9; j++){
    pose_human_single.row(j) << poses_human_.row(j);
    deviation_joints_single[j] = deviation_joints_[j];
  }

  int i;
  int n_space = franka_.get_dim_configuration_space();
  VectorXd q(n_space); //joint values
  VectorXd dq(n_space); //joint velocity values
  franka::RobotState robot_state = state_handle_->getRobotState();

  for(i=0;i<n_space;i++){
    q[i] = robot_state.q_d[i];
    dq[i] = robot_state.dq_d[i];
  }
  
  MatrixXd A(0,0);
  VectorXd b(0);
  MatrixXd A_copy(0,0);
  VectorXd b_copy(0);

  MatrixXd Ap_floor(0,0);
  VectorXd bp_floor(1);

  DQ t, x;

  MatrixXd Jp_p_aux;
  VectorXd d_error;
  VectorXd d_person;

  MatrixXd Jp_p_aux2;
  VectorXd d_error2;
  VectorXd d_person2;

  MatrixXd Jp_p_aux_single;
  VectorXd d_error_single;
  VectorXd d_person_single;

  int joint_counter; //aux variable for the for
  // Iterate it for every joint of the robot
  for(joint_counter = n_space-1; joint_counter<n_space; joint_counter++){

    MatrixXd Jx = franka_.pose_jacobian(q,joint_counter);

    x = franka_.fkm(q, joint_counter);
    t = translation(x);
    // Get the robot's translation Jacobian
    MatrixXd Jt = franka_.translation_jacobian(Jx, x);

    // The Jacobian for one or more poses

    std::tie(Jp_p_aux, d_error, d_person) = J_hmp_.get_3jacobians_human(franka_, Jt,t, poses_human_, deviation_joints_);
    // FOR DEBUGGING AND LOGGING PURPOSES
    std::tie(Jp_p_aux_single, d_error_single, d_person_single) = J_hmp_.get_3jacobians_human(franka_, Jt,t, pose_human_single, deviation_joints_single);

    MatrixXd Jp_p(Jp_p_aux.rows(),n_space);
    Jp_p << Jp_p_aux, MatrixXd::Zero(Jp_p_aux.rows(), n_space-1-joint_counter);
    
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
    Ap_floor.resize(Jt_pi.rows(), Jt_pi.cols());
    Ap_floor = -Jt_pi;
    bp_floor.resize(1);
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

  // IN THE END WE DID NOT USE THE ACCELERATION CONSTRAINTS, BUT HERE IT IS
  // Define the inequalities regarding the max and min acceleration
  ros::Time time_ros;
  double time_now, time_diff;
  time_now = ros::Time::now().toSec();
  time_diff = time_now - time_prev_;
  time_prev_ = time_now;

  // Define variable to help in the log process
  int  exception_happended = 0;

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
      // Probably it shouldn't be a runtime error, but okay. It is just to merge the stop_robt with the solver error 
      throw std::runtime_error("Something is blocking the camera");
    }
    else{
      // Update the linear inequalities in the controller
      translation_controller_.set_inequality_constraint(A, b);
      // Get the next control signal [rad/s]
      u << translation_controller_.compute_setpoint_control_signal(q,vec4(td_));  
      
      // Pass through the filter
      dq_filtered_ = u/K_filter_ + (K_filter_ - 1)*dq_filtered_/K_filter_;
    } 
  }
  catch(std::exception& e){
    exception_happended = 1;
    if(counter_ % 200 == 0){
      ROS_INFO_STREAM("             " << e.what() << "\n");
    }
    
    MatrixXd A_stop(1,1);
    VectorXd b_stop(1);
    
    A_stop.resize(W_q.rows() + W_vel.rows() + Ap_floor.rows(), W_q.cols());
    b_stop.resize(w_q.size() + w_vel.size() + bp_floor.size());

    A_stop << W_q, W_vel, Ap_floor;
    b_stop << w_q, w_vel, bp_floor;

    pose_controller_.set_inequality_constraint(A_stop, b_stop);
    // Get the next control signal [rad/s]
    // We put as objective the current position, so the robot try to stop
    u << pose_controller_.compute_setpoint_control_signal(q,vec8(x));  

    dq_filtered_ = u/K_filter_ + (K_filter_ - 1)*dq_filtered_/K_filter_;
  }

  // THIS PART OF THE CODE RESPONSIBLE FOR LOGGING DATA WILL BE COMMENTED
  // FOR LOGGING PURPOSES
  // if((log_data_ == 1) && (counter_% 100 == 0)){
  //     vec_d_person_.push_back(d_person_single[2]);
  //     vec_d_x_.push_back(vec4(t)[1]);
  //     vec_d_y_.push_back(vec4(t)[2]);
  //     vec_d_z_.push_back(vec4(t)[3]);

  //     vec_head_x_.push_back(pose_human_single(2,0));
  //     vec_head_y_.push_back(pose_human_single(2,1));
  //     vec_head_z_.push_back(pose_human_single(2,2));
  //     // vec_vel_joint1_.push_back(dq_filtered_[1]);
  //     vec_timestamps_.push_back(time_now-time_log_);
  // }

  // Check the error
  VectorXd e = vec4(t - td_);
  if(e.norm() < 0.03){
    
    if(log_data_ == 0 && decide_td_ == 0){
      time_log_ = time_now;
      log_data_ = 1;
    }
    else if(log_data_ == 1 && decide_td_ == 1){
      log_data_ = 0;
    }
    // Changing the desired position of the manipulator
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
    velocity_joint_handles_[i].setCommand(dq_filtered_[i]);
  }              
}

void JointVelocitySide2SideController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.

    // ----- THIS PART OF THE CODE RESPONSIBLE FOR LOGGING WILL BE COMMENTED TO AVOID MISTAKES
    // ----- BUT IF NEEDED, YOU CAN LOG DATA USING THE PIECE OF CODE BELOW
  
    // // PASSING THE LOGGED INFORMATION TO A TXT FILE  
    // // Create and open a text file
    // std::ofstream myfile;
    // myfile.open("/home/geriatronics/victor/Log_Data/example.txt");

    // // Write to the file
    // int i;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_timestamps_[i] << "," ;
    // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_errors_[i] << ",";
    // // }

    // myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_d_person_[i] << ",";
    // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_person2_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_person3_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_person4_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_vel_joint1_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_i_zone_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_arms_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_torso_[i] << ",";
    // // }

    // // myfile << std::endl;

    // // for(i=0; i<vec_timestamps_.size();i++){
    // //   myfile << vec_d_head_[i] << ",";
    // // }

    // myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_d_x_[i] << ",";
    // }
    // myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_d_y_[i] << ",";
    // }

    // myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_d_z_[i] << ",";
    // }

    //     myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_head_x_[i] << ",";
    // }

    //     myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_head_y_[i] << ",";
    // }

    // myfile << std::endl;

    // for(i=0; i<vec_timestamps_.size();i++){
    //   myfile << vec_head_z_[i] << ",";
    // }

    // // Close the file
    // myfile.close();
}

void JointVelocitySide2SideController::cb_update_hmp(const std_msgs::String::ConstPtr& msg){
    str_poses_human_ = msg->data.c_str();
    refresh_pose_ = 1;
}

void JointVelocitySide2SideController::cb_stop_robot(const std_msgs::Int32::ConstPtr& msg){
    stop_robot_ = int(msg->data);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocitySide2SideController,
                       controller_interface::ControllerBase)
