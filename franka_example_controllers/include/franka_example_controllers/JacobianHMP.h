/**
(C) Copyright 2022 DQ Robotics Developers
This file uses the DQ Robotics library.

****************************************************
***** Coded by: Victor Cambraia N. de Oliveira *****
****************************************************

*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

class JacobianHMP{

public:

    double d_safe_arm;
    double d_safe_torso;
    double d_safe_head;

    int counter;
    // Here the common number will be 50 poses predicted in advance
    int num_poses;
    // The n_dim is as expected 3 (x,y,z)
    int num_dim;
    // Probably I should add another variable that would represent the number of joints in each pose
    int num_joints_per_pose;
    // Parameter regarding the importance of the deviation_joints (related with the joint sigma)
    double K_error;

    // COnstructor
    JacobianHMP(const VectorXd &d_safe, double K_error = 0.2);

    // Methods
    void add_counter(int value);
    // This function shouldnt be here in this class
    std::tuple<MatrixXd, VectorXd> transform_camera_points_2matrix(std::string& str_numbers, DQ& pose_camera);
    // This function as well
    VectorXd change_ref_to_lab(VectorXd& point_ref, DQ& pose_ref);
    // MatrixXd transform_points_human2matrix(std::string& str_numbers);
    std::tuple<MatrixXd, VectorXd, VectorXd> get_jacobian_human(const DQ_SerialManipulatorMDH& franka, const MatrixXd &Jt,const DQ &t,const MatrixXd &points_human, const VectorXd &deviation_joints);
    std::tuple<MatrixXd, VectorXd, VectorXd> get_3jacobians_human(const DQ_SerialManipulatorMDH& franka, const MatrixXd &Jt,const DQ &t,const MatrixXd &points_human, const VectorXd &deviation_joints);
    std::tuple<MatrixXd, VectorXd, VectorXd> choose_Jacobian(const DQ_SerialManipulatorMDH& franka, const MatrixXd &Jt,const DQ &t,const MatrixXd &points_human, const VectorXd &deviation_joints, int i_min);

    std::tuple<int, DQ> check_get_plane(MatrixXd &points_plane, const DQ &t);
    std::tuple<int, DQ, double> check_get_line(MatrixXd &points_line, const DQ &t);
};

}