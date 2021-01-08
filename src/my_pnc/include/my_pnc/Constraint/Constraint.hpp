#pragma once

#include <stdio.h>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>

class Constraint {
   public:
    Constraint(RobotSystem* _robot, int _link_idx);
    ~Constraint() {}


    bool isUpdated() {return b_updated_; }

    int getLinkIdx() { return link_idx_; }
    int getDim() { return dim_constraint_; }
    int getPositionDim() { return dim_position_constraint_; }
    int getJointDim() { return dim_joint_constraint_; }

    void getJacobian(Eigen::MatrixXd& Jcs) { Jcs = Jcs_; }
    void getPositionJacobian(Eigen::MatrixXd& Jcs) { 
        Jcs = Jcs_.block(dim_constraint_-dim_position_constraint_, 0,
                        dim_position_constraint_, dim_joint_constraint_); }
    void getPosition(Eigen::VectorXd& Pcs) { Pcs = Pcs_; }
    void getPositionError(Eigen::VectorXd& Pcs_err) { Pcs_err = Pcs_err_; }
    
    void update();
    void setDesired(const POSE_DATA& pos_del);
      
    void printInfos() {
        my_utils::pretty_print(pos_des_, std::cout, "pos err");
        my_utils::pretty_print(Jcs_, std::cout, "task jacobian");
    }

   protected:
    void _updatePositionError();
    void _updatePosition();
    void _updateJacobian();

   protected:
    RobotSystem* robot_;

    bool b_updated_;
    int link_idx_;  

    int dim_constraint_;
    int dim_position_constraint_;
    int dim_joint_constraint_;


    Eigen::MatrixXd Jcs_;
    Eigen::VectorXd Pcs_; // position
    Eigen::VectorXd Pcs_err_; // des - act
    Eigen::VectorXd Jdotqdot_;

    Eigen::VectorXd pos_ini_;
    Eigen::Quaternion<double> ori_ini_; // R_wb

    Eigen::VectorXd pos_des_;
    Eigen::Quaternion<double> ori_des_;

};
