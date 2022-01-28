
#include <../my_utils/Configuration.h>

#include <my_robot_system/RobotSystem.hpp>

#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_interface.hpp>
#include <my_robot_core/magneto_core/magneto_command_api.hpp>

#include <my_robot_core/magneto_core/magneto_planner/magneto_goal_planner.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"


MagnetoGoalPlanner::MagnetoGoalPlanner(RobotSystem* robot) {
    my_utils::pretty_constructor(2, "Magneto CoM Goal Planner");
    // robot system
    robot_ = robot;    
    // copy constructor
    robot_planner_ = new RobotSystem(*robot_);

    // set cost function Matrices
    _InitCostFunction();

    // set Constraint
    std::vector<int> foot_idx_list;
    foot_idx_list.push_back(MagnetoBodyNode::AL_foot_link);
    foot_idx_list.push_back(MagnetoBodyNode::BL_foot_link);
    foot_idx_list.push_back(MagnetoBodyNode::AR_foot_link);
    foot_idx_list.push_back(MagnetoBodyNode::BR_foot_link);
    _InitConstraints(foot_idx_list);
}

MagnetoGoalPlanner::~MagnetoGoalPlanner() {
  _DeleteConstraints();
}

void MagnetoGoalPlanner::_InitConstraints(
                          const std::vector<int> _link_idx_list) {
  _DeleteConstraints();
  for(auto &link_idx : _link_idx_list)
    constraint_list.push_back(
        new Constraint(robot_planner_, link_idx));
}

void MagnetoGoalPlanner::_DeleteConstraints() {
  for(auto &constraint : constraint_list)
    delete constraint;  
  constraint_list.clear();
}

void MagnetoGoalPlanner::_setDesiredFootPosition(MotionCommand _motion_command) {
  // contact class / Task class >> Foot Class ?
  
  // initialize joint values
  q_ = robot_->getQ();
  dotq_ = robot_->getQdot(); 
  delq_ = Eigen::VectorXd::Zero(q_.size());
  _UpdateConfiguration(q_);
  _InitCostFunction();

  // assume one foot is moving
  int moving_foot_idx;
  MOTION_DATA motion_data;
  _motion_command.get_foot_motion_command(motion_data, moving_foot_idx);

  POSE_DATA zero_pose = POSE_DATA();
  for(auto &constraint : constraint_list){
    if(constraint->getLinkIdx() == moving_foot_idx) 
      constraint->setDesired(motion_data.pose);
    else
      constraint->setDesired(zero_pose);    
  }
}

void MagnetoGoalPlanner::getGoalConfiguration(Eigen::VectorXd& _q_goal){
  _q_goal = q_goal_;
}

void MagnetoGoalPlanner::computeGoal(MotionCommand &_motion_command) {

  _setDesiredFootPosition(_motion_command);

  double tol = 1e-5;
  double err = 1e5;
  int iter(0), max_iter(10000);
  while(err > tol && iter++ < max_iter) {
    // synchronize robot_planner
    _UpdateConfiguration(q_);
    // update constraint with robot_planner
    _UpdateConstraints();
    // compute c, Jacob, JacobNS
    _BuildConstraints();
    _BuildNSConstraints();
    // compute optimal delq
    _UpdateDelQ();
    // update q
    // if(iter%20 == 1)
    // {
    //   std::cout<<"iter(" << iter << "), err=" << err << std::endl;
    //   my_utils::pretty_print(q_, std::cout, "q");
    //   my_utils::pretty_print(delq_, std::cout, "delq");
    //   my_utils::pretty_print(ceq_, std::cout, "ceq_");
    // }

    q_ += delq_;
    err = delq_.norm();
  }

  q_goal_ = q_;

  // std::cout<<"iter(" << iter << "), err=" << err << std::endl;
  // my_utils::pretty_print(q_, std::cout, "q");
  // my_utils::pretty_print(ceq_, std::cout, "ceq_");

  // add com goal
  MOTION_DATA motion_data;
  _motion_command.get_foot_motion_command(motion_data);
  motion_data.pose.pos = robot_planner_->getCoMPosition()
                          - robot_->getCoMPosition();
  motion_data.pose.is_baseframe = false;
  motion_data.swing_height = 0.0;
  _motion_command.add_motion(-1, motion_data);
}

void MagnetoGoalPlanner::_UpdateDelQ() {
  // change the problem:
  // q'A q + b'q -> z'Aprime z + bprime'z
  Eigen::MatrixXd Aprime, AprimeInv;
  Eigen::VectorXd bprime;
  _computeCostWeight(Aprime,bprime);
  my_utils::pseudoInverse(Aprime, 0.0001, AprimeInv);

  // get optimal z
  Eigen::VectorXd z = - 0.5 * AprimeInv * bprime;
  // get optimal delq
  delq_ = 0.1 * (- cJacobInv_ * ceq_ + cJacobNull_ * z);
}

void MagnetoGoalPlanner::_computeCostWeight(
                          Eigen::MatrixXd& _Aprime,
                          Eigen::VectorXd& _bprime) {

  _Aprime = cJacobNull_.transpose() * A_ * cJacobNull_;
  _bprime = 2.0*cJacobNull_.transpose() * A_ * (q_ - cJacobInv_ *ceq_)
            + cJacobNull_.transpose() * b_;
}

void MagnetoGoalPlanner::_UpdateConstraints() {
  for(auto &constraint : constraint_list) {
      constraint->update();
  }
}

void MagnetoGoalPlanner::_BuildConstraints() {
  // only consider position _BuildConstraints

  // check Size
  int dim_J(0), dim_Jc(0), dim_c(0);
  for(auto &constraint : constraint_list) {
    dim_Jc = constraint->getJointDim();
    dim_J += constraint->getPositionDim();
    dim_c += constraint->getPositionDim();
  }

  // vercat ceq_, cJacobian_
  cJacobian_ = Eigen::MatrixXd::Zero(dim_J, dim_Jc);
  ceq_ = Eigen::VectorXd::Zero(dim_c);

  int dim_Ji(0), dim_ci(0);
  Eigen::MatrixXd Ji;
  Eigen::VectorXd ci;
  dim_J = 0;   dim_c = 0;
  for(auto &constraint : constraint_list) {
    constraint->getPositionJacobian(Ji);
    constraint->getPositionError(ci);

    dim_Ji = constraint->getPositionDim();
    dim_ci = constraint->getPositionDim();

    cJacobian_.block(dim_J,0, dim_Ji,dim_Jc) = Ji;
    ceq_.segment(dim_c, dim_ci) = ci;

    dim_J += dim_Ji;
    dim_c += dim_ci;
  }

  my_utils::pseudoInverse(cJacobian_, 0.0001, cJacobInv_); 
}


void MagnetoGoalPlanner::_BuildNSConstraints() {
  // build null space of constraint jacobian
  // cJacobian_ * cJacobNull_ = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    cJacobian_, 
    Eigen::ComputeThinU | Eigen::ComputeFullV);
  // matrixV(), matrixU(), singularValues()

  int const nrows(svd.singularValues().rows());
  int const Vrows(svd.matrixV().rows());
  int const Vcols(svd.matrixV().cols());

  cJacobNull_ = svd.matrixV().
                  block(0, nrows, Vrows, Vcols-nrows);

  // if there are more singular value
  double const tol = 1e-5;
  for(int ii=0; ii<nrows; ++ii) {
    if(svd.singularValues().coeff(ii) < tol) {
      cJacobNull_ = svd.matrixV().
                  block(0, ii, Vrows, Vcols-ii);
      break;
    }
  }
} 

void MagnetoGoalPlanner::_UpdateConfiguration(
                        const Eigen::VectorXd& q) {
    bool b_centroidal = false;
    robot_planner_->updateSystem(q, 
                              dotq_, 
                              b_centroidal);   
}

void MagnetoGoalPlanner::_InitCostFunction() {
  // J(q) = (q2+q3+pi/2)^2 + a*(q3+pi/2)^2
  double alpha = 1.;
  // FEMUR:2, TIBIA:3
  num_joint_dof_ = robot_->getNumDofs();
  A_ = Eigen::MatrixXd::Zero(num_joint_dof_,num_joint_dof_);
  b_ = Eigen::VectorXd::Zero(num_joint_dof_);

  // base ori
  double beta = 0.1;
  q_ = robot_->getQ();
  A_(MagnetoDoF::baseRotZ, MagnetoDoF::baseRotZ) = beta;
  A_(MagnetoDoF::baseRotY, MagnetoDoF::baseRotY) = beta;
  A_(MagnetoDoF::_base_joint, MagnetoDoF::_base_joint) = beta;
  b_(MagnetoDoF::baseRotZ) = -2.0*beta*q_(MagnetoDoF::baseRotZ);
  b_(MagnetoDoF::baseRotY) = -2.0*beta*q_(MagnetoDoF::baseRotY);
  b_(MagnetoDoF::_base_joint) = -2.0*beta*q_(MagnetoDoF::_base_joint);

  // leg 
  for(int ii(0); ii<num_joint_dof_; ++ii) {
    // active
    if(_checkJoint(ii,MagnetoJointType::COXA)) {
      A_(ii,ii) = beta;
      b_(ii) =  -2.0*beta*q_(ii);
    }
    else if(_checkJoint(ii, MagnetoJointType::FEMUR)) {
      b_(ii) = M_PI;
      A_(ii,ii) = 1.;
      A_(ii,ii+1) = 1.;
      // for(int jj(0); jj<num_joint_dof_; ++jj) {
      //   if(_checkJoint(jj, MagnetoJointType::FEMUR))
      //     A_(ii,jj) = 1.;
      //   if(_checkJoint(jj, MagnetoJointType::TIBIA))
      //     A_(ii,jj) = 1.;
      // }
    } else if(_checkJoint(ii, MagnetoJointType::TIBIA)) {
      b_(ii) = (1.+alpha)*M_PI;
      A_(ii,ii) = 1.+ alpha;
      A_(ii,ii-1) = 1.;
      // for(int jj(0); jj<num_joint_dof_; ++jj) {
      //   if(_checkJoint(jj, MagnetoJointType::TIBIA))
      //     A_(ii,jj) = 1. + alpha;
      //   if(_checkJoint(jj, MagnetoJointType::FEMUR))
      //     A_(ii,jj) = 1.;
      // }
    }

    // passive
    double gamma = 0.5;
    if(_checkJoint(ii, MagnetoJointType::FOOT1) ||
        _checkJoint(ii, MagnetoJointType::FOOT2) ||
        _checkJoint(ii, MagnetoJointType::FOOT3) ) {
          A_(ii,ii) = gamma;
    }
  }
}



bool MagnetoGoalPlanner::_checkJoint(int joint_idx, MagnetoJointType joint_type)
{
  switch(joint_type){
    case MagnetoJointType::COXA:
    if( joint_idx==MagnetoDoF::AL_coxa_joint ||
        joint_idx==MagnetoDoF::AR_coxa_joint ||
        joint_idx==MagnetoDoF::BL_coxa_joint ||
        joint_idx==MagnetoDoF::BR_coxa_joint)
      return true;
    break;
    case MagnetoJointType::FEMUR:
    if( joint_idx==MagnetoDoF::AL_femur_joint ||
        joint_idx==MagnetoDoF::AR_femur_joint ||
        joint_idx==MagnetoDoF::BL_femur_joint ||
        joint_idx==MagnetoDoF::BR_femur_joint)
      return true;
    break;
    case MagnetoJointType::TIBIA:
    if( joint_idx==MagnetoDoF::AL_tibia_joint ||
        joint_idx==MagnetoDoF::AR_tibia_joint ||
        joint_idx==MagnetoDoF::BL_tibia_joint ||
        joint_idx==MagnetoDoF::BR_tibia_joint)
      return true;
    break;
    case MagnetoJointType::FOOT1:
    if( joint_idx==MagnetoDoF::AL_foot_joint_1 ||
        joint_idx==MagnetoDoF::AR_foot_joint_1 ||
        joint_idx==MagnetoDoF::BL_foot_joint_1 ||
        joint_idx==MagnetoDoF::BR_foot_joint_1)
      return true;
    break;
    case MagnetoJointType::FOOT2:
    if( joint_idx==MagnetoDoF::AL_foot_joint_2 ||
        joint_idx==MagnetoDoF::AR_foot_joint_2 ||
        joint_idx==MagnetoDoF::BL_foot_joint_2 ||
        joint_idx==MagnetoDoF::BR_foot_joint_2)
      return true;
    break;
    case MagnetoJointType::FOOT3:
    if( joint_idx==MagnetoDoF::AL_foot_joint_3 ||
        joint_idx==MagnetoDoF::AR_foot_joint_3 ||
        joint_idx==MagnetoDoF::BL_foot_joint_3 ||
        joint_idx==MagnetoDoF::BR_foot_joint_3)
      return true;
    break;
  }
  return false;
}