
#include <../my_utils/Configuration.h>

#include <my_robot_system/RobotSystem.hpp>

#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>

#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoReachabilityPlanner.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"
#include <my_wbc/WBQPD/WBQPD.hpp>

MagnetoReachabilityNode::MagnetoReachabilityNode(MagnetoReachabilityContact* contact,
                                            const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& dotq) {
  contact_state_ = contact;
  q_ = q;
  dotq_ = dotq;  
}

MagnetoReachabilityNode::~MagnetoReachabilityNode() {}

bool MagnetoReachabilityNode::FindNextNode(const Eigen::VectorXd& ddq_des){
  contact_state_->update(q_, dotq_);

}

MagnetoReachabilityEdge::MagnetoReachabilityEdge(MagnetoReachabilityNode* src_node,
                              MagnetoReachabilityNode* dst_node,
                              const Eigen::VectorXd& trq_atv) {
}

MagnetoReachabilityEdge::~MagnetoReachabilityEdge() {}

MagnetoReachabilityContact::MagnetoReachabilityContact(RobotSystem* robot_planner)  {
  // robot system
  robot_planner_ = robot_planner;
  is_update_centroid_ = false;

  // dimension
  dim_joint_ = Magneto::n_dof;
  dim_contact_ = 0;

  // dyn solver
  wbqpd_param_ = new WbqpdParam();
  wbqpd_result_ = new WbqpdResult();
  wbqpd_ = new WBQPD();
}

MagnetoReachabilityContact::~MagnetoReachabilityContact() {
  _deleteContacts();
}

void MagnetoReachabilityContact::initTorqueLimit(const Eigen::VectorXd& tau_min, 
                                                const Eigen::VectorXd& tau_max) {
  wbqpd_->setTorqueLimit(tau_min, tau_max);
  // wbqpd_->setFrictionCone();
} 


void MagnetoReachabilityContact::_deleteContacts() {
  for(auto &contact : contact_list_)
    delete contact;  
  contact_list_.clear();
}

void MagnetoReachabilityContact::clearContacts() {
  contact_list_.clear();
  dim_contact_=0;
}
void MagnetoReachabilityContact::addContacts(ContactSpec* contact) {
  contact_list_.push_back(contact);
  dim_contact_ += contact->getDim();
}

void MagnetoReachabilityContact::update(const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& dotq) {
  _updateContacts(q, dotq);
  _buildContactJacobian(Jc_);
  _buildContactJcDotQdot(Jcdotqdot_); // Jdotqdot

  // update 
  M_ = robot_planner_->getMassMatrix();
  MInv_ = robot_planner_->getInvMassMatrix();
  cori_grav_ = robot_planner_->getCoriolisGravity();

  AInv_ = Jc_ * MInv_ * Jc_.transpose();
  AMat_ = getPseudoInverse(AInv_);
  Jc_bar_T_ = AMat_ * Jc_* MInv_; // = Jc_bar.transpose() 
  
  Q_ = getNullSpaceMatrix(Jc_bar_T_); 
  Nc_T_ = Eigen::MatrixXd::Identity(dim_joint_,dim_joint_) - Jc_.transpose()*Jc_bar_T_;

}


void MagnetoReachabilityContact::_updateContacts(const Eigen::VectorXd& q,
                                                const Eigen::VectorXd& dotq) {
  robot_planner_->updateSystem(q, dotq, is_update_centroid_);
  for(auto &contact : contact_list_) {
      contact->updateContactSpec();
  }
}

void MagnetoReachabilityContact::_buildContactJacobian(Eigen::MatrixXd& Jc) {
  // initialize Jc
  Jc = Eigen::MatrixXd::Zero(dim_contact_, dim_joint_);

  // vercat Jc
  Eigen::MatrixXd Ji;
  int dim_Ji(0), dim_J(0);
  for(auto &contact : contact_list_) {
    contact->getContactJacobian(Ji);
    dim_Ji = contact->getDim();
    Jc.block(dim_J, 0, dim_Ji, dim_joint_) = Ji;
    dim_J += dim_Ji;
  }
}

void MagnetoReachabilityContact::_buildContactJcDotQdot(
                                  Eigen::VectorXd&  Jcdotqdot) {
  Jcdotqdot = Eigen::VectorXd::Zero(dim_contact_, dim_joint_);
  // vercat Jcdotqdot
  Eigen::VectorXd Ji;
  int dim_Ji(0), dim_J(0);
  for(auto &contact : contact_list_) {
    contact->getJcDotQdot(Ji);
    dim_Ji = contact->getDim();
    Jcdotqdot.segment(dim_J, dim_Ji) = Ji;
    dim_J += dim_Ji;
  }
}


Eigen::MatrixXd MagnetoReachabilityContact::getPseudoInverse(const Eigen::MatrixXd& Mat) {
  Eigen::MatrixXd MatInv;
  my_utils::pseudoInverse(Mat, 0.0001, MatInv);
  return MatInv;
}


Eigen::MatrixXd MagnetoReachabilityContact::getNullSpaceMatrix(const Eigen::MatrixXd& A) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  int nrows(svd.singularValues().rows());
  int Vrows(svd.matrixV().rows());
  int Vcols(svd.matrixV().cols());

  Eigen::MatrixXd NullA = svd.matrixV().block(0, nrows, Vrows, Vcols-nrows);

  // if there are more singular value
  double const tol = 1e-5;
  for(int ii=0; ii<nrows; ++ii) {
    if(svd.singularValues().coeff(ii) < tol) {
      NullA = svd.matrixV().block(0, ii, Vrows, Vcols-ii);
      break;
    }
  }
  return NullA;
}


MagnetoReachabilityPlanner::MagnetoReachabilityPlanner(RobotSystem* robot, 
                                                      double fric_coeff)
    : mu_(fric_coeff) {
    my_utils::pretty_constructor(2, "Magneto Reachablity Planner");
    std::cout<<"fric coeff = " << mu_ << std::endl;
    // robot system
    robot_ = robot;    
    // copy constructor
    robot_planner_ = new RobotSystem(*robot_);

    // Set virtual & actuated selection matrix
    dim_joint_ = Magneto::n_dof;
    Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
    Sv_ = Eigen::MatrixXd::Zero(Magneto::n_vdof, Magneto::n_dof);
    
    for(int i(0); i<Magneto::n_adof; ++i)
      Sa_(i, Magneto::idx_adof[i]) = 1.;
    for(int i(0); i<Magneto::n_vdof; ++i)
      Sv_(i, Magneto::idx_vdof[i]) = 1.;
    
    // etc
    q_zero_dof_ = Eigen::VectorXd::Zero(Magneto::n_dof);
    is_update_centroid_ = false; // used when to update robotsystem

    // set contact
    alfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
                                MagnetoBodyNode::AL_foot_link, mu_);
    blfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
                                MagnetoBodyNode::BL_foot_link, mu_);                          
    arfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
                                MagnetoBodyNode::AR_foot_link, mu_);
    brfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
                                MagnetoBodyNode::BR_foot_link, mu_);

    full_contact_list_.clear();
    full_contact_list_.push_back(alfoot_contact_); 
    full_contact_list_.push_back(blfoot_contact_);
    full_contact_list_.push_back(arfoot_contact_);
    full_contact_list_.push_back(brfoot_contact_);

    swing_contact_state_ = new MagnetoReachabilityContact(robot_planner_);
    full_contact_state_ = new MagnetoReachabilityContact(robot_planner_);
    
    full_contact_state_->clearContacts();
    for(auto &contact : full_contact_list_)
      full_contact_state_->addContacts(contact);
  
}


MagnetoReachabilityPlanner::~MagnetoReachabilityPlanner() {
  full_contact_list_.clear();
  delete alfoot_contact_;
  delete blfoot_contact_;
  delete arfoot_contact_;
  delete brfoot_contact_;

  delete robot_planner_;
  delete full_contact_state_;
  delete swing_contact_state_;
}

void MagnetoReachabilityPlanner::initTorqueLimit(const Eigen::VectorXd& tau_min, 
                                                const Eigen::VectorXd& tau_max) {
  tau_min_ = tau_min; 
  tau_max_ = tau_max;
  full_contact_state_->initTorqueLimit(tau_min_, tau_max_);
  swing_contact_state_->initTorqueLimit(tau_min_, tau_max_);
} 

void MagnetoReachabilityPlanner::setMovingFoot(int moving_foot) {
  moving_foot_idx_ = moving_foot;
  // set contact list
  swing_contact_state_->clearContacts();
  for(auto &contact : full_contact_list_){
    if( ((BodyFramePointContactSpec*)contact)->getLinkIdx() != moving_foot_idx_ )
      swing_contact_state_->addContacts(contact);
  }
}

void MagnetoReachabilityPlanner::compute(const Eigen::VectorXd& q_goal) {
  _setInitGoal(q_goal, q_zero_dof_); // assume zero velocity goal

  q_ = q_init_;
  dotq_ = dotq_init_;

  // full contact node
  MagnetoReachabilityNode node_fc_init(full_contact_state_, 
                                        q_init_, dotq_init_);
  // swing contact node
  MagnetoReachabilityNode node_sc_init(swing_contact_state_, 
                                        q_init_, dotq_init_);

}

void MagnetoReachabilityPlanner::_setInitGoal(const Eigen::VectorXd& q_goal,
                                              const Eigen::VectorXd& qdot_goal) {
  q_goal_ = q_goal;
  dotq_goal_ = qdot_goal;
  q_init_ = robot_->getQ();
  dotq_init_ = robot_->getQdot();
}

