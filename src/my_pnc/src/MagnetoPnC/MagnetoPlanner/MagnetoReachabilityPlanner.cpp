
#include <../my_utils/Configuration.h>

#include <my_robot_system/RobotSystem.hpp>

#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>

#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoReachabilityPlanner.hpp>

#include <my_utils/IO/IOUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"



MagnetoReachabilityNode::MagnetoReachabilityNode(MagnetoReachabilityContact* contact,
                                            const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& dotq) {
  contact_state_ = contact;
  q_ = q;
  dotq_ = dotq;  
}

MagnetoReachabilityNode::~MagnetoReachabilityNode() {}

bool MagnetoReachabilityNode::FindNextNode(const Eigen::VectorXd& ddq_des,
                                          Eigen::VectorXd& tau_a){                                             
  std::cout<< "FindNextNode - 1 "<< std::endl;
  contact_state_->update(q_, dotq_, ddq_des);
  std::cout<< "FindNextNode - 2 "<< std::endl;
  contact_state_->solveContactDyn(tau_a);
  std::cout<< "FindNextNode - 3 "<< std::endl;
  // contact_state_->computeNextState(tau_a, q_next, dotq_next); 
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

  Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
  Sv_ = Eigen::MatrixXd::Zero(Magneto::n_vdof, Magneto::n_dof);
  
  for(int i(0); i<Magneto::n_adof; ++i)
    Sa_(i, Magneto::idx_adof[i]) = 1.;
  for(int i(0); i<Magneto::n_vdof; ++i)
    Sv_(i, Magneto::idx_vdof[i]) = 1.;
}

MagnetoReachabilityContact::~MagnetoReachabilityContact() {
  _deleteContacts();
}

void MagnetoReachabilityContact::initialization(const YAML::Node& node) {
  // torque limit, magnetic force
  double torque_limit(0.);
  double magnetic_force(0.), residual_ratio(0.);
  try {
    my_utils::readParameter(node["controller_params"], 
                          "torque_limit", torque_limit);

    my_utils::readParameter(node["magnetism_params"], 
                          "magnetic_force", magnetic_force);  
    my_utils::readParameter(node["magnetism_params"], 
                          "residual_ratio", residual_ratio);
   
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  Eigen::VectorXd tau_min =
      // sp_->getActiveJointValue(robot_->GetTorqueLowerLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, -torque_limit); // -5.
  Eigen::VectorXd tau_max =
      // sp_->getActiveJointValue(robot_->GetTorqueUpperLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, torque_limit); // 5.

  wbqpd_->setTorqueLimit(tau_min, tau_max);
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

void MagnetoReachabilityContact::FinishContactSet() {
  // set parameters dependent on contact list
  Eigen::VectorXd F_magnetic;
  Eigen::MatrixXd Uf_i, Uf;
  Eigen::VectorXd Fr_ieq_i, Fr_ieq, u0;

  // 1. magnetic force vector && dimension check
  F_magnetic = Eigen::VectorXd::Zero(dim_contact_);
  int contact_idx(0), fz_idx;  
  int Uf_row_dim(0), Uf_col_dim(0);
  for(auto &it : contact_list_) {
    // magnetic force
    fz_idx = contact_idx + ((BodyFramePointContactSpec*)(it))->getFzIndex();    
    F_magnetic[fz_idx] = magnetic_force_;
    contact_idx += ((BodyFramePointContactSpec*)(it))->getDim();
    // contact friction cone dimension check
    it->getRFConstraintMtx(Uf_i);
    Uf_row_dim+= Uf_i.rows();
    Uf_col_dim+= Uf_i.cols();
  }

  // 2. contact friction cone
  Uf = Eigen::MatrixXd::Zero(Uf_row_dim, Uf_col_dim);
  Fr_ieq = Eigen::VectorXd::Zero(Uf_row_dim);
  Uf_row_dim = 0; Uf_col_dim = 0;
  for(auto &contact : contact_list_) {
    contact->getRFConstraintMtx(Uf_i);
    contact->getRFConstraintVec(Fr_ieq_i);
    Uf.block(Uf_row_dim, Uf_col_dim, Uf_i.rows(), Uf_i.cols()) = Uf_i;
    Fr_ieq.segment(Uf_row_dim, Uf_i.rows()) = Fr_ieq_i;
    Uf_row_dim+= Uf_i.rows();
    Uf_col_dim+= Uf_i.cols();
  }
  Fr_ieq -= Uf*F_magnetic; // todo : sign check
  wbqpd_->setFrictionCone(Uf, Fr_ieq);
}

/*
ddq = MInv_*(Nc_T_*Sa_T*tau - NC_T(b+g) -Jc_T_*AMat_*Jcdotqdot_)
    = A*tau + a0
Fc = -AMat_*Jcdotqdot_+Jc_bar_T_(b+g) - Jc_bar_T_*Sa_T*tau
    = B*tau + b0

A = MInv_*Nc_T_*Sa_T
a0 = MInv_*(- NC_T(b+g) -Jc_T_*AMat_*Jcdotqdot_)
B = -Jc_bar_T_*Sa_T
b0 = -AMat_*Jcdotqdot_+Jc_bar_T_(b+g)
*/

void MagnetoReachabilityContact::update(const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& dotq,
                                        const Eigen::VectorXd& ddotq) {
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

  wbqpd_param_->A = MInv_*Nc_T_*Sa_.transpose();
  wbqpd_param_->a0 = MInv_*(- Nc_T_*cori_grav_ -Jc_.transpose()*AMat_*Jcdotqdot_);
  wbqpd_param_->B = -Jc_bar_T_*Sa_.transpose();
  wbqpd_param_->b0 = -AMat_*Jcdotqdot_+Jc_bar_T_*cori_grav_;
  
  wbqpd_param_->ddq_des = ddotq;
  wbqpd_param_->Wq = Eigen::VectorXd::Constant(dim_joint_, 1000.);
  wbqpd_param_->Wf = Eigen::VectorXd::Constant(dim_contact_, 1.0);
  int fz_idx(0), contact_idx(0);
  for(auto &it : contact_list_) {
    fz_idx = contact_idx + ((BodyFramePointContactSpec*)(it))->getFzIndex();  
    wbqpd_param_->Wf[fz_idx] = 0.0001; // Fz no cost
    contact_idx += it->getDim();
  }

  wbqpd_->updateSetting(wbqpd_param_);
}

void MagnetoReachabilityContact::solveContactDyn(Eigen::VectorXd& tau){
  wbqpd_->computeTorque(wbqpd_result_);
  bool b_reachable = wbqpd_result_->b_reachable;
  tau = wbqpd_result_->tau;  
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

MagnetoReachabilityPlanner::MagnetoReachabilityPlanner(RobotSystem* robot) {
  my_utils::pretty_constructor(2, "Magneto Reachablity Planner");
  // robot system
  robot_ = robot;    
  // copy constructor
  robot_planner_ = new RobotSystem(*robot_);

  // Set virtual & actuated selection matrix
  Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
  Sv_ = Eigen::MatrixXd::Zero(Magneto::n_vdof, Magneto::n_dof);
  
  for(int i(0); i<Magneto::n_adof; ++i)
    Sa_(i, Magneto::idx_adof[i]) = 1.;
  for(int i(0); i<Magneto::n_vdof; ++i)
    Sv_(i, Magneto::idx_vdof[i]) = 1.;
  
  // etc
  q_zero_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  is_update_centroid_ = false; // used when to update robotsystem

  // set contact
  mu_ = 0.7; // will be updated later
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
  full_contact_state_->FinishContactSet();

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

void MagnetoReachabilityPlanner::initialization(const YAML::Node& node){

  try {
    my_utils::readParameter(node["contact_params"], "friction", mu_);   
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  for(auto &contact : full_contact_list_)
    ((BodyFramePointContactSpec*)contact)->setFrictionCoeff(mu_);


  full_contact_state_->initialization(node);
  swing_contact_state_->initialization(node);
} 

void MagnetoReachabilityPlanner::setMovingFoot(int moving_foot) {
  moving_foot_idx_ = moving_foot;
  // set contact list
  swing_contact_state_->clearContacts();
  for(auto &contact : full_contact_list_){
    if( ((BodyFramePointContactSpec*)contact)->getLinkIdx() != moving_foot_idx_ )
      swing_contact_state_->addContacts(contact);
  }
  swing_contact_state_->FinishContactSet();
}

void MagnetoReachabilityPlanner::compute(const Eigen::VectorXd& q_goal) {
  _setInitGoal(q_goal, q_zero_); // assume zero velocity goal

  q_ = q_init_;
  dotq_ = dotq_init_;
  std::cout<< "compute - 1 "<< std::endl;
  // full contact node
  MagnetoReachabilityNode* node_fc_init =
                new MagnetoReachabilityNode(full_contact_state_, 
                                        q_init_, dotq_init_);
  //  
  Eigen::VectorXd tau_a;
  std::cout<< "compute - 2 "<< std::endl;
  node_fc_init->FindNextNode(q_zero_, tau_a);
  // 
  std::cout<< "compute - 3 "<< std::endl;
  // swing contact node
  MagnetoReachabilityNode* node_sc_init = 
              new MagnetoReachabilityNode(swing_contact_state_, 
                                        q_init_, dotq_init_);

}

void MagnetoReachabilityPlanner::_setInitGoal(const Eigen::VectorXd& q_goal,
                                              const Eigen::VectorXd& qdot_goal) {
  q_goal_ = q_goal;
  dotq_goal_ = qdot_goal;
  q_init_ = robot_->getQ();
  dotq_init_ = robot_->getQdot();
}

