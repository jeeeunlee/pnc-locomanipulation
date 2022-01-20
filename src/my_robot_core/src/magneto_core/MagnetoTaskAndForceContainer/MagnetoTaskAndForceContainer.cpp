#include <my_robot_core/magneto_core/MagnetoTaskAndForceContainer/MagnetoTaskAndForceContainer.hpp>

MagnetoTaskAndForceContainer::MagnetoTaskAndForceContainer(RobotSystem* _robot)
    : TaskAndForceContainer(_robot) {
  _InitializeTasks();
  _InitializeContacts();
  _InitializeMagnetisms();
}

MagnetoTaskAndForceContainer::~MagnetoTaskAndForceContainer() {
  _DeleteTasks();
  _DeleteContacts();
}

void MagnetoTaskAndForceContainer::_InitializeTasks() {
  my_utils::pretty_constructor(2, "Magneto Task And Force Container");

  // CoM and Pelvis Tasks
  com_task_ = 
      new CoMTask(robot_);
  base_ori_task_ = 
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::base_link); 
  joint_task_ = 
      new BasicTask(robot_, BasicTaskType::JOINT, Magneto::n_adof);

  // Set Foot Motion Tasks
  alfoot_pos_task_ =
      new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::AL_foot_link);
  arfoot_pos_task_ =
      new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::AR_foot_link);
  blfoot_pos_task_ =
      new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::BL_foot_link);
  brfoot_pos_task_ =
      new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::BR_foot_link);
  alfoot_ori_task_ =
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::AL_foot_link);
  arfoot_ori_task_ =
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::AR_foot_link);
  blfoot_ori_task_ =
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::BL_foot_link);
  brfoot_ori_task_ =
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::BR_foot_link);

  // Add all tasks initially. Remove later as needed.
  task_list_.clear();
  task_list_.push_back(com_task_);
  task_list_.push_back(base_ori_task_);
  task_list_.push_back(joint_task_);  

  task_list_.push_back(alfoot_pos_task_);
  task_list_.push_back(arfoot_pos_task_);
  task_list_.push_back(blfoot_pos_task_);
  task_list_.push_back(brfoot_pos_task_);

  task_list_.push_back(alfoot_ori_task_);
  task_list_.push_back(arfoot_ori_task_);
  task_list_.push_back(blfoot_ori_task_);
  task_list_.push_back(brfoot_ori_task_);
}

void MagnetoTaskAndForceContainer::_InitializeContacts() {

  friction_coeff_ = Eigen::VectorXd::Constant(Magneto::n_leg, 0.7); // updated later in setContactFriction 
  
  double foot_x = 0.02; // 0.05; 
  double foot_y = 0.02; 

  alfoot_contact_ = new BodyFrameSurfaceContactSpec(
      robot_, MagnetoBodyNode::AL_foot_link, foot_x, foot_y, friction_coeff_[MagnetoFoot::AL]); 
  arfoot_contact_ = new BodyFrameSurfaceContactSpec(
      robot_, MagnetoBodyNode::AR_foot_link, foot_x, foot_y, friction_coeff_[MagnetoFoot::AR]); 
  blfoot_contact_ = new BodyFrameSurfaceContactSpec(
      robot_, MagnetoBodyNode::BL_foot_link, foot_x, foot_y, friction_coeff_[MagnetoFoot::BL]);
  brfoot_contact_ = new BodyFrameSurfaceContactSpec(
      robot_, MagnetoBodyNode::BR_foot_link, foot_x, foot_y, friction_coeff_[MagnetoFoot::BR]);
      
  dim_contact_ = alfoot_contact_->getDim() + arfoot_contact_->getDim() +
                 blfoot_contact_->getDim() + brfoot_contact_->getDim();

  // max_z_ = 500.;

  // // Set desired reaction forces
  // Fd_des_ = Eigen::VectorXd::Zero(dim_contact_);

  // Add all contacts initially. Remove later as needed.
  contact_list_.clear();
  contact_list_.push_back(alfoot_contact_);
  contact_list_.push_back(arfoot_contact_);
  contact_list_.push_back(blfoot_contact_);  
  contact_list_.push_back(brfoot_contact_); 

  // foot contact map
  foot_contact_map_.clear();
  foot_contact_map_[MagnetoFoot::AL] = alfoot_contact_;
  foot_contact_map_[MagnetoFoot::AR] = arfoot_contact_;
  foot_contact_map_[MagnetoFoot::BL] = blfoot_contact_;  
  foot_contact_map_[MagnetoFoot::BR] = brfoot_contact_; 
  full_dim_contact_ = dim_contact_;
}


void MagnetoTaskAndForceContainer::_InitializeMagnetisms(){
  magnetic_force_ = 0.;
  residual_ratio_ = 0.;
  b_magnetism_map_[MagnetoBodyNode::AL_foot_link] = false;
  b_magnetism_map_[MagnetoBodyNode::BL_foot_link] = false;
  b_magnetism_map_[MagnetoBodyNode::AR_foot_link] = false;
  b_magnetism_map_[MagnetoBodyNode::BR_foot_link] = false;
}

void MagnetoTaskAndForceContainer::_DeleteTasks() {
  task_list_.clear();

  delete joint_task_;
  delete com_task_;
  delete base_ori_task_;

  delete alfoot_pos_task_;
  delete arfoot_pos_task_;
  delete blfoot_pos_task_;
  delete brfoot_pos_task_;

  delete alfoot_ori_task_;
  delete arfoot_ori_task_;
  delete blfoot_ori_task_;
  delete brfoot_ori_task_;  
}

void MagnetoTaskAndForceContainer::_DeleteContacts() {
  contact_list_.clear();

  delete alfoot_contact_;
  delete arfoot_contact_;
  delete blfoot_contact_;
  delete brfoot_contact_;
}

void MagnetoTaskAndForceContainer::setContactFriction() {
  setContactFriction(friction_coeff_);
} 

void MagnetoTaskAndForceContainer::setContactFriction(const Eigen::VectorXd& _mu_vec) {
  // initialize contact
  for (int foot_idx=0; foot_idx<Magneto::n_leg; ++i)
    setContactFriction(foot_idx, _mu_vec[foot_idx]);  
}

void MagnetoTaskAndForceContainer::setContactFriction(int foot_idx, double mu) {
  // initialize contact
  ((BodyFrameSurfaceContactSpec*)foot_contact_map_[foot_idx])->setFrictionCoeff(mu);
}

void MagnetoTaskAndForceContainer::paramInitialization(const YAML::Node& node) {
  
  try {
    
    my_utils::readParameter(node, "w_qddot", w_qddot_);
    my_utils::readParameter(node, "w_xddot", w_xddot_contact_);
    my_utils::readParameter(node, "w_xddot_nocontact", w_xddot_nocontact_);
    my_utils::readParameter(node, "w_rf", w_rf_);
    my_utils::readParameter(node, "w_rf_z", w_rf_z_contact_);
    my_utils::readParameter(node, "w_rf_z_nocontact", w_rf_z_nocontact_);
    my_utils::readParameter(node, "max_rf_z", max_rf_z_contact_);
    my_utils::readParameter(node, "max_rf_z_nocontact", max_rf_z_nocontact_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  // initialize parameters
  int dim_contact = alfoot_contact_->getDim();
  int idx_rf_z = alfoot_contact_->getFzIndex();

  W_qddot_ = Eigen::VectorXd::Constant(Magneto::n_dof, w_qddot_);
  W_xddot_contact_  = Eigen::VectorXd::Constant(dim_contact, w_xddot_contact_);
  W_xddot_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_xddot_contact_);
  W_xddot_nocontact_[idx_rf_z] = w_xddot_nocontact_;
  W_rf_contact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_contact_[idx_rf_z] = w_rf_z_contact_;
  W_rf_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_nocontact_[idx_rf_z] = w_rf_z_nocontact_;

  // Set Maximum Forces
  ((BodyFrameSurfaceContactSpec*)alfoot_contact_)->setMaxFz(max_rf_z_contact_);
  ((BodyFrameSurfaceContactSpec*)blfoot_contact_)->setMaxFz(max_rf_z_contact_);
  ((BodyFrameSurfaceContactSpec*)arfoot_contact_)->setMaxFz(max_rf_z_contact_);
  ((BodyFrameSurfaceContactSpec*)brfoot_contact_)->setMaxFz(max_rf_z_contact_);
}

// -------------------------------------------------------
//    set functions
// -------------------------------------------------------
void MagnetoTaskAndForceContainer::set_magnetism(int moving_cop) {
  b_magnetism_map_[MagnetoBodyNode::AL_foot_link]
                        = (moving_cop != MagnetoBodyNode::AL_tibia_link && 
                        moving_cop != MagnetoBodyNode::AL_foot_link);
  b_magnetism_map_[MagnetoBodyNode::BL_foot_link] 
                        = (moving_cop != MagnetoBodyNode::BL_tibia_link && 
                        moving_cop != MagnetoBodyNode::BL_foot_link); 
  b_magnetism_map_[MagnetoBodyNode::AR_foot_link] 
                        = (moving_cop != MagnetoBodyNode::AR_tibia_link && 
                        moving_cop != MagnetoBodyNode::AR_foot_link);
  b_magnetism_map_[MagnetoBodyNode::BR_foot_link] 
                        = (moving_cop != MagnetoBodyNode::BR_tibia_link && 
                        moving_cop != MagnetoBodyNode::BR_foot_link);
}

void MagnetoTaskAndForceContainer::set_residual_magnetic_force(int moving_cop, double contact_distance) {

  if( moving_cop >= 0 && moving_cop < robot_->getNumBodyNodes() ){
    // set J_residual_
    J_residual_ = robot_->getBodyNodeCoMBodyJacobian(moving_cop); // 6 x ndof
    // set F_residual_
    double distance_ratio;
    double distance_constant = 0.005*4.;
    distance_ratio = distance_constant / (contact_distance + distance_constant);
    distance_ratio = distance_ratio * distance_ratio;
    residual_force_ = distance_ratio * magnetic_force_ * (residual_ratio_/100.);
    F_residual_ = Eigen::VectorXd::Zero(J_residual_.rows());
    F_residual_[F_residual_.size()-1] = residual_force_;
  } else{
    J_residual_ = Eigen::MatrixXd::Zero(6, robot_->getNumDofs()); // 6 x ndof
    F_residual_ = Eigen::VectorXd::Zero(J_residual_.rows());
  }
}

void MagnetoTaskAndForceContainer::set_contact_magnetic_force(int moving_cop) {
  // set F_magnetic_ based on b_magnetism_map_  
  F_magnetic_ = Eigen::VectorXd::Zero(full_dim_contact_);

  int contact_link_idx, fz_idx;
  int dim_contact(0);
  for(auto &[leg_idx, contact] : foot_contact_map_) {
    contact_link_idx = ((BodyFrameSurfaceContactSpec*)(contact))->getLinkIdx();
    if( contact_link_idx != moving_cop) {  
      fz_idx = dim_contact + ((BodyFrameSurfaceContactSpec*)(contact))->getFzIndex();    
      if(b_magnetism_map_[contact_link_idx]){
        F_magnetic_[fz_idx] = magnetic_force_[leg_idx];
      }
      else {
        F_magnetic_[fz_idx] = residual_force_[leg_idx];
        // F_magnetic_[fz_idx] = -residual_force_;
      }
      dim_contact += ((BodyFrameSurfaceContactSpec*)(contact))->getDim();
    }
  }
  F_magnetic_ = F_magnetic_.head(dim_contact);  
}


void MagnetoTaskAndForceContainer::set_contact_list(int moving_cop) {
  // build contact_list_
  dim_contact_=0;
  contact_list_.clear();
  for(auto &it : full_contact_list_) {
    if(((BodyFrameSurfaceContactSpec*)(it))->getLinkIdx()!=moving_cop) {
      contact_list_.push_back((BodyFrameSurfaceContactSpec*)(it));
      dim_contact_ += (it)->getDim();
    }
  }
}

void MagnetoTaskAndForceContainer::set_maxfz_contact(int moving_cop) {
  set_maxfz_contact(moving_cop,
                    max_rf_z_contact_, 
                    max_rf_z_nocontact_);
}

void MagnetoTaskAndForceContainer::set_maxfz_contact(int moving_cop,
                                                  double max_rfz_cntct,
                                                  double max_rfz_nocntct) {
  for(auto &it : contact_list_) {
    if(((BodyFrameSurfaceContactSpec*)(it))->getLinkIdx() == moving_cop) {
      ((BodyFrameSurfaceContactSpec*)(it))->setMaxFz(max_rfz_nocntct);
    } else {
      ((BodyFrameSurfaceContactSpec*)(it))->setMaxFz(max_rfz_cntct);
    }
  }
}
void MagnetoTaskAndForceContainer::compute_weight_param(int moving_cop,
                                    const Eigen::VectorXd &W_contact,
                                    const Eigen::VectorXd &W_nocontact,
                                    Eigen::VectorXd &W_result) {
  int dim_vec = W_contact.size();
  int num_contact = contact_list_.size();
  int idx_accum = 0;
  W_result = Eigen::VectorXd::Zero(dim_vec*num_contact);
  for(auto it = contact_list_.begin(); 
        it != contact_list_.end(); it++) {
    if(((BodyFrameSurfaceContactSpec*)(*it))->getLinkIdx()==moving_cop) {
      // swing foot - no contact
      W_result.segment(idx_accum,dim_vec) = W_nocontact;

    } else {
      W_result.segment(idx_accum,dim_vec) = W_contact;
    }
    idx_accum += dim_vec;
  }
}

void MagnetoTaskAndForceContainer::clear_task_list() {
  task_list_.clear();
}
void MagnetoTaskAndForceContainer::add_task_list(Task* task) {
  task_list_.push_back(task);
}
Task* MagnetoTaskAndForceContainer::get_foot_pos_task(int moving_cop) {
  switch(moving_cop){
        case MagnetoBodyNode::AL_foot_link :
        case MagnetoBodyNode::AL_tibia_link :
            return alfoot_pos_task_;
        case MagnetoBodyNode::BL_foot_link :
        case MagnetoBodyNode::BL_tibia_link :
            return blfoot_pos_task_;
        case MagnetoBodyNode::AR_foot_link :
        case MagnetoBodyNode::AR_tibia_link :
            return arfoot_pos_task_;
        case MagnetoBodyNode::BR_foot_link :
        case MagnetoBodyNode::BR_tibia_link :
            return brfoot_pos_task_;
        default:
            return NULL;
  }
}
Task* MagnetoTaskAndForceContainer::get_foot_ori_task(int moving_cop) {
  switch(moving_cop){
        case MagnetoBodyNode::AL_foot_link :
        case MagnetoBodyNode::AL_tibia_link :
            return alfoot_ori_task_;
        case MagnetoBodyNode::BL_foot_link :
        case MagnetoBodyNode::BL_tibia_link :
            return blfoot_ori_task_;
        case MagnetoBodyNode::AR_foot_link :
        case MagnetoBodyNode::AR_tibia_link :
            return arfoot_ori_task_;
        case MagnetoBodyNode::BR_foot_link :
        case MagnetoBodyNode::BR_tibia_link :
            return brfoot_ori_task_;
        default:
            return NULL;
  }
}
