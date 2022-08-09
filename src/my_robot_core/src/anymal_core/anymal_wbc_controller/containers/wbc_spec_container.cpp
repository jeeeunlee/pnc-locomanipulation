#include <my_utils/Math/MathUtilities.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>

ANYmalWbcSpecContainer::ANYmalWbcSpecContainer(RobotSystem* _robot) {
  my_utils::pretty_constructor(2, "ANYmal Task And Force Container");
  robot_ = _robot;
  _InitializeTasks();
  _InitializeContacts();
  _InitializeWeightParams();
}

ANYmalWbcSpecContainer::~ANYmalWbcSpecContainer() {
  _DeleteTasks();
  _DeleteContacts();
  _DeleteOthers();
}

void ANYmalWbcSpecContainer::_InitializeTasks() {
  // CoM and Pelvis Tasks
  com_task_ = 
      new CoMTask(robot_);
  base_ori_task_ = 
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, ANYmalBodyNode::base); 
  joint_task_ = 
      new BasicTask(robot_, BasicTaskType::JOINT, ANYmal::n_adof);

  // Set Foot Motion Tasks
  for(int i(0); i<ANYmal::n_leg; ++i){
    feet_pos_tasks_[i] = new BasicTask(robot_, 
                    BasicTaskType::LINKXYZ, 3, ANYmalFoot::LinkIdx[i]);
    feet_ori_tasks_[i] = new BasicTask(robot_,
                  BasicTaskType::LINKORI, 3, ANYmalFoot::LinkIdx[i]);
  }

  ee_pos_task_ = new BasicTask(robot_, 
                    BasicTaskType::LINKXYZ, 3, ANYmalEE::EEarm );
  ee_ori_task_ = new BasicTask(robot_,
                    BasicTaskType::LINKORI, 3, ANYmalEE::EEarm );

  // empty task list
  task_list_.clear();
}

void ANYmalWbcSpecContainer::_InitializeContacts() {

  friction_coeff_ = Eigen::VectorXd::Constant(ANYmal::n_leg, 0.7); // updated later in setContactFriction
  
  full_contact_dim_ = 0;
  for(int i(0); i<ANYmal::n_leg; ++i) {
    feet_contacts_[i] = new GroundFramePointContactSpec(
                              robot_, 
                              ANYmalFoot::LinkIdx[i], 
                              friction_coeff_[i] );

    full_contact_dim_ += feet_contacts_[i]->getDim();
    b_feet_contact_list_[i] = true;
  }

  // Add all contacts initially. Remove later as needed.
  contact_list_.clear();
}

void ANYmalWbcSpecContainer::_InitializeWeightParams(){
  for(int i(0); i<ANYmal::n_leg; ++i){
    feet_weights_[i] = new ContactWeight(feet_contacts_[i]);
  }
}

void ANYmalWbcSpecContainer::_DeleteTasks() {
  task_list_.clear();

  delete joint_task_;
  delete com_task_;
  delete base_ori_task_;

  for( auto &task : feet_pos_tasks_)
    delete task;

  for( auto &task : feet_ori_tasks_)
    delete task;

  delete ee_pos_task_;
  delete ee_ori_task_;
}

void ANYmalWbcSpecContainer::_DeleteContacts() {
  contact_list_.clear();

  for( auto &contact : feet_contacts_)
    delete contact;
}

void ANYmalWbcSpecContainer::_DeleteOthers() {
    for( auto &we : feet_weights_)
    delete we;
}

void ANYmalWbcSpecContainer::setContactFriction() {
  setContactFriction(friction_coeff_);
} 

void ANYmalWbcSpecContainer::setContactFriction(const Eigen::VectorXd& _mu_vec) {
  // initialize contact
  for (int foot_idx=0; foot_idx<ANYmal::n_leg; ++foot_idx)
    setContactFriction(foot_idx, _mu_vec[foot_idx]);  
}

void ANYmalWbcSpecContainer::setContactFriction(int foot_idx, double mu) {
  // initialize contact
  feet_contacts_[foot_idx]->setFrictionCoeff(mu);
}

void ANYmalWbcSpecContainer::weightParamInitialization(const YAML::Node& node) {

  try {    
    my_utils::readParameter(node, "w_qddot", w_qddot_);
    my_utils::readParameter(node, "w_xddot", w_xddot_);    
    my_utils::readParameter(node, "w_xddot_z_contact", w_xddot_z_contact_);
    my_utils::readParameter(node, "w_xddot_z_nocontact", w_xddot_z_nocontact_);
    my_utils::readParameter(node, "w_rf", w_rf_);
    my_utils::readParameter(node, "w_rf_z_contact", w_rf_z_contact_);
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
  int dim_contact = feet_contacts_[0]->getDim();
  int idx_rf_z = feet_contacts_[0]->getFzIndex();

  W_qddot_ = Eigen::VectorXd::Constant(ANYmal::n_dof, w_qddot_);

  W_xddot_contact_  = Eigen::VectorXd::Constant(dim_contact, w_xddot_);
  W_xddot_contact_[idx_rf_z] = w_xddot_z_contact_;
  W_xddot_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_xddot_);
  W_xddot_nocontact_[idx_rf_z] = w_xddot_z_nocontact_;
  W_rf_contact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_contact_[idx_rf_z] = w_rf_z_contact_;
  W_rf_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_nocontact_[idx_rf_z] = w_rf_z_nocontact_;

  // Set Maximum Forces
  for(int i(0); i<ANYmal::n_leg; ++i)
    feet_contacts_[i]->setMaxFz(max_rf_z_contact_);

  for(int i(0); i<ANYmal::n_leg; ++i){
    feet_weights_[i]->setWeightRF(w_rf_, w_rf_z_contact_);
    feet_weights_[i]->setWeightXddot(w_xddot_, w_xddot_z_contact_);
  }     
  
}

void ANYmalWbcSpecContainer::contactParamInitialization(const YAML::Node& node){
try {
    my_utils::readParameter(node, "friction", friction_coeff_);  

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  setContactFriction();
}

void ANYmalWbcSpecContainer::set_contact_list(int moving_cop) {
  // build contact_list_
  contact_list_.clear();
  for(int i(0); i<ANYmal::n_leg; i++)
  {
    if( ANYmalFoot::LinkIdx[i] != moving_cop ) {
      contact_list_.push_back(feet_contacts_[i]);
      b_feet_contact_list_[i] = true;
    }
    else
      b_feet_contact_list_[i] = false;
  }
}

void ANYmalWbcSpecContainer::set_contact_maxfz(int moving_cop) {
  for(auto &contact : contact_list_) {
    if( contact->getLinkIdx() == moving_cop) {
      contact->setMaxFz(max_rf_z_trans_);
    } else {
      contact->setMaxFz(max_rf_z_contact_);
    }
  }
}

////////////////////

void ANYmalWbcSpecContainer::set_contact_weight_param(int trans_cop) {
  W_xddot_ = Eigen::VectorXd::Zero(0); 
  W_rf_ = Eigen::VectorXd::Zero(0);
  for(int i(0); i<ANYmal::n_leg; ++i) {
    // stack weight only if feet in contact list
    if(b_feet_contact_list_[i]) {
      // if trans
      if(feet_contacts_[i]->getLinkIdx()==trans_cop){
        feet_weights_[i]->setWeightRF(w_rf_, w_rf_z_trans_);
        feet_weights_[i]->setWeightXddot(w_xddot_, w_xddot_z_trans_); 
      }
      else{
        feet_weights_[i]->setWeightRF(w_rf_, w_rf_z_contact_);
        feet_weights_[i]->setWeightXddot(w_xddot_, w_xddot_z_contact_); 
      }      
      // stack weight
      W_xddot_ = my_utils::vStack(W_xddot_, 
                  feet_weights_[i]->getWxddot());
      W_rf_ = my_utils::vStack(W_rf_, 
                  feet_weights_[i]->getWrf());
    }
  }
}


void ANYmalWbcSpecContainer::reshape_weight_param( double alpha,
                                                    int slip_cop,
                                                    int swing_cop) {
  W_xddot_ = Eigen::VectorXd::Zero(0); 
  W_rf_ = Eigen::VectorXd::Zero(0);
  alpha = alpha*alpha;
  alpha = std::min(alpha, 400.);
  for(int i(0); i<ANYmal::n_leg; ++i) {
    if(b_feet_contact_list_[i]) {
      // if trans
      if(feet_contacts_[i]->getLinkIdx()==slip_cop){
        feet_weights_[i]->reshapeWeightRF(alpha);  
        feet_weights_[i]->reshapeWeightXddot(alpha);         
      }else if(feet_contacts_[i]->getLinkIdx()!=swing_cop){
        feet_weights_[i]->reshapeWeightRF(1/alpha);  // penalty other feet
        feet_weights_[i]->reshapeWeightXddot(1/alpha); 
      }
      // stack weight
      W_xddot_ = my_utils::vStack(W_xddot_, 
                  feet_weights_[i]->getWxddot());
      W_rf_ = my_utils::vStack(W_rf_, 
                  feet_weights_[i]->getWrf());
    }
  }

}

void ANYmalWbcSpecContainer::clear_task_list() {
  task_list_.clear();
}

void ANYmalWbcSpecContainer::add_task_list(Task* task) {
  task_list_.push_back(task);
}

Task* ANYmalWbcSpecContainer::get_foot_pos_task(int moving_cop) {
  int foot_idx = footLink2FootIdx(moving_cop);
  if(foot_idx<0) return NULL;  
  else return feet_pos_tasks_[foot_idx];    
}

Task* ANYmalWbcSpecContainer::get_foot_ori_task(int moving_cop) {
  int foot_idx = footLink2FootIdx(moving_cop);
  if(foot_idx<0) return NULL;  
  else return feet_ori_tasks_[foot_idx]; 
}

int ANYmalWbcSpecContainer::footLink2FootIdx(int moving_cop) {
  for( int foot_idx(0); foot_idx<ANYmal::n_leg; ++foot_idx) {
    if(ANYmalFoot::LinkIdx[foot_idx] == moving_cop)
      return foot_idx; }
  return -1;
}