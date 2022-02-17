#include <my_utils/Math/MathUtilities.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>

MagnetoWbcSpecContainer::MagnetoWbcSpecContainer(RobotSystem* _robot) {
  my_utils::pretty_constructor(2, "Magneto Task And Force Container");
  robot_ = _robot;
  _InitializeTasks();
  _InitializeContacts();
  _InitializeMagnetisms();
  _InitializeWeightParams();
}

MagnetoWbcSpecContainer::~MagnetoWbcSpecContainer() {
  _DeleteTasks();
  _DeleteContacts();
  _DeleteMagnetisms();
  _DeleteOthers();
}

void MagnetoWbcSpecContainer::_InitializeTasks() {
  // CoM and Pelvis Tasks
  com_task_ = 
      new CoMTask(robot_);
  base_ori_task_ = 
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::base_link); 
  joint_task_ = 
      new BasicTask(robot_, BasicTaskType::JOINT, Magneto::n_adof);

  // Set Foot Motion Tasks
  for(int i(0); i<Magneto::n_leg; ++i){
    feet_pos_tasks_[i] = new BasicTask(robot_, 
                    BasicTaskType::LINKXYZ, 3, MagnetoFoot::LinkIdx[i]);
    feet_ori_tasks_[i] = new BasicTask(robot_,
                  BasicTaskType::LINKORI, 3, MagnetoFoot::LinkIdx[i]);
  }

  // empty task list
  task_list_.clear();
}

void MagnetoWbcSpecContainer::_InitializeContacts() {

  friction_coeff_ = Eigen::VectorXd::Constant(Magneto::n_leg, 0.7); // updated later in setContactFriction 
  
  double foot_x = 0.02; // 0.05; 
  double foot_y = 0.02; 

  full_contact_dim_ = 0;
  for(int i(0); i<Magneto::n_leg; ++i){
    feet_contacts_[i] = new BodyFrameSurfaceContactSpec(
                        robot_, 
                        MagnetoFoot::LinkIdx[i], 
                        foot_x, foot_y, 
                        friction_coeff_[i]);

    full_contact_dim_ += feet_contacts_[i]->getDim();
    b_feet_contact_list_[i] = true;
  }

  // Add all contacts initially. Remove later as needed.
  contact_list_.clear();
}


void MagnetoWbcSpecContainer::_InitializeMagnetisms(){
  magnetic_force_ = Eigen::VectorXd::Zero(4);
  residual_ratio_ = Eigen::VectorXd::Zero(4);
  residual_force_ = Eigen::VectorXd::Zero(4);

  for(int i(0); i<Magneto::n_leg; ++i){
    feet_magnets_[i] = new MagnetSpec( feet_contacts_[i], 
                magnetic_force_[i], residual_ratio_[i] ); 
  }
}

void MagnetoWbcSpecContainer::_InitializeWeightParams(){
  for(int i(0); i<Magneto::n_leg; ++i){
    feet_weights_[i] = new ContactWeight(feet_contacts_[i]);
  }
}

void MagnetoWbcSpecContainer::_DeleteTasks() {
  task_list_.clear();

  delete joint_task_;
  delete com_task_;
  delete base_ori_task_;

  for( auto &task : feet_pos_tasks_)
    delete task;

  for( auto &task : feet_ori_tasks_)
    delete task;
}

void MagnetoWbcSpecContainer::_DeleteContacts() {
  contact_list_.clear();

  for( auto &contact : feet_contacts_)
    delete contact;
}

void MagnetoWbcSpecContainer::_DeleteMagnetisms() {
    for( auto &mag : feet_magnets_)
    delete mag;
}

void MagnetoWbcSpecContainer::_DeleteOthers() {
    for( auto &we : feet_weights_)
    delete we;
}

void MagnetoWbcSpecContainer::setContactFriction() {
  setContactFriction(friction_coeff_);
} 

void MagnetoWbcSpecContainer::setContactFriction(const Eigen::VectorXd& _mu_vec) {
  // initialize contact
  for (int foot_idx=0; foot_idx<Magneto::n_leg; ++foot_idx)
    setContactFriction(foot_idx, _mu_vec[foot_idx]);  
}

void MagnetoWbcSpecContainer::setContactFriction(int foot_idx, double mu) {
  // initialize contact
  feet_contacts_[foot_idx]->setFrictionCoeff(mu);
}

void MagnetoWbcSpecContainer::setMagneticForce() {
  setMagneticForce(magnetic_force_, residual_ratio_);
}

void MagnetoWbcSpecContainer::setMagneticForce(const Eigen::VectorXd& magnetic_force,
                                            const Eigen::VectorXd& residual_ratio) {
  for (int foot_idx=0; foot_idx<Magneto::n_leg; ++foot_idx)
    setMagneticForce(foot_idx, magnetic_force[foot_idx], residual_ratio[foot_idx]);  
}
void MagnetoWbcSpecContainer::setMagneticForce(int foot_idx, double fm, double rr){
  feet_magnets_[foot_idx]->setMagneticForce(fm);
  feet_magnets_[foot_idx]->setResidualRatio(rr);
}



void MagnetoWbcSpecContainer::weightParamInitialization(const YAML::Node& node) {
  
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

  W_qddot_ = Eigen::VectorXd::Constant(Magneto::n_dof, w_qddot_);

  W_xddot_contact_  = Eigen::VectorXd::Constant(dim_contact, w_xddot_);
  W_xddot_contact_[idx_rf_z] = w_xddot_z_contact_;
  W_xddot_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_xddot_);
  W_xddot_nocontact_[idx_rf_z] = w_xddot_z_nocontact_;
  W_rf_contact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_contact_[idx_rf_z] = w_rf_z_contact_;
  W_rf_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_nocontact_[idx_rf_z] = w_rf_z_nocontact_;

  // Set Maximum Forces
  for(int i(0); i<Magneto::n_leg; ++i)
    feet_contacts_[i]->setMaxFz(max_rf_z_contact_);

  for(int i(0); i<Magneto::n_leg; ++i){
    feet_weights_[i]->setWeightRF(w_rf_, w_rf_z_contact_);
    feet_weights_[i]->setWeightXddot(w_xddot_, w_xddot_z_contact_);
  }    
  
  
}

void MagnetoWbcSpecContainer::contactParamInitialization(const YAML::Node& node){
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

void MagnetoWbcSpecContainer::magneticParamInitialization(const YAML::Node& node){
  try { 
    // magnetism
    my_utils::readParameter(node,"magnetic_force", magnetic_force_);  
    my_utils::readParameter(node,"residual_ratio", residual_ratio_); 
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  setMagneticForce();
}

// -------------------------------------------------------
//    set functions
// -------------------------------------------------------
void MagnetoWbcSpecContainer::set_foot_magnet_off(int moving_cop) {
  bool onoff;
  for( auto &magnet : feet_magnets_){
    // magnet on if not swing foot 
    onoff = moving_cop != magnet->getLinkIdx();
    magnet->setMagnetOnoff(onoff);
  }
  update_magnet_forces();
}

void MagnetoWbcSpecContainer::set_magnet_distance(int moving_cop, 
                                          double contact_distance) {
  for( auto &magnet : feet_magnets_){
    // onoff 
    if( moving_cop == magnet->getLinkIdx() ) 
      magnet->setContactDistance(contact_distance);
    else magnet->setContactDistance(0.0);
  }
  update_magnet_forces();
}

std::map<FootLinkIdx, bool> MagnetoWbcSpecContainer::get_magnetism_map() {
  std::map<FootLinkIdx, bool> b_map;
  b_map.clear();
  for( auto &magnet : feet_magnets_){
    b_map[magnet->getLinkIdx()] = magnet->getOnOff();
  }
  return b_map;
}

void MagnetoWbcSpecContainer::update_magnet_forces() {
  // update F_residual_, J_residual_, F_magnetic_
  F_magnetic_ = Eigen::VectorXd::Zero(0);
  J_residual_ = Eigen::MatrixXd::Zero(0,0);
  F_residual_ = Eigen::VectorXd::Zero(0);  

  for( int i(0); i<Magneto::n_leg; ++i) { 
    // foot in contactlist : stack F_magnetic_         
    if(b_feet_contact_list_[i]) {
      F_magnetic_ = my_utils::vStack(F_magnetic_, 
                              feet_magnets_[i]->getMagneticForce());
    }
    // foot not in contactlist : stack F_residual_, J_residual_
    else {             
      F_residual_ = my_utils::vStack(F_residual_, 
                              feet_magnets_[i]->getMagneticForce());
      J_residual_ = my_utils::vStack(J_residual_, 
                              feet_magnets_[i]->getJacobian());
    }
  }
}

void MagnetoWbcSpecContainer::set_contact_list(int moving_cop) {
  // build contact_list_
  contact_list_.clear();
  for(int i(0); i<Magneto::n_leg; i++)
  {
    if( MagnetoFoot::LinkIdx[i] != moving_cop ) {
      contact_list_.push_back(feet_contacts_[i]);
      b_feet_contact_list_[i] = true;
    }
    else
      b_feet_contact_list_[i] = false;
  }
}

void MagnetoWbcSpecContainer::set_contact_maxfz(int moving_cop) {
  for(auto &contact : contact_list_) {
    if( contact->getLinkIdx() == moving_cop) {
      contact->setMaxFz(max_rf_z_trans_);
    } else {
      contact->setMaxFz(max_rf_z_contact_);
    }
  }
}

////////////////////

void MagnetoWbcSpecContainer::set_contact_weight_param(int trans_cop) {
  W_xddot_ = Eigen::VectorXd::Zero(0); 
  W_rf_ = Eigen::VectorXd::Zero(0);
  for(int i(0); i<Magneto::n_leg; ++i) {
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


void MagnetoWbcSpecContainer::reshape_weight_param(int slip_cop,
                                                    double alpha) {
  W_xddot_ = Eigen::VectorXd::Zero(0); 
  W_rf_ = Eigen::VectorXd::Zero(0);
  for(int i(0); i<Magneto::n_leg; ++i) {
    if(b_feet_contact_list_[i]) {
      // if trans
      if(feet_contacts_[i]->getLinkIdx()==slip_cop){
        feet_weights_[i]->reshapeWeightRF(alpha);  
        feet_weights_[i]->reshapeWeightXddot(alpha);         
      }else{
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

void MagnetoWbcSpecContainer::clear_task_list() {
  task_list_.clear();
}
void MagnetoWbcSpecContainer::add_task_list(Task* task) {
  task_list_.push_back(task);
}

Task* MagnetoWbcSpecContainer::get_foot_pos_task(int moving_cop) {
  int foot_idx = footLink2FootIdx(moving_cop);
  if(foot_idx<0) return NULL;  
  else return feet_pos_tasks_[foot_idx];    
}

Task* MagnetoWbcSpecContainer::get_foot_ori_task(int moving_cop) {
  int foot_idx = footLink2FootIdx(moving_cop);
  if(foot_idx<0) return NULL;  
  else return feet_ori_tasks_[foot_idx]; 
}

int MagnetoWbcSpecContainer::footLink2FootIdx(int moving_cop){
  for( int foot_idx(0); foot_idx<Magneto::n_leg; ++foot_idx){
    if(MagnetoFoot::LinkIdx[foot_idx] == moving_cop)
      return foot_idx;  }
  return -1;
} 