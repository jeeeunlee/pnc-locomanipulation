#include <my_utils/Math/MathUtilities.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>

ANYmalWbcSpecContainer::ANYmalWbcSpecContainer(RobotSystem* _robot) {
  my_utils::pretty_constructor(2, "ANYmal Task And Force Container");
  robot_ = _robot;
  // ---------------------------------
  //        INITIALIZE CONTACT
  // ---------------------------------
  // CoM and Pelvis Tasks  
  task_container_[ANYMAL_TASK::COM] = 
      new CoMTask(robot_);
  task_container_[ANYMAL_TASK::BASE_ORI] = 
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, ANYmalBodyNode::base); 
  task_container_[ANYMAL_TASK::JOINT_TASK] = 
      new BasicTask(robot_, BasicTaskType::JOINT, ANYmal::n_adof);

  // Set Foot Motion Tasks
  for(int i(0); i<ANYmal::n_leg; ++i){
    task_container_[ANYMAL_TASK::LF_POS+i] = new BasicTask(robot_, 
                    BasicTaskType::LINKXYZ, 3, ANYmalFoot::LinkIdx[i]);
    task_container_[ANYMAL_TASK::LF_ORI+i]  = new BasicTask(robot_,
                  BasicTaskType::LINKORI, 3, ANYmalFoot::LinkIdx[i]);
  } 
  
  task_container_[ANYMAL_TASK::EE_POS] = new BasicTask(robot_, 
                    BasicTaskType::LINKXYZ, 3, ANYmalEE::EEarm );
  task_container_[ANYMAL_TASK::EE_ORI] = new BasicTask(robot_,
                    BasicTaskType::LINKORI, 3, ANYmalEE::EEarm );
  // initialize bool task
  for(int i(0);i<ANYMAL_TASK::n_task;++i)
    b_task_list_[i] = false;

  // ---------------------------------
  //      INITIALIZE CONTACT
  // ---------------------------------
  friction_coeff_ = Eigen::VectorXd::Constant(ANYmal::n_leg, 0.7); 
  // updated later in setContactFriction
  full_contact_dim_ = 0;
  for(int i(0); i<ANYmal::n_leg; ++i) {
    feet_contacts_[i] = new GroundFramePointContactSpec(
                              robot_, 
                              ANYmalFoot::LinkIdx[i], 
                              friction_coeff_[i] );

    full_contact_dim_ += feet_contacts_[i]->getDim();
    b_feet_contact_list_[i] = true;
  }

  // ---------------------------------
  //        INITIALIZE WEIGHT
  // ---------------------------------
  for(int i(0); i<ANYmal::n_leg; ++i){
    feet_weights_[i] = new ContactWeight(feet_contacts_[i]);
  }
}

ANYmalWbcSpecContainer::~ANYmalWbcSpecContainer() {
  for( auto &task : task_container_)
    delete task;
  for( auto &contact : feet_contacts_)
    delete contact;
  for( auto &weight : feet_weights_)
    delete weight;
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
  W_qddot_ = Eigen::VectorXd::Constant(ANYmal::n_dof, w_qddot_);

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

  for(int foot_idx(0); foot_idx<ANYmal::n_leg;++foot_idx)
    feet_contacts_[foot_idx]->setFrictionCoeff(friction_coeff_[foot_idx]);
}


////////////////////


void ANYmalWbcSpecContainer::reshape_weight_param( double alpha,
                                                    int slip_cop,
                                                    int swing_cop) {
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
    }
  }
}

void ANYmalWbcSpecContainer::clear_task_list() {
  for(int i(0);i<ANYMAL_TASK::n_task;++i)
    b_task_list_[i] = false;
}

void ANYmalWbcSpecContainer::check_task_list(){
  std::cout<<"check_task_list : ";
  for(int i(0);i<ANYMAL_TASK::n_task;++i)
    std::cout<< b_task_list_[i]<< ", ";
  std::cout<<std::endl;
}

int ANYmalWbcSpecContainer::footLink2FootIdx(int moving_cop) {
  for( int foot_idx(0); foot_idx<ANYmal::n_leg; ++foot_idx) {
    if(ANYmalFoot::LinkIdx[foot_idx] == moving_cop)
      return foot_idx; }
  return -1;
}