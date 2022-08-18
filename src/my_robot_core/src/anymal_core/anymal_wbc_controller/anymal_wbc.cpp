#include <my_robot_core/anymal_core/anymal_wbc_controller/anymal_wbc.hpp>

ANYmalWBC::ANYmalWBC( ANYmalWbcSpecContainer* _ws_container, 
                            RobotSystem* _robot){
  my_utils::pretty_constructor(2, "ANYMal Whole Body Controller");
  // Initialize Flag
  b_first_visit_ = true;

  // Initialize Pointer to the Task and Force Container
  ws_container_ = _ws_container;
  robot_ = _robot;

  // Initialize State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);

  // Initialize Actuator selection list  
  act_list_.resize(ANYmal::n_dof, true);
  for (int i(0); i < ANYmal::n_vdof; ++i) 
      act_list_[ANYmal::idx_vdof[i]] = false;

  // Initialize WBC 
  wbc_ = new WBLC(act_list_);
  wbc_param_ = new WBLC_ExtraData();
  kin_wbc_ = new KinWBC(act_list_);
  
  tau_cmd_ = Eigen::VectorXd::Zero(ANYmal::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(ANYmal::n_adof);

  // Initialize desired pos, vel, acc containers
  jpos_des_ = Eigen::VectorXd::Zero(ANYmal::n_dof);
  jvel_des_ = Eigen::VectorXd::Zero(ANYmal::n_dof);
  jacc_des_ = Eigen::VectorXd::Zero(ANYmal::n_dof);
}

ANYmalWBC::~ANYmalWBC() {
  delete wbc_;
  delete wbc_param_;
}

void ANYmalWBC::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Update task and contact list pointers from container object
  task_list_.clear();  
  for (int i = 0; i < ANYMAL_TASK::n_task; i++) {
    if(ws_container_->b_task_list_[i])
      task_list_.push_back(ws_container_->task_container_[i]);
  }
  contact_list_.clear();  
  for (int i = 0; i < ANYmal::n_leg; i++) {
    if(ws_container_->b_feet_contact_list_[i]) {
      ws_container_->feet_contacts_[i]->updateContactSpec();
      contact_list_.push_back(ws_container_->feet_contacts_[i]);
    }
  }

  // update weight params
  Eigen::VectorXd W_xddot = Eigen::VectorXd::Zero(0);
  Eigen::VectorXd W_rf = Eigen::VectorXd::Zero(0);
  for (int i = 0; i < ANYmal::n_leg; i++) {  
    if(ws_container_->b_feet_contact_list_[i]) {
      W_xddot= my_utils::vStack(W_xddot, 
            ws_container_->feet_weights_[i]->getWxddot());
      W_rf = my_utils::vStack(W_rf, 
            ws_container_->feet_weights_[i]->getWrf());
    }
  }
  wbc_param_->W_qddot_ = ws_container_->W_qddot_;
  wbc_param_->W_xddot_ = W_xddot;
  wbc_param_->W_rf_ = W_rf;
}

void ANYmalWBC::getCommand(void* _cmd) {

  // grab & update task_list and contact_list & QP weights
  _PreProcessing_Command();

  // ---- Solve Inv Kinematics
  // kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, 
  //                               jpos_des_, jvel_des_, jacc_des_); 
  kin_wbc_->FindFullConfiguration(sp_->q, task_list_, contact_list_, 
                                    jpos_des_, jvel_des_, jacc_des_); 

  Eigen::VectorXd jacc_des_cmd = jacc_des_;
  for(int i(0); i<ANYmal::n_adof; ++i) {
    jacc_des_cmd[ANYmal::idx_adof[i]] +=
          Kp_[i]*(jpos_des_[ANYmal::idx_adof[i]] - sp_->q[ANYmal::idx_adof[i]])
          + Kd_[i]*(jvel_des_[ANYmal::idx_adof[i]] - sp_->qdot[ANYmal::idx_adof[i]]);
  }
                               
  // wbmc
  wbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  wbc_->makeTorqueGivenRef(jacc_des_cmd, contact_list_, jtrq_des_, wbc_param_);

  set_grf_des();
  
  ((ANYmalCommand*)_cmd)->jtrq = jtrq_des_;
  ((ANYmalCommand*)_cmd)->q = sp_->getActiveJointValue(jpos_des_);
  ((ANYmalCommand*)_cmd)->qdot = sp_->getActiveJointValue(jvel_des_);

  // _PostProcessing_Command(); // unset task and contact

   

  // my_utils::pretty_print(((ANYmalCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // my_utils::pretty_print(jpos_des_, std::cout, "jpos_des_");
  // my_utils::pretty_print(((ANYmalCommand*)_cmd)->q, std::cout, "q");
  // my_utils::pretty_print(((ANYmalCommand*)_cmd)->qdot, std::cout, "qdot");
}


void ANYmalWBC::firstVisit() { 
  
}

void ANYmalWBC::set_grf_des(){


  std::array<Eigen::VectorXd, ANYmal::n_leg> grf_des_list;  
  // initialize
  for(int foot_idx(0); foot_idx<ANYmal::n_leg; ++foot_idx) 
    grf_des_list[foot_idx] = Eigen::VectorXd::Zero(6);
  

  int dim_grf_stacked(0), dim_grf(0), foot_idx;
  for ( auto &contact : contact_list_) {
    foot_idx = ws_container_->footLink2FootIdx(contact->getLinkIdx());
    if(foot_idx>-1 && foot_idx< ANYmal::n_leg){
      dim_grf = contact->getDim();
      grf_des_list[foot_idx]  = wbc_param_->Fr_.segment(dim_grf_stacked, dim_grf);
      dim_grf_stacked += dim_grf;    
    }else{
      std::cout<<"set_grf_des??? foot_idx = "<< foot_idx << std::endl;
      std::cout<<"set_grf_des???? link idx = "<< contact->getLinkIdx() << std::endl;
    }    
  }

  sp_->foot_rf_des = grf_des_list;  
}

void ANYmalWBC::ctrlInitialization(const YAML::Node& node) {
  // WBC Defaults
  wbc_dt_ = ANYmalAux::servo_rate;
  b_enable_torque_limits_ = true;  // Enable WBC torque limits

  // Joint Integrator Defaults
  vel_freq_cutoff_ = 2.0;  // Hz
  pos_freq_cutoff_ = 1.0;  // Hz
  max_pos_error_ = 0.2;    // Radians

  // Load Custom Parmams ----------------------------------
  try {
    // Load Integration Parameters
    my_utils::readParameter(node, "enable_torque_limits", b_enable_torque_limits_);
    my_utils::readParameter(node, "torque_limit", torque_limit_); 

    my_utils::readParameter(node, "velocity_freq_cutoff", vel_freq_cutoff_);
    my_utils::readParameter(node, "position_freq_cutoff", pos_freq_cutoff_);
    my_utils::readParameter(node, "max_position_error", max_pos_error_);

    my_utils::readParameter(node, "kp", Kp_);
    my_utils::readParameter(node, "kd", Kd_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  // ----------------------------------

  // Set WBC Parameters
  // Enable Torque Limits

  Eigen::VectorXd tau_min =
      // sp_->getActiveJointValue(robot_->GetTorqueLowerLimits());
      Eigen::VectorXd::Constant(ANYmal::n_adof, -torque_limit_); //-2500.
  Eigen::VectorXd tau_max =
      // sp_->getActiveJointValue(robot_->GetTorqueUpperLimits());
      Eigen::VectorXd::Constant(ANYmal::n_adof, torque_limit_); //-2500.
  wbc_->setTorqueLimits(tau_min, tau_max);

  // Set Joint Integrator Parameters
}
