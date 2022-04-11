#include <my_robot_core/magneto_core/magneto_wbc_controller/magneto_mcwbc.hpp>

MagnetoMCWBC::MagnetoMCWBC( MagnetoWbcSpecContainer* _ws_container, 
                            RobotSystem* _robot, int _controller_type ){
  my_utils::pretty_constructor(2, "Magnetic Contact Whole Body Controller");
  // Initialize Flag
  b_first_visit_ = true;

  // Initialize Pointer to the Task and Force Container
  ws_container_ = _ws_container;
  robot_ = _robot;

  // Initialize State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // Initialize Actuator selection list  
  act_list_.resize(Magneto::n_dof, true);
  for (int i(0); i < Magneto::n_vdof; ++i) 
      act_list_[Magneto::idx_vdof[i]] = false;

  // Initialize WBC 
  if(_controller_type == MCWBC_TYPES::MFWBCC)
    mcwbc_ = new MFWBCC(act_list_);
  else if(_controller_type == MCWBC_TYPES::MRWBCC)
    mcwbc_ = new MRWBCC(act_list_);
  else{
    std::cout<<"error @ MagnetoMCWBC : invalid controller_type"<<_controller_type<<std::endl;
    exit(0);
  }
  mcwbc_param_ = new MCWBC_ExtraData();
  kin_wbc_ = new KinWBC(act_list_);
  
  tau_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);

  // Initialize desired pos, vel, acc containers
  jpos_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  jvel_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  jacc_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
}

MagnetoMCWBC::~MagnetoMCWBC() {
  delete mcwbc_;
  delete mcwbc_param_;
}

void MagnetoMCWBC::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Grab Variables from the container.
  mcwbc_param_->W_qddot_ = ws_container_->W_qddot_;
  mcwbc_param_->W_xddot_ = ws_container_->W_xddot_;
  mcwbc_param_->W_rf_ = ws_container_->W_rf_;

  // Clear out local pointers
  task_list_.clear();
  contact_list_.clear();
  magnet_list_.clear();

  // Update task and contact list pointers from container object
  for (int i = 0; i < ws_container_->task_list_.size(); i++) {
    task_list_.push_back(ws_container_->task_list_[i]);
  }
  for (int i = 0; i < ws_container_->contact_list_.size(); i++) {
    contact_list_.push_back(ws_container_->contact_list_[i]);
  }

  for (int i=0; i<ws_container_->feet_magnets_.size(); i++) {
    magnet_list_.push_back(ws_container_->feet_magnets_[i]);
  }

  // Update Contact Spec
  for (int i = 0; i < contact_list_.size(); i++) {
    contact_list_[i]->updateContactSpec();
  }
}

void MagnetoMCWBC::getCommand(void* _cmd) {

  // grab & update task_list and contact_list & QP weights
  _PreProcessing_Command();

  // ---- Solve Inv Kinematics
  
  // kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, 
  //                               jpos_des_, jvel_des_, jacc_des_); 
  // my_utils::saveVector(jpos_des_, "jpos_des");
  // my_utils::saveVector(jvel_des_, "jvel_des");
  kin_wbc_->FindFullConfiguration(sp_->q, task_list_, contact_list_, 
                                    jpos_des_, jvel_des_, jacc_des_); 
  // my_utils::saveVector(jpos_des_, "jpos_des_full");
  // my_utils::saveVector(jvel_des_, "jvel_des_full");
                
  Eigen::VectorXd jacc_des_cmd = jacc_des_;
  // for(int i(0); i<Magneto::n_dof; ++i) {
  //   jacc_des_cmd[i] +=
  //         Kp_[0]*(jpos_des_[i] - sp_->q[i])
  //         + Kd_[0]*(jvel_des_[i] - sp_->qdot[i]);
  // }
  for(int i(0); i<Magneto::n_adof; ++i) {
    jacc_des_cmd[Magneto::idx_adof[i]] +=
          Kp_[i]*(jpos_des_[Magneto::idx_adof[i]] - sp_->q[Magneto::idx_adof[i]])
          + Kd_[i]*(jvel_des_[Magneto::idx_adof[i]] - sp_->qdot[Magneto::idx_adof[i]]);
  }
                                
  // wbmc
  mcwbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  mcwbc_->makeTorqueGivenRef(jacc_des_cmd, contact_list_, magnet_list_, jtrq_des_, mcwbc_param_);
  
  ((MagnetoCommand*)_cmd)->jtrq = jtrq_des_;
  ((MagnetoCommand*)_cmd)->q = sp_->getActiveJointValue(jpos_des_);
  ((MagnetoCommand*)_cmd)->qdot = sp_->getActiveJointValue(jvel_des_);
  ws_container_->get_magnetism_onoff(((MagnetoCommand*)_cmd)->magnetism_onoff);
  

  // _PostProcessing_Command(); // unset task and contact

  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // my_utils::pretty_print(jpos_des_, std::cout, "jpos_des_");
  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->q, std::cout, "q");
  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->qdot, std::cout, "qdot");
}


void MagnetoMCWBC::firstVisit() { 
  
}

void MagnetoMCWBC::ctrlInitialization(const YAML::Node& node) {
  // WBC Defaults
  wbc_dt_ = MagnetoAux::servo_rate;
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
      Eigen::VectorXd::Constant(Magneto::n_adof, -torque_limit_); //-2500.
  Eigen::VectorXd tau_max =
      // sp_->getActiveJointValue(robot_->GetTorqueUpperLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, torque_limit_); //-2500.
  mcwbc_->setTorqueLimits(tau_min, tau_max);

  // Set Joint Integrator Parameters
}
