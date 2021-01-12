#include <my_pnc/MagnetoPnC/MagnetoCtrl/MagnetoMainController.hpp>

MagnetoMainController::MagnetoMainController(
    MagnetoTaskAndForceContainer* _taf_container, RobotSystem* _robot) {
  my_utils::pretty_constructor(2, "Magneto Main Controller");
  // Initialize Flag
  b_first_visit_ = true;

  // Initialize Pointer to the Task and Force Container
  taf_container_ = _taf_container;
  robot_ = _robot;

  // Initialize State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // Initialize Actuator selection list
  
  act_list_.resize(Magneto::n_dof, true);
  for (int i(0); i < Magneto::n_vdof; ++i) 
      act_list_[Magneto::idx_vdof[i]] = false;

  // Initialize WBC
  kin_wbc_ = new KinWBC(act_list_);
  wbmc_ = new WBMC(act_list_);
  wbmc_param_ = new WBMC_ExtraData();

  tau_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);

  // Initialize desired pos, vel, acc containers
  jpos_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  jvel_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  jacc_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);
}

MagnetoMainController::~MagnetoMainController() {
  delete wbmc_;
}

void MagnetoMainController::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Clear out local pointers
  task_list_.clear();
  contact_list_.clear();

  // Grab Variables from the container.
  wbmc_param_->W_qddot_ = taf_container_->W_qddot_;
  wbmc_param_->W_xddot_ = taf_container_->W_xddot_;
  wbmc_param_->W_rf_ = taf_container_->W_rf_;
  wbmc_param_->F_magnetic_ = taf_container_->F_magnetic_;

  // my_utils::pretty_print(wbmc_param_->W_qddot_, std::cout, "W_qddot_");
  // my_utils::pretty_print(wbmc_param_->W_xddot_, std::cout, "W_xddot_");
  // my_utils::pretty_print(wbmc_param_->W_rf_, std::cout, "W_rf_");
  // my_utils::pretty_print(wbmc_param_->F_magnetic_, std::cout, "F_magnetic_");

  // Update task and contact list pointers from container object
  for (int i = 0; i < taf_container_->task_list_.size(); i++) {
    task_list_.push_back(taf_container_->task_list_[i]);
  }
  for (int i = 0; i < taf_container_->contact_list_.size(); i++) {
    contact_list_.push_back(taf_container_->contact_list_[i]);
  }

  // std::cout<<" task size = " << task_list_.size() << std::endl;
  // std::cout<<" contact size = " << contact_list_.size() << std::endl;

  // Update Task Jacobians and commands
  // for (int i = 0; i < task_list_.size(); i++) {
  //   task_list_[i]->updateJacobians();
  //   task_list_[i]->computeCommands();
  // }

  // Update Contact Spec
  for (int i = 0; i < contact_list_.size(); i++) {
    contact_list_[i]->updateContactSpec();
  }
}

void MagnetoMainController::getCommand(void* _cmd) {
  // grab & update task_list and contact_list & QP weights
  _PreProcessing_Command();

  // ---- Solve Inv Kinematics
  kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, 
                                jpos_des_, jvel_des_, jacc_des_); 
  
  // my_utils::pretty_print(jpos_des_, std::cout, "jpos_des_");
  // my_utils::pretty_print(jvel_des_, std::cout, "jvel_des_");
  // my_utils::pretty_print(jacc_des_, std::cout, "jacc_des_");

  Eigen::VectorXd jacc_des_cmd =
      jacc_des_ +
      Kp_.cwiseProduct(jpos_des_ - sp_->getActiveJointValue()) +
      Kd_.cwiseProduct(jvel_des_ - sp_->getActiveJointValue(sp_->qdot));

  // my_utils::pretty_print(jacc_des_cmd, std::cout, "jacc_des_cmd");
                                
  // wbmc
  wbmc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  wbmc_->makeTorqueGivenRef(jacc_des_cmd, contact_list_, jtrq_des_, wbmc_param_);

  // my_utils::pretty_print(jtrq_des_, std::cout, "jtrq_des_");

  // // Integrate Joint Velocities and Positions
  // des_jacc_ = qddot_cmd_;
  // if (joint_integrator_->isInitialized()) {
  //   joint_integrator_->integrate(
  //       des_jacc_, sp_->qdot.segment(Magneto::n_vdof, Magneto::n_adof),
  //       sp_->q.segment(Magneto::n_vdof, Magneto::n_adof), des_jvel_, des_jpos_);
  // } else {
  //   des_jpos_ = sp_->q.segment(Magneto::n_vdof, Magneto::n_adof);
  //   des_jvel_ = sp_->qdot.segment(Magneto::n_vdof, Magneto::n_adof);
  // }

  for (int i(0); i < Magneto::n_adof; ++i) {
      ((MagnetoCommand*)_cmd)->jtrq[i] = jtrq_des_[i];
      ((MagnetoCommand*)_cmd)->q[i] = jpos_des_[i];
      ((MagnetoCommand*)_cmd)->qdot[i] = jvel_des_[i];
  }

  // ((MagnetoCommand*)_cmd)->alfoot_magnetism_on = taf_container_->alfoot_magnetism_on_;
  // ((MagnetoCommand*)_cmd)->blfoot_magnetism_on = taf_container_->blfoot_magnetism_on_;
  // ((MagnetoCommand*)_cmd)->arfoot_magnetism_on = taf_container_->arfoot_magnetism_on_;
  // ((MagnetoCommand*)_cmd)->brfoot_magnetism_on = taf_container_->brfoot_magnetism_on_;
  
  ((MagnetoCommand*)_cmd)->b_magnetism_map = taf_container_->b_magnetism_map_;


  // _PostProcessing_Command(); // unset task and contact

  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->q, std::cout, "q");
  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->qdot, std::cout, "qdot");
}


void MagnetoMainController::firstVisit() { 
  
}

void MagnetoMainController::ctrlInitialization(const YAML::Node& node) {
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

  tau_min_ =
      // sp_->getActiveJointValue(robot_->GetTorqueLowerLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, -torque_limit_); //-2500.
  tau_max_ =
      // sp_->getActiveJointValue(robot_->GetTorqueUpperLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, torque_limit_); //-2500.
  wbmc_->setTorqueLimits(tau_min_, tau_max_);

  // Set Joint Integrator Parameters
}
