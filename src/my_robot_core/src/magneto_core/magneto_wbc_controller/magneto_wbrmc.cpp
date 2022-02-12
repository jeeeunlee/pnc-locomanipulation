#include <my_robot_core/magneto_core/magneto_wbc_controller/magneto_wbrmc.hpp>

MagnetoWBRMC::MagnetoWBRMC(
    MagnetoWbcSpecContainer* _ws_container, RobotSystem* _robot)
    :MagnetoWBMC(_ws_container, _robot) {
  my_utils::pretty_constructor(2, "Magneto Residual Controller");
  // Initialize Flag
  // b_first_visit_ = true;

  // // Initialize Pointer to the Task and Force Container
  // ws_container_ = _ws_container;
  // robot_ = _robot;

  // // Initialize State Provider
  // sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // // Initialize Actuator selection list
  
  // act_list_.resize(Magneto::n_dof, true);
  // for (int i(0); i < Magneto::n_vdof; ++i) 
  //     act_list_[Magneto::idx_vdof[i]] = false;

  // // Initialize WBC
  // kin_wbc_ = new KinWBC(act_list_);
  wbrmc_ = new WBRMRC(act_list_);
  wbrmc_param_ = new WBRMRC_ExtraData();

  // tau_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  // qddot_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);

  // // Initialize desired pos, vel, acc containers
  jpos_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  jvel_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  jacc_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
}

MagnetoWBRMC::~MagnetoWBRMC() {
  delete wbrmc_;
}

void MagnetoWBRMC::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Clear out local pointers
  task_list_.clear();
  contact_list_.clear();

  // Grab Variables from the container.
  wbrmc_param_->W_qddot_ = ws_container_->W_qddot_;
  wbrmc_param_->W_xddot_ = ws_container_->W_xddot_;
  wbrmc_param_->W_rf_ = ws_container_->W_rf_;

  wbrmc_param_->F_magnetic_ = - ws_container_->F_magnetic_;
  wbrmc_param_->F_residual_ = - ws_container_->F_residual_;
  wbrmc_param_->J_residual_ = ws_container_->J_residual_;
  
  //0112 my_utils::saveVector(wbrmc_param_->F_residual_,"wbc_F_residual");
  Eigen::VectorXd wrf = wbrmc_param_->W_rf_.head(6);
  //0112 my_utils::saveVector(wrf,"W_rf_");
  // my_utils::pretty_print(wbrmc_param_->W_qddot_, std::cout, "W_qddot_");
  // my_utils::pretty_print(wbrmc_param_->W_xddot_, std::cout, "W_xddot_");
  // my_utils::pretty_print(wbrmc_param_->W_rf_, std::cout, "W_rf_");
  // my_utils::pretty_print(wbrmc_param_->F_magnetic_, std::cout, "F_magnetic_");

  // Update task and contact list pointers from container object
  for (int i = 0; i < ws_container_->task_list_.size(); i++) {
    task_list_.push_back(ws_container_->task_list_[i]);
  }
  for (int i = 0; i < ws_container_->contact_list_.size(); i++) {
    contact_list_.push_back(ws_container_->contact_list_[i]);
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

void MagnetoWBRMC::getCommand(void* _cmd) {

  // state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // _set_magnetic_force(_cmd);    

  // grab & update task_list and contact_list & QP weights
  _PreProcessing_Command();

  kin_wbc_->FindFullConfiguration(sp_->q, task_list_, contact_list_, 
                                    jpos_des_, jvel_des_, jacc_des_); 
    
  Eigen::VectorXd jacc_des_cmd = jacc_des_;
  // for(int i(0); i<Magneto::n_adof; ++i) {
  //   jacc_des_cmd[Magneto::idx_adof[i]] +=
  //         Kp_[i]*(jpos_des_[Magneto::idx_adof[i]] - sp_->q[Magneto::idx_adof[i]])
  //         + Kd_[i]*(jvel_des_[Magneto::idx_adof[i]] - sp_->qdot[Magneto::idx_adof[i]]);
  // }
  // for(int i(6); i<Magneto::n_vdof; ++i)  {
  //   jacc_des_cmd[Magneto::idx_vdof[i]] = 0.0;
  // }
  for(int i(0); i<Magneto::n_dof; ++i) {
    jacc_des_cmd[i] +=
          Kp_[0]*(jpos_des_[i] - sp_->q[i])
          + Kd_[0]*(jvel_des_[i] - sp_->qdot[i]);
  }

  // my_utils::pretty_print(jpos_des_, std::cout, "jpos_des_");
  // my_utils::pretty_print(jvel_des_, std::cout, "jvel_des_");
  // my_utils::pretty_print(jacc_des_, std::cout, "jacc_des_");
  // my_utils::pretty_print(jacc_des_cmd, std::cout, "jacc_des_cmd");
  // exit(0);
  //0112 my_utils::saveVector(jpos_des_,"jpos_des_");
  //0112 my_utils::saveVector(jvel_des_,"jvel_des_");
  
                                
  // wbmc
  wbrmc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  wbrmc_->makeTorqueGivenRef(jacc_des_cmd, contact_list_, jtrq_des_, wbrmc_param_);

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

  
  ((MagnetoCommand*)_cmd)->jtrq = jtrq_des_;
  ((MagnetoCommand*)_cmd)->q = sp_->getActiveJointValue(jpos_des_);
  ((MagnetoCommand*)_cmd)->qdot = sp_->getActiveJointValue(jvel_des_); 
  ws_container_->update_magnetism_map( ((MagnetoCommand*)_cmd)->b_magnetism_map );
  

  // _PostProcessing_Command(); // unset task and contact

  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // my_utils::pretty_print(jpos_des_, std::cout, "jpos_des_");
  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->q, std::cout, "q");
  // my_utils::pretty_print(((MagnetoCommand*)_cmd)->qdot, std::cout, "qdot");
}


void MagnetoWBRMC::firstVisit() { 
  
}

void MagnetoWBRMC::ctrlInitialization(const YAML::Node& node) {
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
  wbrmc_->setTorqueLimits(tau_min, tau_max);

  // Set Joint Integrator Parameters
}
