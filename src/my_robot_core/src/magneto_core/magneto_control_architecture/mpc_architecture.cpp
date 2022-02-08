
#include <my_robot_core/magneto_core/magneto_control_architecture/magneto_control_architecture_set.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>


MagnetoMpcControlArchitecture::MagnetoMpcControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  my_utils::pretty_constructor(1, "Magneto Mpc Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM "config/Magneto/ARCHITECTURE/CLIMBING_PARAMS.yaml");

  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  ws_container_ = new MagnetoWbcSpecContainer(robot_);
  rg_container_ = new MagnetoReferenceGeneratorContainer(robot_);

  _ReadParameters();
  
  switch(controller_type_) {
    case(CONTROLLER_TYPES::WBMC):
      wbc_controller = new MagnetoWBMC(ws_container_, robot_);
      break;
    case(CONTROLLER_TYPES::WBRMC):
    default:
      wbc_controller = new MagnetoWBRMC(ws_container_, robot_);
      break;
  }

  slip_ob_ = new SlipObserver(ws_container_, robot_);
  slip_ob_data_ = new SlipObserverData();

  states_sequence_ = new StateSequence<SimMotionCommand>();
  user_cmd_ = SimMotionCommand();

  // Initialize states: add all states to the state machine map
  state_machines_[MAGNETO_STATES::BALANCE] =
      new FullSupport(MAGNETO_STATES::BALANCE, robot_, ws_container_, rg_container_);
  state_machines_[MAGNETO_STATES::SWING_START_TRANS] =
      new Transition(MAGNETO_STATES::SWING_START_TRANS, robot_, ws_container_, rg_container_, 0);
  state_machines_[MAGNETO_STATES::SWING] =
      new Swing(MAGNETO_STATES::SWING, robot_, ws_container_, rg_container_);
  state_machines_[MAGNETO_STATES::SWING_END_TRANS] =
      new Transition(MAGNETO_STATES::SWING_END_TRANS, robot_, ws_container_, rg_container_, 1);
  
  // Set Starting State
  state_ = MAGNETO_STATES::BALANCE;
  prev_state_ = state_;
  b_state_first_visit_ = true;
  sp_->curr_state = state_;
  

  _InitializeParameters();
}

MagnetoMpcControlArchitecture::~MagnetoMpcControlArchitecture() {
  delete ws_container_;
  delete rg_container_;
  delete wbc_controller;

  delete slip_ob_;
  delete slip_ob_data_;

  // Delete the state machines
  // delete state_machines_[MAGNETO_STATES::INITIALIZE];
  // delete state_machines_[MAGNETO_STATES::STAND];
  delete state_machines_[MAGNETO_STATES::BALANCE];
  delete state_machines_[MAGNETO_STATES::SWING_START_TRANS];
  delete state_machines_[MAGNETO_STATES::SWING];
  delete state_machines_[MAGNETO_STATES::SWING_END_TRANS];

}

void MagnetoMpcControlArchitecture::ControlArchitectureInitialization() {}

void MagnetoMpcControlArchitecture::getCommand(void* _command) {

  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }
  // Update State Machine
  state_machines_[state_]->oneStep();

  // Get Wholebody control commands
  if (state_ == MAGNETO_STATES::INITIALIZE) {
    getIVDCommand(_command);
  } else {
    wbc_controller->getCommand(_command);
  }

  // --------------------------------------------------
  // State Estimator / Observer  
  static double print_time = 0.0;
  if( (sp_->curr_time-print_time) > 0.01 ){
    slip_ob_->checkVelocity(MagnetoFoot::AL);
    slip_ob_->checkVelocity(MagnetoFoot::BL);
    slip_ob_->checkVelocity(MagnetoFoot::AR);
    slip_ob_->checkVelocity(MagnetoFoot::BR);
    print_time = sp_->curr_time;
  }
  Eigen::VectorXd tau = sp_->getFullJointValue(((MagnetoCommand*)_command)->jtrq);
  
  slip_ob_->checkForce(tau);
  // --------------------------------------------------

  // Save Data
  saveData();

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    states_sequence_->getNextState(state_, user_cmd_);
    
    sp_->curr_state = state_;
    sp_->curr_motion_command = (MotionCommand)user_cmd_;
    sp_->curr_simulation_command = (SimulationCommand)user_cmd_;

    b_state_first_visit_ = true;
  }
  sp_->num_state = states_sequence_->getNumStates();
};

void MagnetoMpcControlArchitecture::addState(void* _user_state_command) {
  std::cout<<" MagnetoMpcControlArchitecture::addState" << std::endl;
  MagnetoUserStateCommand* state_pair = ((MagnetoUserStateCommand*)_user_state_command); 
  std::cout<<" MagnetoMpcControlArchitecture::state_pair done" << std::endl;
  states_sequence_->addState( state_pair->state_id, 
                            state_pair->user_cmd);
  std::cout<<" MagnetoMpcControlArchitecture::states_sequence_->addState done" << std::endl;
}

///////////////////////////////////////////////////////////////////////


void MagnetoMpcControlArchitecture::getIVDCommand(void* _cmd) {
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd des_jpos = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd des_jvel = Eigen::VectorXd::Zero(Magneto::n_adof);

  Eigen::MatrixXd A = robot_->getMassMatrix();
  Eigen::VectorXd grav = robot_->getGravity();
  Eigen::VectorXd cori = robot_->getCoriolis();

  Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd q_des = sp_->getActiveJointValue();

  // ws_container_->joint_task_->updateJacobians();
  // ws_container_->joint_task_->computeCommands();
  // ws_container_->joint_task_->getCommand(xddot_des);

  ws_container_->joint_task_->updateTask(q_des, qdot_des, qddot_des);
  ws_container_->joint_task_->getCommand(qddot_des);
  qddot_des = sp_->getFullJointValue(qddot_des);

  des_jpos = sp_->getActiveJointValue();
  des_jvel = sp_->getActiveJointValue(sp_->qdot);

  tau_cmd = sp_->getActiveJointValue(A * qddot_des + cori + grav);

  for (int i(0); i < Magneto::n_adof; ++i) {
    ((MagnetoCommand*)_cmd)->jtrq[i] = tau_cmd[i];
    ((MagnetoCommand*)_cmd)->q[i] = des_jpos[i];
    ((MagnetoCommand*)_cmd)->qdot[i] = des_jvel[i];
  }
}

void MagnetoMpcControlArchitecture::_ReadParameters() {
  try {
    my_utils::readParameter(cfg_["controller_params"], 
                          "wbc_type", controller_type_);  
    // magnetism
    my_utils::readParameter(cfg_["magnetism_params"], 
                          "magnetic_force", ws_container_->magnetic_force_);  
    my_utils::readParameter(cfg_["magnetism_params"], 
                          "residual_ratio", ws_container_->residual_ratio_);  

    my_utils::readParameter(cfg_["contact_params"], 
                          "friction", ws_container_->friction_coeff_);  

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  ws_container_->setContactFriction();
}

void MagnetoMpcControlArchitecture::_InitializeParameters() {
  // weigt parameter initialization
  ws_container_->paramInitialization(cfg_["qp_weights_params"]);

  // Controller initialization
  wbc_controller->ctrlInitialization(cfg_["controller_params"]);

  // States Initialization:
  state_machines_[MAGNETO_STATES::SWING]
                  ->initialization(cfg_["state_swing_params"]);
  state_machines_[MAGNETO_STATES::SWING_START_TRANS]
                  ->initialization(cfg_["transition_params"]);
  state_machines_[MAGNETO_STATES::SWING_END_TRANS]
                  ->initialization(cfg_["transition_params"]);

}

void MagnetoMpcControlArchitecture::saveData() {
  // 
  //  sp_->
}
