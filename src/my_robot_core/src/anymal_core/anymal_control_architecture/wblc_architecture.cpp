#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>

ANYmalWblcControlArchitecture::ANYmalWblcControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  my_utils::pretty_constructor(1, "ANYmal Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM "config/ANYmal/ARCHITECTURE/CLIMBING_PARAMS.yaml");

  sp_ = ANYmalStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  ws_container_ = new ANYmalWbcSpecContainer(robot_);
  rg_container_ = new ANYmalReferenceGeneratorContainer(ws_container_, robot_);

  _ReadParameters();  

  wbc_controller = new ANYmalWBC(ws_container_, robot_);

  states_sequence_ = new StateSequence<MotionCommand>();

  // Initialize states: add all states to the state machine map
  state_machines_[ANYMAL_STATES::BALANCE] =
      new FullSupport(ANYMAL_STATES::BALANCE, rg_container_);
  state_machines_[ANYMAL_STATES::SWING_START_TRANS] =
      new Transition(ANYMAL_STATES::SWING_START_TRANS, rg_container_, 0);
  state_machines_[ANYMAL_STATES::SWING] =
      new Swing(ANYMAL_STATES::SWING, rg_container_);
  state_machines_[ANYMAL_STATES::SWING_END_TRANS] =
      new Transition(ANYMAL_STATES::SWING_END_TRANS, rg_container_, 1);
  // Set Starting State
  state_ = ANYMAL_STATES::BALANCE;
  prev_state_ = state_;
  b_state_first_visit_ = true;
  sp_->curr_state = state_;

  _InitializeParameters();
}

ANYmalWblcControlArchitecture::~ANYmalWblcControlArchitecture() {
  delete ws_container_;
  delete rg_container_;
  delete wbc_controller;

  // Delete the state machines
  // delete state_machines_[ANYMAL_STATES::INITIALIZE];
  // delete state_machines_[ANYMAL_STATES::STAND];
  delete state_machines_[ANYMAL_STATES::BALANCE];
  delete state_machines_[ANYMAL_STATES::SWING_START_TRANS];
  delete state_machines_[ANYMAL_STATES::SWING];
  delete state_machines_[ANYMAL_STATES::SWING_END_TRANS];

}

void ANYmalWblcControlArchitecture::ControlArchitectureInitialization() {}

void ANYmalWblcControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

  // static bool b_integrator_init = true;
  // if ( (prev_state_ == ANYMAL_STATES::STAND || state_ == ANYMAL_STATES::BALANCE) 
  //       && b_integrator_init) {
  //   wbc_controller->initializeJointIntegrator();
  //   b_integrator_init = false;
  // }

  // Update State Machine
  state_machines_[state_]->oneStep();

  // Get Wholebody control commands
  if (state_ == ANYMAL_STATES::INITIALIZE) {
    getIVDCommand(_command);
  } else {
    wbc_controller->getCommand(_command);
  }

  // Save Data
  saveData();

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    states_sequence_->getNextState(state_, user_cmd_);
    
    sp_->curr_state = state_;
    sp_->curr_motion_command = (MotionCommand)user_cmd_;

    b_state_first_visit_ = true;
  }
  sp_->num_state = states_sequence_->getNumStates();
};

void ANYmalWblcControlArchitecture::addState(StateIdentifier _state_id, 
                                            void* _user_state_command) {
  states_sequence_->addState( _state_id, 
                            *((MotionCommand*)_user_state_command) );
}

///////////////////////////////////////////////////////////////////////

void ANYmalWblcControlArchitecture::getIVDCommand(void* _cmd) {
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd des_jpos = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd des_jvel = Eigen::VectorXd::Zero(ANYmal::n_adof);

  Eigen::MatrixXd A = robot_->getMassMatrix();
  Eigen::VectorXd grav = robot_->getGravity();
  Eigen::VectorXd cori = robot_->getCoriolis();

  Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd q_des = sp_->getActiveJointValue();

  ws_container_->joint_task_->updateTask(q_des, qdot_des, qddot_des);
  ws_container_->joint_task_->getCommand(qddot_des);
  qddot_des = sp_->getFullJointValue(qddot_des);

  des_jpos = sp_->getActiveJointValue();
  des_jvel = sp_->getActiveJointValue(sp_->qdot);

  tau_cmd = sp_->getActiveJointValue(A * qddot_des + cori + grav);

  for (int i(0); i < ANYmal::n_adof; ++i) {
    ((ANYmalCommand*)_cmd)->jtrq[i] = tau_cmd[i];
    ((ANYmalCommand*)_cmd)->q[i] = des_jpos[i];
    ((ANYmalCommand*)_cmd)->qdot[i] = des_jvel[i];
  }
}

void ANYmalWblcControlArchitecture::_ReadParameters() {
  try {
    my_utils::readParameter(cfg_["controller_params"], 
                          "wbc_type", controller_type_);  

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

void ANYmalWblcControlArchitecture::_InitializeParameters() {
  // weigt parameter initialization
  ws_container_->weightParamInitialization(cfg_["qp_weights_params"]);
  ws_container_->contactParamInitialization(cfg_["contact_params"]);

  // Controller initialization
  wbc_controller->ctrlInitialization(cfg_["controller_params"]);

  // States Initialization:
  state_machines_[ANYMAL_STATES::SWING]
                  ->initialization(cfg_["state_swing_params"]);
  state_machines_[ANYMAL_STATES::SWING_START_TRANS]
                  ->initialization(cfg_["transition_params"]);
  state_machines_[ANYMAL_STATES::SWING_END_TRANS]
                  ->initialization(cfg_["transition_params"]);

}

void ANYmalWblcControlArchitecture::saveData() {
  // 
  //  sp_->
}
