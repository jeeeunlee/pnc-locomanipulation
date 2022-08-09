
#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>


ANYmalManipulationControlArchitecture::ANYmalManipulationControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  my_utils::pretty_constructor(1, "ANYmal Manipulation Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM "config/ANYmal/ARCHITECTURE/MANIPULATION_PARAMS.yaml");

  sp_ = ANYmalStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  ws_container_ = new ANYmalWbcSpecContainer(robot_);
  rg_container_ = new ANYmalReferenceGeneratorContainer(ws_container_, robot_);

  _ReadParameters();

  wbc_controller = new ANYmalWBC(ws_container_, robot_);

  states_sequence_ = new StateSequence<ManipulationCommand>();
  user_cmd_ = ManipulationCommand();

  // Initialize states: add all states to the state machine map

  state_machines_[ANYMAL_STATES::BALANCE] =
      new FullSupport(ANYMAL_STATES::BALANCE, rg_container_);
  state_machines_[ANYMAL_STATES::MANIPULATION] = 
      new Manipulation(ANYMAL_STATES::MANIPULATION, rg_container_);

  // Set Starting State
  state_ = ANYMAL_STATES::BALANCE;
  prev_state_ = ANYMAL_STATES::INITIALIZE;
  b_state_first_visit_ = true;
  b_env_param_updated_ = false;
  sp_->curr_state = state_;
  
  _InitializeParameters();
}

ANYmalManipulationControlArchitecture::~ANYmalManipulationControlArchitecture() {
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

void ANYmalManipulationControlArchitecture::ControlArchitectureInitialization() {}

void ANYmalManipulationControlArchitecture::getCommand(void* _command) {

  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

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
    // sp_->curr_motion_command = MotionCommand(user_cmd_.ee_motion_data,
    //                                           user_cmd_.motion_period  ); 

    if(user_cmd_.get_ee_idx() < 0 ) {
      POSE_DATA com_motion_data;
      user_cmd_.get_ee_motion(com_motion_data);
      double motion_period = user_cmd_.get_motion_period();

      sp_->curr_motion_command = MotionCommand(com_motion_data, motion_period);
      sp_->curr_manipulation_command = user_cmd_;
    }else{
      sp_->curr_motion_command = MotionCommand();
      sp_->curr_manipulation_command = user_cmd_;
    }


    b_state_first_visit_ = true;

    // if(states_sequence_->getNumStates()==0)
    //   exit(0);
  }
  sp_->num_state = states_sequence_->getNumStates();
};

void ANYmalManipulationControlArchitecture::addState(
  StateIdentifier _state_id, void* _user_state_command) {
  states_sequence_->addState( 
    _state_id, *((ManipulationCommand*)_user_state_command) );
}

///////////////////////////////////////////////////////////////////////

void ANYmalManipulationControlArchitecture::getIVDCommand(void* _cmd) {
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

void ANYmalManipulationControlArchitecture::_ReadParameters() {
  try {
    // my_utils::readParameter(cfg_["controller_params"], 
    //                       "wbc_type", controller_type_);  

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

void ANYmalManipulationControlArchitecture::_InitializeParameters() {  
  // weigt parameter initialization
  ws_container_->weightParamInitialization(cfg_["qp_weights_params"]);
  ws_container_->contactParamInitialization(cfg_["contact_params"]);

  // Controller initialization
  wbc_controller->ctrlInitialization(cfg_["controller_params"]);

  // States Initialization:
  // state_machines_[ANYMAL_STATES::BALANCE]->initialization(cfg_["**"]);
  // state_machines_[ANYMAL_STATES::MANIPULATION]->initialization(cfg_["**"]);
}

void ANYmalManipulationControlArchitecture::saveData() {
  // 
  //  fsm state
  double state_val = (double) state_;
  my_utils::saveValue( state_val, "fsm_state" );


  // weights
  std::string filename;
  Eigen::VectorXd W_tmp;
  for(int i(0); i<ANYmal::n_leg; ++i) {
    filename = ANYmalFoot::Names[i] + "_Wrf";    
    W_tmp = ws_container_->feet_weights_[i]->getWrf();
    my_utils::saveVector(W_tmp, filename);

  }


}
