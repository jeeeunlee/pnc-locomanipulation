#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>

MagnetoControlArchitecture::MagnetoControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  my_utils::pretty_constructor(1, "Magneto Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM "config/Magneto/ARCHITECTURE/WALKING_PARAMS.yaml");

  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  taf_container_ = new MagnetoTaskAndForceContainer(robot_);

  _ReadParameters();
  switch(controller_type_) {
    case(CONTROLLER_TYPES::WBMC):
      main_controller_ = new MagnetoMainController(taf_container_, robot_);
      break;
    case(CONTROLLER_TYPES::WBRMC):
    default:
      main_controller_ = new MagnetoResidualController(taf_container_, robot_);
      break;
  }

  // Initialize Trajectory managers
  foot_trajectory_manager_ = new FootPosTrajectoryManager(robot_);                    
  com_trajectory_manager_ = new CoMTrajectoryManager(robot_);
  joint_trajectory_manager_ = new JointTrajectoryManager(robot_);
  base_ori_trajectory_manager_ = new BaseOriTrajectoryManager(robot_);

  // -- TaskWeightTrajectoryManager : class for managing hierarchy with weight
  // -- MaxNormalForceTrajectoryManager : class for managing max normal force
  max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(robot_);
  QPweight_qddot_manager_ = new QPWeightTrajectoryManager(robot_);
  QPweight_xddot_manager_ = new QPWeightTrajectoryManager(robot_);
  QPweight_reactforce_manager_ = new QPWeightTrajectoryManager(robot_);
  weight_residualforce_manager_ = new SingleWeightTrajectoryManager(robot_);

  goal_planner_ = new MagnetoGoalPlanner(robot_);
  trajectory_planner_ = new MagnetoReachabilityPlanner(robot_, taf_container_->friction_coeff_);
  // -- Trajctory Planner ?
  // dcm_trajectory_manager_ = new DCMTrajectoryManager(
  //     dcm_planner_, taf_container_->com_task_, taf_container_->base_ori_task_,
  //     robot_, MagnetoBodyNode::lFootCenter, MagnetoBodyNode::rFootCenter);

  // Initialize states: add all states to the state machine map
  state_machines_[MAGNETO_STATES::BALANCE] =
      new FullSupport(MAGNETO_STATES::BALANCE, this, robot_);
  state_machines_[MAGNETO_STATES::SWING_START_TRANS] =
      new Transition(MAGNETO_STATES::SWING_START_TRANS, this, robot_, 0);
  state_machines_[MAGNETO_STATES::SWING] =
      new Swing(MAGNETO_STATES::SWING, this, robot_);
  state_machines_[MAGNETO_STATES::SWING_END_TRANS] =
      new Transition(MAGNETO_STATES::SWING_END_TRANS, this, robot_, 1);
  // Set Starting State
  state_ = MAGNETO_STATES::BALANCE;
  prev_state_ = state_;
  motion_command_ = MotionCommand();
  b_state_first_visit_ = true;

  _InitializeParameters();
}

MagnetoControlArchitecture::~MagnetoControlArchitecture() {
  delete taf_container_;
  delete main_controller_;

  // Delete the trajectory managers
  delete foot_trajectory_manager_;
  delete com_trajectory_manager_;
  delete joint_trajectory_manager_;
  delete base_ori_trajectory_manager_;

  // -- TaskWeightTrajectoryManager : class for managing hierarchy with weight
  // -- MaxNormalForceTrajectoryManager : class for managing max normal force
  delete max_normal_force_manager_;
  delete QPweight_qddot_manager_;
  delete QPweight_xddot_manager_;
  delete QPweight_reactforce_manager_;

  // Delete the state machines
  // delete state_machines_[MAGNETO_STATES::INITIALIZE];
  // delete state_machines_[MAGNETO_STATES::STAND];
  delete state_machines_[MAGNETO_STATES::BALANCE];
  delete state_machines_[MAGNETO_STATES::SWING_START_TRANS];
  delete state_machines_[MAGNETO_STATES::SWING];
  delete state_machines_[MAGNETO_STATES::SWING_END_TRANS];
}

void MagnetoControlArchitecture::ControlArchitectureInitialization() {}

void MagnetoControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

  // static bool b_integrator_init = true;
  // if ((prev_state_ == MAGNETO_STATES::STAND || state_ == MAGNETO_STATES::BALANCE)
  // &&
  // b_integrator_init) {
  // std::cout << "[Joint Integrator] Start" << std::endl;
  // main_controller_->initializeJointIntegrator();
  // b_integrator_init = false;
  //}

  // Update State Machine
  state_machines_[state_]->oneStep();
  // Get Wholebody control commands
  if (state_ == MAGNETO_STATES::INITIALIZE) {
    getIVDCommand(_command);
  } else {
    main_controller_->getCommand(_command);
  }
  // Smoothing trq for initial state
  smoothing_torque(_command);
  // Save Data
  saveData();

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    // state_ = state_machines_[state_]->getNextState();
    get_next_state_pair(state_, motion_command_);
    b_state_first_visit_ = true;
  }
};



//////////////////////////////////////////////////////////////////


int MagnetoControlArchitecture::get_num_states() {
  return states_sequence_.size();
}

MotionCommand MagnetoControlArchitecture::get_next_motion_command() {
  MotionCommand cmd;
  states_sequence_mtx_.lock();
  if(states_sequence_.empty()) {
    cmd = MotionCommand();
  }    
  else {
    StatePair next_state_pair = states_sequence_.front();
    cmd = next_state_pair.second;
    //states_sequence_.pop_front();    
  }
  states_sequence_mtx_.unlock();
  return cmd;
}

MotionCommand MagnetoControlArchitecture::get_motion_command() {
  return motion_command_;
}

void MagnetoControlArchitecture::get_next_state_pair(StateIdentifier &_next_state, 
                                              MotionCommand &_next_motion_command) {
  // DRACO & VALKIYRIE VERSION
  // return state_machines_[state_]->getNextState(); 
  states_sequence_mtx_.lock();
  if(states_sequence_.empty()) {
     _next_state = MAGNETO_STATES::BALANCE;
     _next_motion_command = MotionCommand();
     std::cout<<"states_sequence_ is empty!!" << std::endl;
  }    
  else {
    StatePair next_state_pair = states_sequence_.front();
    _next_state = next_state_pair.first;
    _next_motion_command = next_state_pair.second;
    states_sequence_.pop_front();       
  }
  states_sequence_mtx_.unlock();
  // std::cout<<"get_next_state = " << _next_state << std::endl;
}

void MagnetoControlArchitecture::add_next_state(int _state, const MotionCommand &_motion_command) {
  StatePair next_state_pair = std::make_pair(_state, _motion_command);
  add_next_state(next_state_pair);  
} 

void MagnetoControlArchitecture::add_next_state(StatePair _state_pair) {
  states_sequence_mtx_.lock();
  states_sequence_.push_back(_state_pair);
  states_sequence_mtx_.unlock();
}

///////////////////////////////////////////////////////////////////////
void MagnetoControlArchitecture::smoothing_torque(void* _cmd) {
  // if (state_ == MAGNETO_STATES::INITIALIZE) {
  //   double rat = ((Initialize*)state_machines_[state_])->progression_variable();
  //   for (int i = 0; i < Magneto::n_adof; ++i) {
  //     ((MagnetoCommand*)_cmd)->jtrq[i] =
  //         my_utils::smoothing(0, ((MagnetoCommand*)_cmd)->jtrq[i], rat);
  //     sp_->prev_trq_cmd[i] = ((MagnetoCommand*)_cmd)->jtrq[i];
  //   }
  // }
}

void MagnetoControlArchitecture::getIVDCommand(void* _cmd) {
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd des_jpos = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd des_jvel = Eigen::VectorXd::Zero(Magneto::n_adof);

  Eigen::MatrixXd A = robot_->getMassMatrix();
  Eigen::VectorXd grav = robot_->getGravity();
  Eigen::VectorXd cori = robot_->getCoriolis();

  Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(Magneto::n_adof);
  Eigen::VectorXd q_des = sp_->getActiveJointValue();

  // taf_container_->joint_task_->updateJacobians();
  // taf_container_->joint_task_->computeCommands();
  // taf_container_->joint_task_->getCommand(xddot_des);

  taf_container_->joint_task_->updateTask(q_des, qdot_des, qddot_des);
  taf_container_->joint_task_->getCommand(qddot_des);
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

void MagnetoControlArchitecture::_ReadParameters() {
  try {
    my_utils::readParameter(cfg_["controller_params"], 
                          "wbc_type", controller_type_);  
    // magnetism
    my_utils::readParameter(cfg_["magnetism_params"], 
                          "magnetic_force", taf_container_->magnetic_force_);  
    my_utils::readParameter(cfg_["magnetism_params"], 
                          "residual_ratio", taf_container_->residual_ratio_);  

    my_utils::readParameter(cfg_["contact_params"], 
                          "friction", taf_container_->friction_coeff_);  

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  taf_container_->setContactFriction();
}

void MagnetoControlArchitecture::_InitializeParameters() {
  // weigt parameter initialization
  taf_container_->paramInitialization(cfg_["qp_weights_params"]);

  // Controller initialization
  main_controller_->ctrlInitialization(cfg_["controller_params"]);
  main_controller_->getTorqueLimit(tau_min_, tau_max_);
  trajectory_planner_->initTorqueLimit(tau_min_, tau_max_); // todo: cfg_


  // States Initialization:
  state_machines_[MAGNETO_STATES::SWING]
                  ->initialization(cfg_["state_swing_params"]);
  state_machines_[MAGNETO_STATES::SWING_START_TRANS]
                  ->initialization(cfg_["transition_params"]);
  state_machines_[MAGNETO_STATES::SWING_END_TRANS]
                  ->initialization(cfg_["transition_params"]);
}

void MagnetoControlArchitecture::saveData() {
  // 
  //  sp_->
}
