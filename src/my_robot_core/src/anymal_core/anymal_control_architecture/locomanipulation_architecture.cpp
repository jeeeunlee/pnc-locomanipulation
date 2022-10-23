
#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>


ANYmalLocoManipulationControlArchitecture::ANYmalLocoManipulationControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  my_utils::pretty_constructor(1, "ANYmal Loco Manipulation Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM "config/ANYmal/ARCHITECTURE/LOCOMANIPULATION_PARAMS.yaml");

  sp_ = ANYmalStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  ws_container_ = new ANYmalWbcSpecContainer(robot_);
  rg_container_ = new ANYmalReferenceGeneratorContainer(ws_container_, robot_);

  _ReadParameters();

  wbc_controller = new ANYmalWBC(ws_container_, robot_);

  prev_state_ = ANYMAL_STATES::INITIALIZE;
  state_ = ANYMAL_STATES::IDLE;

  com_states_sequence_ = new StateSequence<MotionCommand>();
  arm_states_sequence_= new StateSequence<MotionCommand>(); 
  com_state_machine = new CoMStateMachine(rg_container_);
  ee_state_machine = new Manipulation(rg_container_);
  b_com_state_first_visit_ = true;
  b_ee_state_first_visit_ = true;

  for(int foot_idx=0; foot_idx<ANYmal::n_leg; ++foot_idx){
    feet_states_sequence_[foot_idx] = new StateSequence<MotionCommand>();    
    foot_state_machines_[foot_idx][ANYMAL_FOOT_STATES::SUPPORT]
      = new Support(foot_idx, rg_container_);
    foot_state_machines_[foot_idx][ANYMAL_FOOT_STATES::SWING_START_TRANS]
     = new Transition(foot_idx, rg_container_, 0);
    foot_state_machines_[foot_idx][ANYMAL_FOOT_STATES::SWING]
     = new Swing(foot_idx, rg_container_);
    foot_state_machines_[foot_idx][ANYMAL_FOOT_STATES::SWING_END_TRANS]
     = new Transition(foot_idx, rg_container_, 1);
    b_foot_state_first_visit_[foot_idx] = true;
    foot_state_[foot_idx] = ANYMAL_FOOT_STATES::SUPPORT;
  }   

  sp_->feet_curr_state = foot_state_;

  _InitializeParameters();
}

ANYmalLocoManipulationControlArchitecture::~ANYmalLocoManipulationControlArchitecture() {
  delete ws_container_;
  delete rg_container_;
  delete wbc_controller;

  for(auto &foot_state_machine : foot_state_machines_){
    delete foot_state_machine[ANYMAL_FOOT_STATES::SUPPORT];
    delete foot_state_machine[ANYMAL_FOOT_STATES::SWING_START_TRANS];
    delete foot_state_machine[ANYMAL_FOOT_STATES::SWING];
    delete foot_state_machine[ANYMAL_FOOT_STATES::SWING_END_TRANS];
  }

  delete com_state_machine;
  delete ee_state_machine; 
  
}

void ANYmalLocoManipulationControlArchitecture::ControlArchitectureInitialization() {}

void ANYmalLocoManipulationControlArchitecture::getCommand(void* _command) {

  sp_->num_com_state = com_states_sequence_->getNumStates();
  sp_->num_ee_state = arm_states_sequence_->getNumStates();
  for(int foot_idx=0; foot_idx<ANYmal::n_leg; ++foot_idx){  
    sp_->num_feet_state[foot_idx] = feet_states_sequence_[foot_idx]->getNumStates();
  }

  //----------------------------------------------------
  //        SET CONTACT/TASKs
  //----------------------------------------------------
  if(b_com_state_first_visit_){    
    com_state_machine->firstVisit();
    b_com_state_first_visit_= false;
    // ws_container_->check_task_list();
  }
  for(int foot_idx=0; foot_idx<ANYmal::n_leg; ++foot_idx){    
    if (b_foot_state_first_visit_[foot_idx]) {      
      foot_state_machines_[foot_idx][foot_state_[foot_idx]]->firstVisit();
      b_foot_state_first_visit_[foot_idx] = false;
      // ws_container_->check_task_list();
    }
  }
  if(b_ee_state_first_visit_){    
    ee_state_machine->firstVisit();
    b_ee_state_first_visit_= false;
    // ws_container_->check_task_list();
  }

  //----------------------------------------------------
  //        UPDATE TASKs
  //----------------------------------------------------
  com_state_machine->oneStep();
  for(int foot_idx=0; foot_idx<ANYmal::n_leg; ++foot_idx){
    foot_state_machines_[foot_idx][foot_state_[foot_idx]]->oneStep();
  }
  ee_state_machine->oneStep();

  //----------------------------------------------------
  //        SOLVE WBC
  //----------------------------------------------------
  wbc_controller->getCommand(_command);
  // Save Data
  saveData();

  //----------------------------------------------------
  //        CHECK END OF STATES
  //----------------------------------------------------
  if (com_state_machine->endOfState()){    
    if( com_states_sequence_->getNextState(com_state_, user_cmd_) ){
      com_state_machine->lastVisit();         
      sp_->com_motion_command = user_cmd_;
      sp_->com_state = com_state_;
      b_com_state_first_visit_ = true;
    }  
  }
  for(int foot_idx=0; foot_idx<ANYmal::n_leg; ++foot_idx){
    if (foot_state_machines_[foot_idx][foot_state_[foot_idx]]->endOfState()) {
      if(feet_states_sequence_[foot_idx]->getNextState(foot_state_[foot_idx], user_cmd_)){
        // user_cmd_.printMotionInfo();
        foot_state_machines_[foot_idx][foot_state_[foot_idx]]->lastVisit();
        sp_->feet_motion_command[foot_idx] = user_cmd_;
        sp_->feet_curr_state[foot_idx] = foot_state_[foot_idx];
        b_foot_state_first_visit_[foot_idx] = true;
      }
    }
  }
  if(ee_state_machine->endOfState()){
    if(arm_states_sequence_->getNextState(arm_state_, user_cmd_)){
      ee_state_machine->lastVisit();
      sp_->ee_motion_command = user_cmd_;
      sp_->arm_state = arm_state_;
      b_ee_state_first_visit_=true;
    }
  }    
};

void ANYmalLocoManipulationControlArchitecture::setMotionStartTime(){
  std::cout<<"motion_start_time set to "<< sp_->curr_time << std::endl;
  sp_->motion_start_time = sp_->curr_time;
}

void ANYmalLocoManipulationControlArchitecture::addState(
                                StateIdentifier _state_id, 
                                void* _user_state_command, 
                                int state_type) {
  if(state_type == EE_IDX::armEE)
    arm_states_sequence_->addState(_state_id, *((MotionCommand*)_user_state_command) );
  else if(state_type == EE_IDX::CoM)
    com_states_sequence_->addState(_state_id, *((MotionCommand*)_user_state_command) );
  else if(state_type > -1)
    feet_states_sequence_[state_type]->addState(_state_id, *((MotionCommand*)_user_state_command) );
}

///////////////////////////////////////////////////////////////////////

void ANYmalLocoManipulationControlArchitecture::getIVDCommand(void* _cmd) {
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd des_jpos = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd des_jvel = Eigen::VectorXd::Zero(ANYmal::n_adof);

  Eigen::MatrixXd A = robot_->getMassMatrix();
  Eigen::VectorXd grav = robot_->getGravity();
  Eigen::VectorXd cori = robot_->getCoriolis();

  Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(ANYmal::n_adof);
  Eigen::VectorXd q_des = sp_->getActiveJointValue();

  ws_container_->task_container_[ANYMAL_TASK::JOINT_TASK]->updateTask(q_des, qdot_des, qddot_des);
  ws_container_->task_container_[ANYMAL_TASK::JOINT_TASK]->getCommand(qddot_des);
  qddot_des = sp_->getFullJointValue(qddot_des, false);

  des_jpos = sp_->getActiveJointValue();
  des_jvel = sp_->getActiveJointValue(sp_->qdot);

  tau_cmd = sp_->getActiveJointValue(A * qddot_des + cori + grav);

  for (int i(0); i < ANYmal::n_adof; ++i) {
    ((ANYmalCommand*)_cmd)->jtrq[i] = tau_cmd[i];
    ((ANYmalCommand*)_cmd)->q[i] = des_jpos[i];
    ((ANYmalCommand*)_cmd)->qdot[i] = des_jvel[i];
  }
}

void ANYmalLocoManipulationControlArchitecture::_ReadParameters() {
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

void ANYmalLocoManipulationControlArchitecture::_InitializeParameters() {  
  // weigt parameter initialization
  ws_container_->weightParamInitialization(cfg_["qp_weights_params"]);
  ws_container_->contactParamInitialization(cfg_["contact_params"]);

  // Controller initialization
  wbc_controller->ctrlInitialization(cfg_["controller_params"]);

  // States Initialization:
  // state_machines_[ANYMAL_STATES::BALANCE]->initialization(cfg_["**"]);
  // state_machines_[ANYMAL_STATES::MANIPULATION]->initialization(cfg_["**"]);
}

void ANYmalLocoManipulationControlArchitecture::saveData() {
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

  // feet position
  Eigen::VectorXd fpos;  
  for(int i(0); i<ANYmal::n_leg; ++i) {
    filename = ANYmalFoot::Names[i] + "_position";    
    fpos = robot_->getBodyNodeIsometry(
            ANYmalFoot::LinkIdx[i]).translation();
    my_utils::saveVector(fpos, filename);
  }

  //  feet fsm state

  Eigen::VectorXd pos;  
  for(int i(0); i<ANYmal::n_leg; ++i) {
    filename = ANYmalFoot::Names[i] + "_state";    
    state_val = (double) foot_state_[i];
    my_utils::saveValue( state_val, filename );
  }

  


}
