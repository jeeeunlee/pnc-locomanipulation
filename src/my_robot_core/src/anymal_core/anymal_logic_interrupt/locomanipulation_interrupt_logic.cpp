#include <my_robot_core/anymal_core/anymal_logic_interrupt/locomanipulation_interrupt_logic.hpp>
#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>

LocoManipulationInterruptLogic::LocoManipulationInterruptLogic(
        ControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "ANYmal Loco-Manipulation Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = ANYmalStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize motion commands
  script_user_cmd_map_.clear();
}

LocoManipulationInterruptLogic::~LocoManipulationInterruptLogic() {}

// Process Interrupts here
void LocoManipulationInterruptLogic::processInterrupts() {   
  if(b_button_pressed) {
    // std::cout << "[Walking Interrupt Logic] button pressed : " << pressed_button << std::endl;
    switch(pressed_button){
      case 's':
        std::cout << "[LocoManipulation Interrupt Logic] button S pressed" << std::endl;        
        std::cout << "---------     SCRIPT MOTION      ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::IDLE) {
          std::cout << "------------------------------------------" << std::endl;
          // set stateMachine sequences
          ctrl_arch_->setMotionStartTime();
          scriptMotionToCommandSet();
          for(auto &it : script_user_cmd_arm_) 
            addStateCommand(it.first, it.second, EE_IDX::armEE);         
          for(auto &it : script_user_cmd_CoM_) 
            addStateCommand(it.first, it.second, EE_IDX::CoM);
          for(int foot_idx(0);foot_idx<ANYmal::n_leg;++foot_idx){
            for(auto &it : script_user_cmd_feet_[foot_idx])
              addStateCommand(it.first, it.second, foot_idx);
          }       
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
}

void LocoManipulationInterruptLogic::addStateCommand(int _state_id, 
                                          const MotionCommand& _mc, 
                                          int state_type){
  MotionCommand* user_state_cmd = new MotionCommand(_mc);
  ctrl_arch_->addState(_state_id, user_state_cmd, state_type);
}


void LocoManipulationInterruptLogic::scriptMotionToCommandSet(){
  double script_time_max=0.0;
  for (auto const& map_it : script_user_cmd_map_){
    if( map_it.second.size()>0 ){
      script_time_max = script_time_max > map_it.second.back().periods[1] ? 
                        script_time_max:map_it.second.back().periods[1];
    }
  }
  // arm motion  
  scriptMotionToEECommandSet(EE_IDX::armEE, script_time_max, script_user_cmd_arm_);
  // com motion
  scriptMotionToEECommandSet(EE_IDX::CoM, script_time_max, script_user_cmd_CoM_);  
  // foot motion
  for(int idx(0);idx<ANYmal::n_leg;++idx)
    scriptMotionToFeetCommandSet(idx, 
                                script_time_max, 
                                script_user_cmd_feet_[idx]);
}

void LocoManipulationInterruptLogic::scriptMotionToEECommandSet(int ee_idx, 
                  double script_time_max,
                  std::vector<std::pair<int, MotionCommand>> &_script_cmds) {
  double t_tmp(0.);
  MotionCommand mc_zero;
  mc_zero.periods = Eigen::VectorXd::Zero(2);
  mc_zero.dpose.pos = Eigen::VectorXd::Zero(3);
  mc_zero.dpose.ori = Eigen::Quaternion<double>::Identity();
  mc_zero.dpose.is_baseframe = true;

  std::vector<MotionCommand> motion_commands = script_user_cmd_map_[ee_idx];
  if(motion_commands.empty()){    
    mc_zero.periods << 0.0, script_time_max;
    _script_cmds.push_back(std::make_pair(ANYMAL_COM_STATES::IDLE, mc_zero));
  }else{
    t_tmp=0.;
    for( auto &mc : motion_commands ){
      if( t_tmp < mc.periods[0] ){
        mc_zero.periods << t_tmp, mc.periods[0];
        _script_cmds.push_back(std::make_pair(ANYMAL_COM_STATES::IDLE, mc_zero));
      }
      _script_cmds.push_back(std::make_pair(ANYMAL_COM_STATES::BALANCE, mc));
      t_tmp = mc.periods[1];
    }
  }

  if(t_tmp < script_time_max){
    mc_zero.periods << t_tmp, script_time_max;
    _script_cmds.push_back(std::make_pair(ANYMAL_COM_STATES::IDLE, mc_zero));
  }
}

void LocoManipulationInterruptLogic::scriptMotionToFeetCommandSet(int ee_idx,
                  double script_time_max,
                  std::vector<std::pair<int, MotionCommand>> &_script_cmds) {
  double t_tmp(0.0);
  MotionCommand mc_zero;
  mc_zero.periods = Eigen::VectorXd::Zero(2);
  mc_zero.dpose.pos = Eigen::VectorXd::Zero(3);
  mc_zero.dpose.ori = Eigen::Quaternion<double>::Identity();
  mc_zero.dpose.is_baseframe = true;

  std::vector<MotionCommand> motion_commands = script_user_cmd_map_[ee_idx];
  _script_cmds.clear();

  double transition_period = 0.05;
  if(motion_commands.empty()){    
    mc_zero.periods << 0.0, script_time_max;
    _script_cmds.push_back(std::make_pair(ANYMAL_FOOT_STATES::SUPPORT, mc_zero));
  }else{
    t_tmp=0.0;
    for( auto &mc : motion_commands ){
      // mc.printMotionInfo();
      if( t_tmp < mc.periods[0] ){
        mc_zero.periods << t_tmp, mc.periods[0];
        _script_cmds.push_back(std::make_pair(ANYMAL_FOOT_STATES::SUPPORT, mc_zero));
      }
      double trans_start = mc.periods[0];
      double swing_start = mc.periods[0]+transition_period;
      double swing_end = mc.periods[1]-transition_period;
      double trans_end = mc.periods[1];
      mc.periods << trans_start, swing_start;
      _script_cmds.push_back(std::make_pair(ANYMAL_FOOT_STATES::SWING_START_TRANS, mc));
      mc.periods << swing_start, swing_end;
      _script_cmds.push_back(std::make_pair(ANYMAL_FOOT_STATES::SWING, mc));
      mc.periods << swing_end, trans_end;
      _script_cmds.push_back(std::make_pair(ANYMAL_FOOT_STATES::SWING_END_TRANS, mc));
    }
  }

  if(t_tmp < script_time_max){
    mc_zero.periods << t_tmp, script_time_max;
    _script_cmds.push_back(std::make_pair(ANYMAL_FOOT_STATES::SUPPORT, mc_zero));
  }
}

void LocoManipulationInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg){
  // add motion_command_script_list_  

  bool is_baseframe;
  Eigen::VectorXd pos_temp, ori_temp;  
  my_utils::readParameter(motion_cfg, "pos", pos_temp);
  my_utils::readParameter(motion_cfg, "ori", ori_temp);
  my_utils::readParameter(motion_cfg, "b_relative", is_baseframe);

  Eigen::VectorXd periods;
  my_utils::readParameter(motion_cfg, "periods", periods);

  Eigen::VectorXi eeidces;
  my_utils::readParameter(motion_cfg, "ee", eeidces);

  MotionCommand mc;
  mc.periods = periods;
  mc.dpose.pos = pos_temp;
  mc.dpose.ori = Eigen::Quaternion<double>(ori_temp[0], ori_temp[1], ori_temp[2], ori_temp[3]);
  mc.dpose.is_baseframe = is_baseframe;

  for(int i=0;i<eeidces.size();++i) {
    int ee = eeidces[i];
    mc.dpose.swing_height = ee>0 ? 0.1:0.0;
    (script_user_cmd_map_[ee]).push_back(mc);
  }
  
}