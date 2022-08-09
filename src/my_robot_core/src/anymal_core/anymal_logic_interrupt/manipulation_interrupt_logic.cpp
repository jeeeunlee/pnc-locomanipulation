#include <my_robot_core/anymal_core/anymal_logic_interrupt/manipulation_interrupt_logic.hpp>
#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>

ManipulationInterruptLogic::ManipulationInterruptLogic(
        ControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "ANYmal Manipulation Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = ANYmalStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize motion commands
  script_user_cmd_deque_.clear();
}

ManipulationInterruptLogic::~ManipulationInterruptLogic() {}

// Process Interrupts here
void ManipulationInterruptLogic::processInterrupts() {   
  if(b_button_pressed) {
    // std::cout << "[Walking Interrupt Logic] button pressed : " << pressed_button << std::endl;
    switch(pressed_button){
      case 's':
        std::cout << "[Manipulation Interrupt Logic] button S pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     SCRIPT MOTION      ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::BALANCE) {
          // set stateMachine sequences
          for(auto &it : script_user_cmd_deque_) {
            // set env for simulation
            if(it.get_ee_idx() < 0) addStateCommand(ANYMAL_STATES::BALANCE, it);
            else addStateCommand(ANYMAL_STATES::MANIPULATION, it);
          }
          addStateCommand(ANYMAL_STATES::BALANCE, ManipulationCommand());

        }
      break;
      case 'w':
        std::cout << "[Manipulation Interrupt Logic] button w pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------          EE up         ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::BALANCE) {
          POSE_DATA pose_up(0,0,0.01, 1,0,0,0);
          addStateCommand(ANYMAL_STATES::MANIPULATION, 
                            ManipulationCommand(0, pose_up, 0.5) );
          addStateCommand(ANYMAL_STATES::BALANCE, ManipulationCommand());
        }
      break;
      case 'x':
        std::cout << "[Manipulation Interrupt Logic] button x pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------         EE down        ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::BALANCE) {
          POSE_DATA pose_dn(0,0,-0.01, 1,0,0,0);
          addStateCommand(ANYMAL_STATES::MANIPULATION, 
                          ManipulationCommand(0, pose_dn, 0.5) );
          addStateCommand(ANYMAL_STATES::BALANCE, ManipulationCommand());
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
}

void ManipulationInterruptLogic::addStateCommand(int _state_id, const ManipulationCommand& _mc){
  ManipulationCommand* user_state_cmd = new ManipulationCommand(_mc);
  ctrl_arch_->addState(_state_id, user_state_cmd);
}

void ManipulationInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg){
  // add motion_command_script_list_  
  bool is_baseframe;
  Eigen::VectorXd pos_temp, ori_temp;  
  my_utils::readParameter(motion_cfg, "pos", pos_temp);
  my_utils::readParameter(motion_cfg, "ori", ori_temp);
  my_utils::readParameter(motion_cfg, "b_relative", is_baseframe);
  POSE_DATA dpose = POSE_DATA(pos_temp, ori_temp, is_baseframe);
  int eeidx;
  my_utils::readParameter(motion_cfg, "ee", eeidx);  
  double motion_period;
  my_utils::readParameter(motion_cfg, "duration", motion_period);

  script_user_cmd_deque_.push_back( ManipulationCommand(eeidx, dpose, motion_period) );
}