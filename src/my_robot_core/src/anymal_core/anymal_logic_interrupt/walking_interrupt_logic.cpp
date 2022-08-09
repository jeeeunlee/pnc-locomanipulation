#include <my_robot_core/anymal_core/anymal_logic_interrupt/walking_interrupt_logic.hpp>
#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>

WalkingInterruptLogic::WalkingInterruptLogic(
        ControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "ANYmal Walking Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = ANYmalStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize motion commands
  script_user_cmd_deque_.clear();
}

WalkingInterruptLogic::~WalkingInterruptLogic() {}

// Process Interrupts here
void WalkingInterruptLogic::processInterrupts() {   
  if(b_button_pressed) {
    // std::cout << "[Walking Interrupt Logic] button pressed : " << pressed_button << std::endl;
    switch(pressed_button){
      case 's':
        std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     SCRIPT MOTION      ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::BALANCE) {
          // set stateMachine sequences
          for(auto &it : script_user_cmd_deque_) {
            // set env for simulation
            addStateCommand(ANYMAL_STATES::BALANCE, it);
            addStateCommand(ANYMAL_STATES::SWING_START_TRANS, it);
            addStateCommand(ANYMAL_STATES::SWING, it);
            addStateCommand(ANYMAL_STATES::SWING_END_TRANS, it);
          }
          addStateCommand(ANYMAL_STATES::BALANCE, MotionCommand());

        }
      break;
      case 'w':
        std::cout << "[Walking Interrupt Logic] button w pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com up      ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::BALANCE) {
          POSE_DATA pose_up(0,0,0.01, 1,0,0,0);
          addStateCommand(ANYMAL_STATES::BALANCE, 
                            MotionCommand(pose_up, 0.5) );
        }
      break;
      case 'x':
        std::cout << "[Walking Interrupt Logic] button x pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com down      ---------" << std::endl;
        if (ctrl_arch_->getState() == ANYMAL_STATES::BALANCE) {
          POSE_DATA pose_dn(0,0,-0.01, 1,0,0,0);
          addStateCommand(ANYMAL_STATES::BALANCE, 
                          MotionCommand(pose_dn, 0.5) );
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
}

void WalkingInterruptLogic::addStateCommand(int _state_id, const MotionCommand& _mc){
  MotionCommand* user_state_cmd = new MotionCommand(_mc);
  ctrl_arch_->addState(_state_id, user_state_cmd);
}

void WalkingInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg){
  // add motion_command_script_list_

  Eigen::VectorXd pos_temp, ori_temp;
  bool is_baseframe;
  my_utils::readParameter(motion_cfg, "pos",pos_temp);
  my_utils::readParameter(motion_cfg, "ori", ori_temp);
  my_utils::readParameter(motion_cfg, "b_relative", is_baseframe);

  SWING_DATA swing_motion;
  swing_motion.dpose = POSE_DATA(pos_temp, ori_temp, is_baseframe);
  my_utils::readParameter(motion_cfg, "foot", swing_motion.foot_idx);    
  my_utils::readParameter(motion_cfg, "swing_height", swing_motion.swing_height);

  Eigen::VectorXd motion_periods;
  my_utils::readParameter(motion_cfg, "durations", motion_periods);
  script_user_cmd_deque_.push_back( MotionCommand(swing_motion, motion_periods));

}