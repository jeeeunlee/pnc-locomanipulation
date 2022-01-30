#include <my_robot_core/magneto_core/magneto_control_architecture/wbmc_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_logic_interrupt/walking_interrupt_logic.hpp>

WalkingInterruptLogic::WalkingInterruptLogic(
        MagnetoWbmcControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "Magneto Walking Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = MagnetoStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize motion commands
  motion_command_script_list_.clear();
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
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          // set stateMachine sequences
          for(auto &it : script_user_cmd_deque_) {
            // set env for simulation
            ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::BALANCE, it) ;
            ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::SWING_START_TRANS, it) ;
            ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::SWING, it) ;
            ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::SWING_END_TRANS, it) ;
          }
          ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::BALANCE, MotionCommand() );
        }
      break;
      case 'w':
        std::cout << "[Walking Interrupt Logic] button w pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com up      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          POSE_DATA pose_up(0,0,0.01, 1,0,0,0);
          MOTION_DATA motion_com_up(pose_up, 0.5);
          ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::BALANCE, 
                                                MotionCommand(motion_com_up) );
        }
      break;
      case 'x':
        std::cout << "[Walking Interrupt Logic] button x pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com down      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          POSE_DATA pose_dn(0,0,-0.01, 1,0,0,0);
          MOTION_DATA motion_com_dn(pose_dn, 0.5);
          ctrl_arch_->states_sequence_->addState(MAGNETO_STATES::BALANCE, 
                                                MotionCommand(motion_com_dn) );
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
}

void WalkingInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg){
  // add motion_command_script_list_
    int link_idx;
    MOTION_DATA md_temp;

    Eigen::VectorXd pos_temp;
    Eigen::VectorXd ori_temp;
    bool is_baseframe;
    my_utils::readParameter(motion_cfg, "foot", link_idx);
    my_utils::readParameter(motion_cfg, "duration", md_temp.motion_period);
    my_utils::readParameter(motion_cfg, "swing_height", md_temp.swing_height);
    my_utils::readParameter(motion_cfg, "pos",pos_temp);
    my_utils::readParameter(motion_cfg, "ori", ori_temp);
    my_utils::readParameter(motion_cfg, "b_relative", is_baseframe);
    md_temp.pose = POSE_DATA(pos_temp, ori_temp, is_baseframe);
    MotionCommand mc_temp = MotionCommand(link_idx, md_temp);

    script_user_cmd_deque_.push_back(mc_temp);
}