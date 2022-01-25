#include <my_robot_core/magneto_core/magneto_control_architecture/magneto_control_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_logic_interrupt/ClimbingInterruptLogic.hpp>

ClimbingInterruptLogic::ClimbingInterruptLogic(
        MagnetoWbmcControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "Magneto Climbing Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = MagnetoStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize simenv, motion commands
  simenv_motion_script_list_.clear();
  motion_identifier = 0;
}

ClimbingInterruptLogic::~ClimbingInterruptLogic() {}

// Process Interrupts here
void ClimbingInterruptLogic::processInterrupts() {   
  if(b_button_pressed) {
    // std::cout << "[Climbing Interrupt Logic] button pressed : " << pressed_button << std::endl;
    switch(pressed_button){
      case 's':
        std::cout << "[Climbing Interrupt Logic] button S pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     SCRIPT MOTION      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          // set stateMachine sequences

          for(auto &it : simenv_motion_script_list_) {
            motion_identifier++;     
            ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, it.second );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING_START_TRANS, it.second );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING, it.second );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING_END_TRANS, it.second );

            sp_->
          }
          ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, MotionCommand() );
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
}

void ClimbingInterruptLogic::addPresetMotion(const YAML::Node& motion_cfg) {

}