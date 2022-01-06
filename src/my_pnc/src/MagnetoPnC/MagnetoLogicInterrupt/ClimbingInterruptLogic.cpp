#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoLogicInterrupt/ClimbingInterruptLogic.hpp>

ClimbingInterruptLogic::ClimbingInterruptLogic(
        MagnetoControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "Magneto Climbing Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = MagnetoStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize motion commands
  motion_command_script_list_.clear();

  motion_data_default_.pose = POSE_DATA(-0.1,0,0, 1,0,0,0);
  motion_data_default_.swing_height = 0.05;
  motion_data_default_.motion_period = 0.9;
  
  motion_command_alfoot_ = new ClimbingMotionCommand(
    MagnetoBodyNode::AL_tibia_link, motion_data_default_, 0.7, 100.);
  motion_command_blfoot_ = new ClimbingMotionCommand(
    MagnetoBodyNode::BL_tibia_link, motion_data_default_, 0.7, 100.);
  motion_command_arfoot_ = new ClimbingMotionCommand(
    MagnetoBodyNode::AR_tibia_link, motion_data_default_, 0.7, 100.);
  motion_command_brfoot_ = new ClimbingMotionCommand(
    MagnetoBodyNode::BR_tibia_link, motion_data_default_, 0.7, 100.);
  motion_command_instant_ = new ClimbingMotionCommand();
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
          // initialize trajectory_manager
          // ctrl_arch_->floating_base_lifting_up_manager_->
          // set stateMachine sequences
          for(auto &it : motion_command_script_list_) {          
            ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, it );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING_START_TRANS, it );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING, it );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING_END_TRANS, it );            
          }
          ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, MotionCommand() );
        }
      break;
      case 'w':
        std::cout << "[Climbing Interrupt Logic] button w pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com up      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          MotionCommand mc_com_up;
          POSE_DATA pose_up(0,0,0.01, 1,0,0,0);
          double duration_up = 0.5;
          mc_com_up.add_com_motion(pose_up, duration_up);
          ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, mc_com_up);
        }
      break;
      case 'x':
        std::cout << "[Climbing Interrupt Logic] button x pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com down      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          MotionCommand mc_com_up;
          POSE_DATA pose_up(0,0,-0.01, 1,0,0,0);
          double duration_up = 0.5;
          mc_com_up.add_com_motion(pose_up, duration_up);
          ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, mc_com_up);
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