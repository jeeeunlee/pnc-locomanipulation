#include <my_robot_core/magneto_core/magneto_control_architecture/magneto_control_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_logic_interrupt/ClimbingInterruptLogic.hpp>

ClimbingInterruptLogic::ClimbingInterruptLogic(
        MagnetoWbmcControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "Magneto Climbing Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = MagnetoStateProvider::getStateProvider(ctrl_arch_->robot_);
  script_user_cmd_deque_.clear();
}

ClimbingInterruptLogic::~ClimbingInterruptLogic() {
}

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

          for(auto &it : script_user_cmd_deque_) {

            // set env for simulation
            sp_->sim_env_sequence.push_back(std::make_pair(mc_id, it.first));

            // set motion command with state machine identifier
            ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, mc_id, it.second );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING_START_TRANS, mc_id, it.second );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING, mc_id, it.second );
            ctrl_arch_->add_next_state(MAGNETO_STATES::SWING_END_TRANS, mc_id, it.second ); 

            sp_->states_sequence_->addCommand();         
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

// climbset.yaml
// # foot : AL-0, AR-1, BL-2, BR-3
// # frame : base-0, foot-1
// foot: 2
// pos: [-0.1,0,0]
// ori: [1,0,0,0]
// swing_height: 0.05
// duration: 0.4  
// frame: 0
// new_contact_spec: [0.7, 100] # for only simulation [mu, adhesion]

void ClimbingInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg) {
  // motion
  int foot_idx;
  MOTION_DATA md_temp;
  Eigen::VectorXd pos_temp;
  Eigen::VectorXd ori_temp;
  int frame;  
  
  my_utils::readParameter(motion_cfg, "foot", foot_idx);
  my_utils::readParameter(motion_cfg, "pos",pos_temp);
  my_utils::readParameter(motion_cfg, "ori", ori_temp);
  my_utils::readParameter(motion_cfg, "frame", frame);
  my_utils::readParameter(motion_cfg, "duration", md_temp.motion_period);
  my_utils::readParameter(motion_cfg, "swing_height", md_temp.swing_height);
  md_temp.pose = POSE_DATA(pos_temp, ori_temp, frame==0);
  MotionCommand mc_temp = MotionCommand(foot_idx, md_temp);

  // simulation enviroment spec
  double mu;
  double fm;
  my_utils::readParameter(motion_cfg, "mu", mu);
  my_utils::readParameter(motion_cfg, "fm", fm);
  SimMotionCommand* smc_temp = new SimMotionCommand(mc_temp, mu, fm);

  script_user_cmd_deque_.push_back( smc_temp );
}