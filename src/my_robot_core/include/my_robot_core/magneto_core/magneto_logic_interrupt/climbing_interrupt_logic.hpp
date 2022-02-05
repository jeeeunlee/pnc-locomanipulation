#pragma once

#include <my_robot_core/interrupt_logic.hpp>
#include <my_robot_core/control_architecture.hpp>

// Forward Declare 
class MagnetoStateProvider;
class SimMotionCommand;
class MagnetoUserStateCommand;

class ClimbingInterruptLogic : public InterruptLogic {
 public:
  ClimbingInterruptLogic(ControlArchitecture* ctrl_arch_);
  ~ClimbingInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  ControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  MagnetoUserStateCommand* user_state_cmd_;
  std::deque<SimMotionCommand> script_user_cmd_deque_;

private:
  void addStateCommand(int _state_id, const SimMotionCommand& _smc);
};
