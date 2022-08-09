#pragma once

#include <my_robot_core/interrupt_logic.hpp>
#include <my_robot_core/control_architecture.hpp>

// Forward Declare 
class ANYmalStateProvider;
class MotionCommand;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(ControlArchitecture* ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  ControlArchitecture* ctrl_arch_;
  ANYmalStateProvider* sp_;

  std::deque<MotionCommand> script_user_cmd_deque_;

private:
  void addStateCommand(int _state_id, const MotionCommand& _mc);
};
