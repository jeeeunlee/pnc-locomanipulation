#pragma once

#include <my_robot_core/interrupt_logic.hpp>
#include <my_robot_core/control_architecture.hpp>

// Forward Declare 
class ANYmalStateProvider;
class ManipulationCommand;

class ManipulationInterruptLogic : public InterruptLogic {
 public:
  ManipulationInterruptLogic(ControlArchitecture* ctrl_arch_);
  ~ManipulationInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  ControlArchitecture* ctrl_arch_;
  ANYmalStateProvider* sp_;

  std::deque<ManipulationCommand> script_user_cmd_deque_;

private:
  void addStateCommand(int _state_id, const ManipulationCommand& _mc);
};
