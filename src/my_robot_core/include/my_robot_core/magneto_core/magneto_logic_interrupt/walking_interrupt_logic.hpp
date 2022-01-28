#pragma once

#include <my_robot_core/interrupt_logic.hpp>

// Forward Declare Control Architecture
class MagnetoWbmcControlArchitecture;
class MagnetoStateProvider;
class MotionCommand;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(MagnetoWbmcControlArchitecture* ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  MagnetoWbmcControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  std::deque<MotionCommand*> script_user_cmd_deque_;
};
