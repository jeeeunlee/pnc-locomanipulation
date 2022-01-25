#pragma once

#include <my_robot_core/interrupt_logic.hpp>

// Forward Declare Control Architecture
class MagnetoWbmcControlArchitecture;
class MagnetoStateProvider;
class MotionCommand;
class SimEnvComand;

class ClimbingInterruptLogic : public InterruptLogic {
 public:
  ClimbingInterruptLogic(MagnetoWbmcControlArchitecture* ctrl_arch_);
  ~ClimbingInterruptLogic();

  void processInterrupts();
  void addPresetMotion(const YAML::Node& motion_cfg);

  MagnetoWbmcControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  std::deque<std::pair<SimEnvComand, MotionCommand>> simenv_motion_script_list_; 
  int motion_identifier;
};
