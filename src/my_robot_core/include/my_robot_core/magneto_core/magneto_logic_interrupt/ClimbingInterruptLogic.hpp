#pragma once

#include <my_robot_core/InterruptLogic.hpp>

// Forward Declare Control Architecture
class MagnetoControlArchitecture;
class MagnetoStateProvider;
class ClimbingMotionCommand;

class ClimbingInterruptLogic : public InterruptLogic {
 public:
  ClimbingInterruptLogic(MagnetoControlArchitecture* ctrl_arch_);
  ~ClimbingInterruptLogic();

  void processInterrupts();
  void addPresetMotion(const YAML::Node& motion_cfg);

  MagnetoControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  std::deque<ClimbingMotionCommand> motion_command_script_list_;
  MOTION_DATA motion_data_default_;
  ClimbingMotionCommand* motion_command_alfoot_;
  ClimbingMotionCommand* motion_command_blfoot_;
  ClimbingMotionCommand* motion_command_arfoot_;
  ClimbingMotionCommand* motion_command_brfoot_;
  ClimbingMotionCommand* motion_command_instant_;

};
