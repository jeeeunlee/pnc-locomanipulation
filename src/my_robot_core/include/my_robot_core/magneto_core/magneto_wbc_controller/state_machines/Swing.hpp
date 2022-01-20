#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/magneto_core/MagnetoStateProvider.hpp>
#include <my_robot_core/StateMachine.hpp>

class MagnetoControlArchitecture;
class MagnetoTaskAndForceContainer;

class Swing : public StateMachine {
 public:
  Swing(const StateIdentifier state_identifier_in,
                       MagnetoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~Swing();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  MagnetoStateProvider* sp_;
  MagnetoControlArchitecture* ctrl_arch_;
  MagnetoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double ctrl_end_time_;
  double ctrl_duration_;
  int moving_foot_idx_; // link index

  bool state_switch_button_trigger_;

  void _taskUpdate();
  void _weightUpdate();
  void _ResidualMagnetismUpdate();
};
