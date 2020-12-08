#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/StateMachine.hpp>

class MagnetoControlArchitecture;
class MagnetoTaskAndForceContainer;

class Transition : public StateMachine {
 public:
  Transition(const StateIdentifier state_identifier_in,
                       MagnetoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot, bool contact_start);
  ~Transition();

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

  double trans_duration_;

  bool b_contact_start_;
  bool state_switch_button_trigger_;

  int moving_foot_idx_;

  void _taskUpdate();
  void _weightUpdate();
};
