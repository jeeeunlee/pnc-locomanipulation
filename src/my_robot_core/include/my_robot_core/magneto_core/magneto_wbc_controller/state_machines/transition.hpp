#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_robot_core/state_machine.hpp>

class MagnetoWbcSpecContainer;
class MagnetoReferenceGeneratorContainer;

class Transition : public StateMachine {
 public:
  Transition(const StateIdentifier state_identifier_in,
              RobotSystem* _robot,
              MagnetoWbcSpecContainer* ws_container, 
              MagnetoReferenceGeneratorContainer* rg_container, 
              bool contact_start);
  ~Transition();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  MagnetoStateProvider* sp_;
  MagnetoWbcSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

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
