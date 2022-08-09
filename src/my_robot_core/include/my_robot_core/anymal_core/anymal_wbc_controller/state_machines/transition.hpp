#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>
#include <my_robot_core/state_machine.hpp>

class ANYmalWbcSpecContainer;
class ANYmalReferenceGeneratorContainer;

class Transition : public StateMachine {
 public:
  Transition(const StateIdentifier state_identifier_in,
              ANYmalReferenceGeneratorContainer* rg_container, 
              bool contact_start);
  ~Transition();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  ANYmalStateProvider* sp_;
  ANYmalWbcSpecContainer* ws_container_;
  ANYmalReferenceGeneratorContainer* rg_container_;

  double trans_duration_;

  bool b_contact_start_;
  bool state_switch_button_trigger_;

  int moving_foot_link_idx_;
  int moving_foot_idx_;

  void _taskUpdate();
  void _weightUpdate();
};
