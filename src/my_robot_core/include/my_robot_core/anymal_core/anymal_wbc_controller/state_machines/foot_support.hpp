#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>
#include <my_robot_core/state_machine.hpp>

class ANYmalWbcSpecContainer;
class ANYmalReferenceGeneratorContainer;

class Support : public StateMachine {
 public:
  Support(const StateIdentifier state_identifier_in,
              ANYmalReferenceGeneratorContainer* rg_container);
  ~Support();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

 protected:
  ANYmalStateProvider* sp_;
  ANYmalWbcSpecContainer* ws_container_;
  ANYmalReferenceGeneratorContainer* rg_container_;

  int foot_link_idx_; // link index
  int foot_idx_; // foot index 0~3

  void _taskUpdate();
  void _weightUpdate();
};
