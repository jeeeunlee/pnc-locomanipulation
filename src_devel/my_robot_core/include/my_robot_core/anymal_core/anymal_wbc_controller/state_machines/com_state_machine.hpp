#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>
#include <my_robot_core/state_machine.hpp>

class ANYmalWbcSpecContainer;
class ANYmalReferenceGeneratorContainer;

class CoMStateMachine : public StateMachine {
 public:
  CoMStateMachine(ANYmalReferenceGeneratorContainer* rg_container);
  ~CoMStateMachine();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

 protected:
  ANYmalStateProvider* sp_;
  ANYmalWbcSpecContainer* ws_container_;
  ANYmalReferenceGeneratorContainer* rg_container_;

  void _taskUpdate();
  void _weightUpdate();
};
