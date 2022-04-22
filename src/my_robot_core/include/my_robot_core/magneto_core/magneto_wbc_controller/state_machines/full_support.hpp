#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_robot_core/state_machine.hpp>

class MagnetoWbcSpecContainer;
class MagnetoReferenceGeneratorContainer;

class FullSupport : public StateMachine {
 public:
  FullSupport(const StateIdentifier state_identifier_in,
              MagnetoReferenceGeneratorContainer* rg_container);
  ~FullSupport();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

 protected:
  MagnetoStateProvider* sp_;
  MagnetoWbcSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

  void _taskUpdate();
  void _weightUpdate();
};
