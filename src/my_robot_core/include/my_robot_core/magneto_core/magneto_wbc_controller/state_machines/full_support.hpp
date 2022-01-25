#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_robot_core/StateMachine.hpp>

class MagnetoWbcSpecContainer;
class MagnetoReferenceGeneratorContainer;

class FullSupport : public StateMachine {
 public:
  FullSupport(const StateIdentifier state_identifier_in,
              RobotSystem* _robot,
              MagnetoWbcSpecContainer* ws_container, 
              MagnetoReferenceGeneratorContainer* rg_container);
  ~FullSupport();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  MagnetoStateProvider* sp_;
  MagnetoWbcSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

  double ctrl_start_time_;
  double ctrl_end_time_;
  double ctrl_duration_;

  void _taskUpdate();
  void _weightUpdate();
};
