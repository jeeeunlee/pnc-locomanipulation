#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/StateMachine.hpp>

class MagnetoControlArchitecture;
class MagnetoTaskAndForceContainer;

class FullSupport : public StateMachine {
 public:
  FullSupport(const StateIdentifier state_identifier_in,
                       MagnetoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~FullSupport();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  MagnetoStateProvider* sp_;
  MagnetoControlArchitecture* ctrl_arch_;
  MagnetoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double ctrl_end_time_;
  double ctrl_duration_;

  void _taskUpdate();
  void _weightUpdate();
};
