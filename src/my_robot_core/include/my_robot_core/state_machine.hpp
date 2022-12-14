#pragma once

#include <my_robot_system/RobotSystem.hpp>
#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Task/Task.hpp>
#include <my_utils/IO/IOUtilities.hpp>

typedef int StateIdentifier;

class StateMachine {
 public:
  StateMachine(const StateIdentifier state_identifier_in, RobotSystem* _robot) {
    robot_ = _robot;
    state_machine_time_ = 0.;
    state_identity_ = state_identifier_in;
    ctrl_start_time_= 0.;
    ctrl_end_time_ = 0.;
    ctrl_duration_ = 0.;
  }

  double getStateMachineStartTime() {return ctrl_start_time_;}
  virtual ~StateMachine() {}

  virtual void oneStep() = 0;
  virtual void firstVisit() = 0;
  virtual void lastVisit() = 0;
  virtual bool endOfState() = 0;
  virtual void initialization(const YAML::Node& node) = 0;

  StateIdentifier getStateIdentity() { return state_identity_; }

 protected:
  StateIdentifier state_identity_;  // Unique integer of this state
  RobotSystem* robot_;              // Pointer to the robot
  double state_machine_time_;

  double ctrl_start_time_;
  double ctrl_end_time_;
  double ctrl_duration_;
};
