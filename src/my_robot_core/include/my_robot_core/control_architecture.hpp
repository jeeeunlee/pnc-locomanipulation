#pragma once

#include <map>

#include <../my_utils/Configuration.h>
#include <my_robot_core/state_machine.hpp>

#include <my_robot_system/RobotSystem.hpp>

#include <my_utils/IO/IOUtilities.hpp>

// Generic Control Architecture Object
class ControlArchitecture {
 public:
  ControlArchitecture(RobotSystem* _robot) {
    robot_ = _robot;
  }

  virtual ~ControlArchitecture() {}

  virtual void ControlArchitectureInitialization() = 0;
  virtual void getCommand(void* _command) = 0;  
  virtual void addState(StateIdentifier _state_id, void* _user_state_command) = 0;
  
  int getState() { return state_; }
  int getPrevState() { return prev_state_; }
  RobotSystem* robot_;

  std::map<StateIdentifier, StateMachine*> state_machines_;

 protected:
  int state_;
  int prev_state_;
};
