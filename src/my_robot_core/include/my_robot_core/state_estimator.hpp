#pragma once

#include <my_robot_system/RobotSystem.hpp>
#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Task/Task.hpp>
#include <my_utils/IO/IOUtilities.hpp>


class StateEstimator {
 public:
  StateEstimator(RobotSystem* _robot) {
    robot_ = _robot;
  }

  virtual ~StateEstimator() {}

  virtual void evaluate() = 0;
  virtual void initialization(const YAML::Node& node) = 0;

 protected:
  RobotSystem* robot_;              // Pointer to the robot
  
};