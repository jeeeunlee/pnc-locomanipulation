#pragma once

#include <vector>

#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>


#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Task/Task.hpp>

// Object to manage common trajectory primitives.
// Base class for the trajectory manager
// Will be used to updateTask

class TrajectoryManagerBase {
 public:
  TrajectoryManagerBase(RobotSystem* _robot) { 
    robot_ = _robot; 
    traj_start_time_ = 0.;
    traj_end_time_ = 0.;
    traj_duration_ = 0.;
    sp_ = MagnetoStateProvider::getStateProvider(robot_);
  }
  virtual ~TrajectoryManagerBase() {}
  virtual void paramInitialization(const YAML::Node& node) = 0;

  RobotSystem* robot_;
  MagnetoStateProvider* sp_;


 protected:
  double traj_start_time_;
  double traj_end_time_;
  double traj_duration_;
};
