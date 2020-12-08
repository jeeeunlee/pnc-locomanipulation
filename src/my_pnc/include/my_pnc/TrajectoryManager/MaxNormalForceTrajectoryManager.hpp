#pragma once

#include <my_pnc/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <my_wbc/Task/BasicTask.hpp>

// Object to manage common trajectory primitives
class MaxNormalForceTrajectoryManager : public TrajectoryManagerBase {
 public:
  MaxNormalForceTrajectoryManager(RobotSystem* _robot);
  ~MaxNormalForceTrajectoryManager(){};

  double max_rf_z_init_;
  double max_rf_z_target_;

  // Initialize the joint trajectory
  void setMaxNormalForceTrajectory(const double _start_time, 
                          const double _duration,
                          const double _init,
                          const double _target);

  void updateMaxNormalForce(const double current_time,
                            double &max_rf_z);

  void paramInitialization(const YAML::Node& node){};
};
