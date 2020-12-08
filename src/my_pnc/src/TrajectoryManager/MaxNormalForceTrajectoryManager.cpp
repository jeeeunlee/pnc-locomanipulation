#include <my_pnc/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>

MaxNormalForceTrajectoryManager::MaxNormalForceTrajectoryManager(RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: MaxNormalForce");

  max_rf_z_init_ = 0.0;
  max_rf_z_target_ = 0.0;
}

void MaxNormalForceTrajectoryManager::setMaxNormalForceTrajectory(
                                          const double _start_time, 
                                          const double _duration,
                                          const double _init,
                                          const double _target) {
  traj_start_time_ = _start_time;
  traj_duration_ = _duration;
  traj_end_time_ = traj_start_time_ + traj_duration_;
  max_rf_z_init_ = _init;
  max_rf_z_target_ = _target;
}

void MaxNormalForceTrajectoryManager::updateMaxNormalForce(
                                          const double current_time,
                                          double &max_rf) {
  double ts = (current_time - traj_start_time_) / traj_duration_; // 0~1
  ts = 0. > ts ? 0. : ts;
  ts = 1. < ts ? 1. : ts;
  double alpha = 0.5 * (1 - cos(M_PI * ts)); // 0~1
  // max_rf = alpha*max_rf_z_init_ + (1.-alpha)*max_rf_z_target_;
  max_rf = (1.-alpha)*max_rf_z_init_ + alpha*max_rf_z_target_;
}
