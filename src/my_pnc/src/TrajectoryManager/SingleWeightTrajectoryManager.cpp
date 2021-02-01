#include <my_pnc/TrajectoryManager/SingleWeightTrajectoryManager.hpp>

SingleWeightTrajectoryManager::SingleWeightTrajectoryManager(RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: weight");
}


void SingleWeightTrajectoryManager::setSingleWeightTrajectory(const double& _start_time, 
                                                          const double& _duration,
                                                          const double& _init,
                                                          const double& _target) {
  setSingleWeightInitTarget(_init, _target);
  setSingleWeightTime(_start_time, _duration);
}

void SingleWeightTrajectoryManager::setSingleWeightInitTarget(const double& _init,
                                                          const double& _target) {
  weight_init_ = _init;
  weight_target_ = _target;
}

void SingleWeightTrajectoryManager::setSingleWeightTime(const double& _start_time, 
                                                        const double& _duration) {
  traj_start_time_ = _start_time;
  traj_duration_ = _duration;
  traj_end_time_ = traj_start_time_ + traj_duration_;
}

void SingleWeightTrajectoryManager::updateSingleWeight(const double& current_time,
                                                        double &_weight) {
  double ts = (current_time - traj_start_time_) / traj_duration_; // 0~1
  ts = 0. > ts ? 0. : ts;
  ts = 1. < ts ? 1. : ts;
  double alpha = 0.5 * (1 - cos(M_PI * ts)); // 0~1
  _weight = (1.-alpha)*weight_init_ + alpha*weight_target_;
}
