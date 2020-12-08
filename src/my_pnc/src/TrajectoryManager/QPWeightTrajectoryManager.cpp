#include <my_pnc/TrajectoryManager/QPWeightTrajectoryManager.hpp>

QPWeightTrajectoryManager::QPWeightTrajectoryManager(RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: weight");

  // weight_init_ = Eigen::VectorXd::Zero(weight_dim_);
  // weight_target_ = Eigen::VectorXd::Zero(weight_dim_);
  weight_dim_ = 0;
}


void QPWeightTrajectoryManager::setQPWeightTrajectory(const double _start_time, 
                                                  const double _duration,
                                                  const Eigen::VectorXd _init,
                                                  const Eigen::VectorXd _target) {
  setQPWeightInitTarget(_init, _target);
  setQPWeightTime(_start_time, _duration);
}

void QPWeightTrajectoryManager::setQPWeightInitTarget(const Eigen::VectorXd _init,
                                                  const Eigen::VectorXd _target) {
  weight_init_ = _init;
  weight_target_ = _target;
  weight_dim_ =  weight_init_.size(); 
}

void QPWeightTrajectoryManager::setQPWeightTime(const double _start_time, 
                                                  const double _duration) {
  traj_start_time_ = _start_time;
  traj_duration_ = _duration;
  traj_end_time_ = traj_start_time_ + traj_duration_;
}

void QPWeightTrajectoryManager::updateQPWeight(const double current_time,
                                          Eigen::VectorXd &_weight) {
  double ts = (current_time - traj_start_time_) / traj_duration_; // 0~1
  ts = 0. > ts ? 0. : ts;
  ts = 1. < ts ? 1. : ts;
  double alpha = 0.5 * (1 - cos(M_PI * ts)); // 0~1
  // _weight = alpha*weight_init_ + (1.-alpha)*weight_target_;
  _weight = (1.-alpha)*weight_init_ + alpha*weight_target_;
}
