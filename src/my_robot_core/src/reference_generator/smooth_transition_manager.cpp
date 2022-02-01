#include <my_robot_core/reference_generator/smooth_transition_manager.hpp>


SmoothTransitionManager::SmoothTransitionManager()
  :TransitionManagerBase() {
  my_utils::pretty_constructor(2, "SmoothTransitionManager");
}

void SmoothTransitionManager::setTransition(const double& _start_time, 
                                                const double& _duration,
                                                const double& _init,
                                                const double& _target) {
  weight_init_ = _init;
  weight_target_ = _target;
  trans_start_time_ = _start_time;
  trans_duration_ = _duration;
  trans_end_time_ = trans_start_time_ + trans_duration_;
}

void SmoothTransitionManager::updateTransition(const double& current_time,
                                                  double &_weight) {
  double ts = (current_time - trans_start_time_) / trans_duration_; // 0~1
  ts = 0. > ts ? 0. : ts;
  ts = 1. < ts ? 1. : ts;
  double alpha = 0.5 * (1 - cos(M_PI * ts)); // 0~1
  _weight = (1.-alpha)*weight_init_ + alpha*weight_target_;
}


SmoothVectorTransitionManager::SmoothVectorTransitionManager()
  :TransitionManagerBase() {
  my_utils::pretty_constructor(2, "SmoothVectorTransitionManager");
}

void SmoothVectorTransitionManager::setTransition(const double& _start_time, 
                                                const double& _duration,
                                                const Eigen::VectorXd& _init,
                                                const Eigen::VectorXd& _target) {
  weight_init_ = _init;
  weight_target_ = _target;
  trans_start_time_ = _start_time;
  trans_duration_ = _duration;
  trans_end_time_ = trans_start_time_ + trans_duration_;
}

void SmoothVectorTransitionManager::updateTransition(const double& current_time,
                                                  Eigen::VectorXd &_weight) {
  double ts = (current_time - trans_start_time_) / trans_duration_; // 0~1
  ts = 0. > ts ? 0. : ts;
  ts = 1. < ts ? 1. : ts;
  double alpha = 0.5 * (1 - cos(M_PI * ts)); // 0~1
  _weight = (1.-alpha)*weight_init_ + alpha*weight_target_;
}
