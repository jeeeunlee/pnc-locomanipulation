#include <my_robot_core/reference_generator/smooth_transition_manager.hpp>

template<typename T>
SmoothTransitionManager<T>::SmoothTransitionManager()
  :TransitionManagerBase() {
  my_utils::pretty_constructor(2, "SmoothTransitionManager");
}

template<typename T>
void SmoothTransitionManager<T>::setTransition(const double& _start_time, 
                                                const double& _duration,
                                                const T& _init,
                                                const T& _target) {
  weight_init_ = _init;
  weight_target_ = _target;
  trans_start_time_ = _start_time;
  trans_duration_ = _duration;
  trans_end_time_ = trans_start_time_ + trans_duration_;
}

template<typename T>
void SmoothTransitionManager<T>::updateTransition(const double& current_time,
                                                        T &_weight) {
  double ts = (current_time - trans_start_time_) / trans_duration_; // 0~1
  ts = 0. > ts ? 0. : ts;
  ts = 1. < ts ? 1. : ts;
  double alpha = 0.5 * (1 - cos(M_PI * ts)); // 0~1
  _weight = (1.-alpha)*weight_init_ + alpha*weight_target_;
}
