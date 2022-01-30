#pragma once
#include <Eigen/Dense>
#include <my_robot_core/reference_generator/transition_manager_base.hpp>

// Object to manage common transectory primitives
template<class T>
class SmoothTransitionManager : public TransitionManagerBase {
 public:
  SmoothTransitionManager();
  ~SmoothTransitionManager(){};

  T weight_init_;
  T weight_target_;

  // Initialize transition init to target
  void setTransition(const double& _start_time, 
                    const double& _duration,
                    const T& _init,
                    const T& _target);

  void updateTransition(const double& current_time,
                T &_val);
  
};
