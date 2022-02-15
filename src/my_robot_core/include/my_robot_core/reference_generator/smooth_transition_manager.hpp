#pragma once
#include <Eigen/Dense>
#include <my_robot_core/reference_generator/transition_manager_base.hpp>

// Object to manage common transectory primitives
class SmoothTransitionManager : public TransitionManagerBase {
 public:
  SmoothTransitionManager(double* _val);
  ~SmoothTransitionManager(){};

  double weight_init_;
  double weight_target_;
  double* val_;

  // Initialize transition init to target
  void setTransition(const double& _start_time, 
                    const double& _duration,
                    const double& _init,
                    const double& _target);

  void updateTransition(const double& current_time); 
};

class SmoothVectorTransitionManager : public TransitionManagerBase {
 public:
  SmoothVectorTransitionManager(Eigen::VectorXd* _val);
  ~SmoothVectorTransitionManager(){};

  Eigen::VectorXd weight_init_;
  Eigen::VectorXd weight_target_;
  Eigen::VectorXd* val_;

  // Initialize transition init to target
  void setTransition(const double& _start_time, 
                    const double& _duration,
                    const Eigen::VectorXd& _init,
                    const Eigen::VectorXd& _target);

  void updateTransition(const double& current_time);
  
};
