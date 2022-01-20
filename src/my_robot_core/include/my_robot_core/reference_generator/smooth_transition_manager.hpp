#pragma once

#include <my_robot_core/reference_generator/transition_manager_base.hpp>
#include <my_wbc/Task/BasicTask.hpp>

// Object to manage common trajectory primitives
class SingleWeightTrajectoryManager : public TransitionManagerBase {
 public:
  SingleWeightTrajectoryManager(RobotSystem* _robot);
  ~SingleWeightTrajectoryManager(){};

  double weight_init_;
  double weight_target_;

  // Initialize the joint trajectory
  void setSingleWeightTrajectory(const double& _start_time, 
                          const double& _duration,
                          const double& _init,
                          const double& _target);

  void setSingleWeightInitTarget(const double& _init,
                            const double& _target);
  
  void setSingleWeightTime(const double& _start_time, 
                      const double& _duration);

  void updateSingleWeight(const double& current_time,
                      double &_weight);

  
};
