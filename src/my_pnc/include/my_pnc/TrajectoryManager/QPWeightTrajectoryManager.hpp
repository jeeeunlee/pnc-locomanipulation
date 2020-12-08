#pragma once

#include <my_pnc/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <my_wbc/Task/BasicTask.hpp>

// Object to manage common trajectory primitives
class QPWeightTrajectoryManager : public TrajectoryManagerBase {
 public:
  QPWeightTrajectoryManager(RobotSystem* _robot);
  ~QPWeightTrajectoryManager(){};

  int weight_dim_;
  Eigen::VectorXd weight_init_;
  Eigen::VectorXd weight_target_;

  // Initialize the joint trajectory
  void setQPWeightTrajectory(const double _start_time, 
                          const double _duration,
                          const Eigen::VectorXd _init,
                          const Eigen::VectorXd _target);

  void setQPWeightInitTarget(const Eigen::VectorXd _init,
                            const Eigen::VectorXd _target);
  
  void setQPWeightTime(const double _start_time, 
                      const double _duration);

  void updateQPWeight(const double current_time,
                      Eigen::VectorXd &_weight);

  void paramInitialization(const YAML::Node& node){};
};
