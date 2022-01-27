#pragma once

#include <my_robot_core/reference_generator/trajectory_manager_base.hpp>
#include <my_wbc/Task/BasicTask.hpp>

// Object to manage common trajectory primitives
class JointTrajectoryManager : public TrajectoryManagerBase {
 public:
  JointTrajectoryManager(RobotSystem* _robot);
  ~JointTrajectoryManager(){};

  int full_joint_dim_;
  int active_joint_dim_;
  int joint_dim_;

  Eigen::VectorXd ini_jpos_;
  Eigen::VectorXd target_jpos_;

  Eigen::VectorXd joint_pos_des_;
  Eigen::VectorXd joint_vel_des_;
  Eigen::VectorXd joint_acc_des_;

  // Updates the task desired values
  void updateTask(const double&  current_time, Task* _joint_task);

  // Initialize the joint trajectory
  void setJointTrajectory(const double& _start_time,
                          const double& _duration,
                          const Eigen::VectorXd& _target_jpos);
  void setJointTrajectory(const double& _start_time, 
                          const double& _duration);
                          
  void updateJointTrajectory(const double& current_time);
  
};
