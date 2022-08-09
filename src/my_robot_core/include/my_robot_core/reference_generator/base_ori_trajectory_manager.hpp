#pragma once

#include <my_robot_core/reference_generator/trajectory_manager_base.hpp>
#include <my_wbc/Task/BasicTask.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>

// interpolators
#include <my_utils/Math/hermite_curve_vec.hpp>
#include <my_utils/Math/hermite_quaternion_curve.hpp>

// Object to manage common trajectory primitives
class BaseOriTrajectoryManager : public TrajectoryManagerBase {
 public:
  BaseOriTrajectoryManager(RobotSystem* _robot);
  ~BaseOriTrajectoryManager();
  

  Eigen::VectorXd base_pos_ini_;  
  Eigen::Quaterniond base_quat_ini_;
  Eigen::Quaterniond base_quat_des_;

  Eigen::VectorXd base_ori_pos_des_; // 4d
  Eigen::Vector3d base_ori_vel_des_; // 3d
  Eigen::Vector3d base_ori_acc_des_; // 3d

  // Updates the task desired values
  void updateTask(const double& current_time,
                  Task* _base_ori_task);

  // Initialize the swing foot trajectory
  void setBaseOriTrajectory(const double& _start_time, 
                            const double& _duration,
                            const Eigen::Quaterniond &_base_quat_des);

  void setBaseOriTrajectory(const double& _start_time, 
                            const double& _duration);

  // Computes the swing foot trajectory
  void updateBaseOriTrajectory(const double& current_time);


 private:
  Eigen::VectorXd zero_vel_;
  HermiteQuaternionCurve quat_hermite_curve_;

};
