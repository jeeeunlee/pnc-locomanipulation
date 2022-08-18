#pragma once

#include <my_robot_core/reference_generator/trajectory_manager_base.hpp>
#include <my_wbc/Task/BasicTask.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>

// interpolators
#include <my_utils/Math/hermite_curve_vec.hpp>
#include <my_utils/Math/hermite_quaternion_curve.hpp>

class ANYmalStateProvider;

// Object to manage common trajectory primitives
class EETrajectoryManager : public TrajectoryManagerBase {
 public:
  EETrajectoryManager(RobotSystem* _robot, Task* _pos, Task* _ori);
  ~EETrajectoryManager();
  
  int ee_idx_;
  int link_idx_;
  bool is_base_frame_;
  
  Eigen::VectorXd ee_pos_ini_;
  Eigen::MatrixXd ee_rot_ini_; // R_wb
  Eigen::Quaterniond ee_quat_ini_;
  Eigen::Quaterniond ee_quat_des_;

  Eigen::Vector3d ee_pos_des_;
  Eigen::Vector3d ee_vel_des_;
  Eigen::Vector3d ee_acc_des_;

  Eigen::VectorXd ee_ori_pos_des_;
  Eigen::Vector3d ee_ori_vel_des_;
  Eigen::Vector3d ee_ori_acc_des_; 

  // Updates the task desired values
  void updatePosTask(const double& current_time);
  void updateTask(const double& current_time);

  // Initialize the swing ee trajectory
  void setEETrajectory(const double& _start_time,
                      const double& _end_time,
                      MotionCommand _motion_cmd);

  // Computes the swing ee trajectory
  void updateEETrajectory(const double& current_time);

  double getTrajEndTime() {  return traj_end_time_; };
  double getTrajDuration() {  return traj_duration_; };
  int getMovingEEIdx() { return ee_idx_; }


 private:
  MotionCommand* mp_curr_;
  Eigen::VectorXd zero_vel_;

  // Hermite Curve containers`
  HermiteCurveVec pos_hermite_curve_;
  HermiteQuaternionCurve quat_hermite_curve_;

  Task* ee_pos_task_;
  Task* ee_ori_task_;

 protected:
  ANYmalStateProvider* sp_;

};
