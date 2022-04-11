#pragma once

#include <my_robot_core/reference_generator/trajectory_manager_base.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_wbc/Task/BasicTask.hpp>
#include <my_robot_core/magneto_core/magneto_command_api.hpp>

// interpolators
#include <my_utils/Math/hermite_curve_vec.hpp>
#include <my_utils/Math/hermite_quaternion_curve.hpp>

class MagnetoStateProvider;

// Object to manage common trajectory primitives
class FootPosTrajectoryManager : public TrajectoryManagerBase {
 public:
  FootPosTrajectoryManager(RobotSystem* _robot);
  ~FootPosTrajectoryManager();
  
  int foot_idx_;
  int link_idx_;
  bool is_base_frame_;
  
  Eigen::VectorXd foot_pos_ini_;
  Eigen::MatrixXd foot_rot_ini_; // R_wb
  Eigen::Quaterniond foot_quat_ini_;
  Eigen::Quaterniond foot_quat_des_;

  Eigen::Vector3d foot_pos_des_;
  Eigen::Vector3d foot_vel_des_;
  Eigen::Vector3d foot_acc_des_;

  Eigen::VectorXd foot_ori_pos_des_;
  Eigen::Vector3d foot_ori_vel_des_;
  Eigen::Vector3d foot_ori_acc_des_; 

  // Updates the task desired values
  void updateTask(const double& current_time, Task* _foot_pos_task);
  void updateTask(const double& current_time, Task* _foot_pos_task, Task* _foot_ori_task);

  // Initialize the swing foot trajectory
  void setFootPosTrajectory(const double& _start_time,
                            MotionCommand* _motion_cmd);

  // Computes the swing foot trajectory
  void updateFootPosTrajectory(const double& current_time);

  double getTrajEndTime() {  return traj_end_time_; };
  double getTrajDuration() {  return traj_duration_; };
  int getMovingFootIdx() { return foot_idx_; }

  double getTrajHeight();


 private:
  double swing_height_; // swing height where the middle point will be located
  MotionCommand* mp_curr_;
  Eigen::VectorXd zero_vel_;

  // Hermite Curve containers`
  HermiteCurveVec pos_traj_init_to_mid_;
  HermiteCurveVec pos_traj_mid_to_end_;
  HermiteQuaternionCurve quat_hermite_curve_;

 protected:
  MagnetoStateProvider* sp_;
  void convertQuatDesToOriDes(const Eigen::Quaterniond& quat_in, 
                              Eigen::VectorXd& ori_out);
  void setSwingPosCurve(const Eigen::VectorXd& foot_pos_ini, 
                        const Eigen::VectorXd& foot_pos_des,
                        const double& swing_height);
};
