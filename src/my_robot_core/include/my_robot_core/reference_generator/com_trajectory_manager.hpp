#pragma once

#include <my_robot_core/anymal_core/anymal_command_api.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/reference_generator/trajectory_manager_base.hpp>
#include <my_wbc/Task/BasicTask.hpp>

// interpolators
#include <my_utils/Math/hermite_curve_vec.hpp>

class ANYmalStateProvider;

// Object to manage common trajectory primitives
class CoMTrajectoryManager : public TrajectoryManagerBase {
 public:
  CoMTrajectoryManager(RobotSystem* _robot);
  ~CoMTrajectoryManager();
  

  Eigen::VectorXd com_pos_ini_;  

  Eigen::Vector3d com_pos_des_;
  Eigen::Vector3d com_vel_des_;
  Eigen::Vector3d com_acc_des_;

  // Updates the task desired values
  void updateTask(const double& current_time, Task* _com_pos_task);

  // Initialize the swing com trajectory
  void setCoMTrajectory(double  _start_time,
                        const ComMotionCommand& _motion_cmd);

  // Initialize the swing com trajectory
  void setCoMTrajectory(double _start_time,
                        double _duration);

  // Computes the swing com trajectory
  void updateCoMTrajectory(double current_time);

  double getTrajEndTime() {  return traj_end_time_; };
  double getTrajDuration() {  return traj_duration_; };


 private:
  Eigen::VectorXd zero_vel_;

  // Hermite Curve containers
  HermiteCurveVec pos_traj;
  ANYmalStateProvider* sp_;

};
