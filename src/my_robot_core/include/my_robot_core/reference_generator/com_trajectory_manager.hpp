#pragma once

#include <my_robot_core/magneto_core/magneto_motion_api.hpp>
#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/reference_generator/TrajectoryManagerBase.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_wbc/Task/BasicTask.hpp>

// interpolators
#include <my_utils/Math/hermite_curve_vec.hpp>

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
                        MotionCommand* _motion_cmd);

  // Initialize the swing com trajectory
  void setCoMTrajectory(double _start_time,
                        double _duration);
// Initialize the swing com trajectory
//   void setCoMTrajectory(const double& _start_time,
//                         std::vector<ContactSpec*> &contact_list);

  // Computes the swing com trajectory
  void updateCoMTrajectory(double current_time);

  double getTrajEndTime() {  return traj_end_time_; };
  double getTrajDuration() {  return traj_duration_; };


 private:
  double swing_height_;
  MotionCommand* mp_curr_;
  Eigen::VectorXd zero_vel_;

  // Hermite Curve containers
  HermiteCurveVec pos_traj;


 protected:
  void setPosCurve(const Eigen::VectorXd& com_pos_ini, 
                    const Eigen::VectorXd& com_pos_des);
};
