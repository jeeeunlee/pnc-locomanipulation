#include <my_robot_core/reference_generator/ori_trajectory_manager.hpp>

BaseOriTrajectoryManager::BaseOriTrajectoryManager(RobotSystem* _robot)
                        : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: Base Ori");

  
  base_pos_ini_ = Eigen::VectorXd::Zero(4);
  base_quat_ini_ = Eigen::Quaternion<double> (1,0,0,0);;
  base_quat_des_ = Eigen::Quaternion<double> (1,0,0,0);;

  base_ori_pos_des_ = Eigen::VectorXd::Zero(4); // quat
  base_ori_vel_des_ = Eigen::VectorXd::Zero(3); // so(3)
  base_ori_acc_des_ = Eigen::VectorXd::Zero(3); // so(3)

  zero_vel_ = Eigen::VectorXd::Zero(3);
}

BaseOriTrajectoryManager::~BaseOriTrajectoryManager() {}

void BaseOriTrajectoryManager::updateTask(const double& current_time, 
                                          Task* _base_ori_task) {
  updateBaseOriTrajectory(current_time);
  _base_ori_task->updateTask(base_ori_pos_des_, 
                            base_ori_vel_des_, 
                            base_ori_acc_des_);
}

// Initialize the swing base_ori trajectory

void BaseOriTrajectoryManager::setBaseOriTrajectory(const double& _start_time, 
                          const double& _duration,
                          const Eigen::Quaterniond &_base_quat_des) {
  
  traj_start_time_ = _start_time;
  traj_duration_ = _duration;
  traj_end_time_ = traj_start_time_ + traj_duration_;

  base_quat_ini_ = Eigen::Quaternion<double>(
                          robot_->getBodyNodeIsometry(
                            MagnetoBodyNode::base_link).linear() );
  base_quat_des_ = _base_quat_des;
  quat_hermite_curve_.initialize(base_quat_ini_, zero_vel_,
                                 base_quat_des_, zero_vel_);
}

void BaseOriTrajectoryManager::setBaseOriTrajectory(const double& _start_time, 
                          const double& _duration) {
  Eigen::Quaterniond _base_quat_des 
                        = Eigen::Quaternion<double>(
                          robot_->getBodyNodeIsometry(
                            MagnetoBodyNode::base_link).linear() );
  setBaseOriTrajectory(_start_time, _duration, _base_quat_des);
}

// Computes the swing base_ori trajectory
void BaseOriTrajectoryManager::updateBaseOriTrajectory(const double& current_time) {
  double s = (current_time - traj_start_time_) / traj_duration_;

  quat_hermite_curve_.evaluate(s, base_quat_des_);
  quat_hermite_curve_.getAngularVelocity(s, base_ori_vel_des_);
  quat_hermite_curve_.getAngularAcceleration(s, base_ori_acc_des_);
  convertQuatDesToOriDes(base_quat_des_, base_ori_pos_des_);
  
}

void BaseOriTrajectoryManager::convertQuatDesToOriDes(
                                const Eigen::Quaterniond& quat_in,  
                                Eigen::VectorXd& ori_out) {
  ori_out = Eigen::VectorXd::Zero(4);
  ori_out[0] = quat_in.w();
  ori_out[1] = quat_in.x();
  ori_out[2] = quat_in.y();
  ori_out[3] = quat_in.z();
}