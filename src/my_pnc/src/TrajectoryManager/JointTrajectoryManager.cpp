#include <my_pnc/TrajectoryManager/JointTrajectoryManager.hpp>

JointTrajectoryManager::JointTrajectoryManager(RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: JointPos");

  active_joint_dim_ = _robot->getNumActuatedDofs();
  joint_pos_des_ = Eigen::VectorXd::Zero(active_joint_dim_);
  joint_vel_des_ = Eigen::VectorXd::Zero(active_joint_dim_);
  joint_acc_des_ = Eigen::VectorXd::Zero(active_joint_dim_);
}

void JointTrajectoryManager::updateTask(Task* joint_task) {
  joint_task->updateTask(joint_pos_des_, joint_vel_des_, joint_acc_des_);
}

void JointTrajectoryManager::setJointTrajectory(const double _start_time, 
                                      const double _duration,
                                      const Eigen::VectorXd& _target_jpos) {
  traj_start_time_ = _start_time;
  traj_duration_ = _duration;
  ini_jpos_ = robot_->getActiveQ();
  if(_target_jpos.size() == active_joint_dim_) target_jpos_ = _target_jpos;
  else target_jpos_ = ini_jpos_;

    
}

void JointTrajectoryManager::setJointTrajectory(const double _start_time, 
                                                const double _duration) {
  traj_start_time_ = _start_time;
  traj_duration_ = _duration;
  ini_jpos_ = robot_->getActiveQ();
  target_jpos_ = ini_jpos_;
}

void JointTrajectoryManager::updateJointTrajectory(const double current_time) {
  double traj_time = current_time - traj_start_time_;
  for (int i = 0; i < active_joint_dim_; ++i) {
    joint_pos_des_[i] = my_utils::smooth_changing(
        ini_jpos_[i], target_jpos_[i], traj_duration_, traj_time);
    joint_vel_des_[i] = my_utils::smooth_changing_vel(
        ini_jpos_[i], target_jpos_[i], traj_duration_, traj_time);
    joint_acc_des_[i] = my_utils::smooth_changing_acc(
        ini_jpos_[i], target_jpos_[i], traj_duration_, traj_time);
  }
}
