#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/reference_generator/foot_trajectory_manager.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>

FootTrajectoryManager::FootTrajectoryManager(
  RobotSystem* _robot, Task* _pos, Task* _ori, int _foot_idx)
  : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(3, "TrajectoryManager: Foot pos and ori");

  sp_ = ANYmalStateProvider::getStateProvider(robot_);
  foot_pos_task_ = _pos;
  foot_ori_task_ = _ori;
  foot_idx_ = _foot_idx;
  link_idx_ = ANYmalFoot::LinkIdx[foot_idx_];

  // Initialize member variables
  foot_pos_des_.setZero();
  foot_vel_des_.setZero();
  foot_acc_des_.setZero();

  foot_ori_pos_des_ = Eigen::VectorXd::Zero(4); // quat
  foot_ori_vel_des_ = Eigen::VectorXd::Zero(3); // so(3)
  foot_ori_acc_des_ = Eigen::VectorXd::Zero(3); // so(3)

  zero_vel_ = Eigen::VectorXd::Zero(3);

  swing_height_ = 0.04;  // 4cm default
}

FootTrajectoryManager::~FootTrajectoryManager() {}

void FootTrajectoryManager::updatePosTask(const double& current_time) {
  updateFootPosTrajectory(current_time);
  foot_pos_task_->updateTask(foot_pos_des_, 
                            foot_vel_des_, 
                            foot_acc_des_);
}

void FootTrajectoryManager::updateTask(const double& current_time) {
  updateFootPosTrajectory(current_time);
  foot_pos_task_->updateTask(foot_pos_des_, 
                            foot_vel_des_, 
                            foot_acc_des_);                                
  foot_ori_task_->updateTask(foot_ori_pos_des_, 
                            foot_ori_vel_des_, 
                            foot_ori_acc_des_);
}

// Initialize the swing foot trajectory
void FootTrajectoryManager::setFootPosTrajectory(
                              const double& _start_time,
                              const double& _end_time,
                              const MotionCommand& _mc) { 
  // TODO
  swing_height_ = 0.1;
  is_base_frame_ = _mc.dpose.is_baseframe;
  Eigen::VectorXd pos_dev_b = _mc.dpose.pos;  
  my_utils::pretty_print(pos_dev_b,std::cout,"pos_dev_b");  

  traj_start_time_ = _start_time;
  traj_end_time_ = _end_time;
  traj_duration_ = traj_end_time_ - traj_start_time_;  

  //-----------------------------------------
  //            SET FOOT POS
  //-----------------------------------------
  // initialize pos_ini_ with current position
  foot_pos_ini_ = robot_->getBodyNodeIsometry(link_idx_).translation();
  foot_rot_ini_ = robot_->getBodyNodeIsometry(link_idx_).linear();

  if(is_base_frame_) {
    // Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(link_idx_).linear();
    Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(ANYmalBodyNode::base).linear();
    foot_pos_des_ = foot_pos_ini_ + R_wb*pos_dev_b;
  }
  else // absolute coordinate
    foot_pos_des_ = foot_pos_ini_ + pos_dev_b; 
  setSwingPosCurve(foot_pos_ini_,foot_pos_des_,swing_height_);

  //-----------------------------------------
  //            SET FOOT ORI
  //-----------------------------------------
  foot_quat_ini_ = Eigen::Quaternion<double>(
                    robot_->getBodyNodeIsometry(link_idx_).linear() );
  foot_quat_des_ = foot_quat_ini_;
  quat_hermite_curve_.initialize(foot_quat_ini_, zero_vel_,
                                 foot_quat_des_, zero_vel_,traj_duration_);

  sp_->foot_pos_init = foot_pos_ini_;
  sp_->foot_pos_target = foot_pos_des_;
  sp_->check_foot_planner_updated ++; // draw plot on the simulation
}

// Computes the swing foot trajectory
void FootTrajectoryManager::updateFootPosTrajectory(const double& current_time) {
  double t = (current_time - traj_start_time_) ;
  quat_hermite_curve_.evaluate(t, foot_quat_des_);
  quat_hermite_curve_.getAngularVelocity(t, foot_ori_vel_des_);
  quat_hermite_curve_.getAngularAcceleration(t, foot_ori_acc_des_);
  my_utils::convertQuatDesToOriDes(foot_quat_des_, foot_ori_pos_des_);
  // Get foot position and its derivatives
  if (t <= 0.5*traj_duration_) {  // 0.0 <= s < 0.5 use the first trajectory
    foot_pos_des_ = pos_traj_init_to_mid_.evaluate(t);
    foot_vel_des_ = pos_traj_init_to_mid_.evaluateFirstDerivative(t);
    foot_acc_des_ = pos_traj_init_to_mid_.evaluateSecondDerivative(t);
  } else {  // 0.5 <= s < 1.0 use the second trajectory
    t -= 0.5*traj_duration_;
    foot_pos_des_ = pos_traj_mid_to_end_.evaluate(t);
    foot_vel_des_ = pos_traj_mid_to_end_.evaluateFirstDerivative(t);
    foot_acc_des_ = pos_traj_mid_to_end_.evaluateSecondDerivative(t);
  }

  // my_utils::saveVector(foot_pos_des_, "foot_pos_des_");
  // my_utils::saveVector(foot_vel_des_, "foot_vel_des_");
  // my_utils::saveVector(foot_acc_des_, "foot_acc_des_");

  // my_utils::saveVector(foot_quat_des_, "foot_quat_des_");
  // my_utils::saveVector(foot_ori_vel_des_, "foot_ori_vel_des_");
  // my_utils::saveVector(foot_ori_acc_des_, "foot_ori_acc_des_");

}

void FootTrajectoryManager::setSwingPosCurve(const Eigen::VectorXd& foot_pos_ini, 
                                              const Eigen::VectorXd& foot_pos_des,
                                              const double& swing_height) {
  // Set Middle Swing Position/Velocity for Swing
  Eigen::Vector3d foot_pos_mid, foot_vel_mid;  
  Eigen::Vector3d p_b(0, 0, swing_height);
  Eigen::Matrix3d R_wb = robot_->getBodyNodeIsometry(link_idx_).linear();
  foot_pos_mid = 0.5*(foot_pos_des+foot_pos_ini) + R_wb*p_b;  
  foot_vel_mid = 1.0*(foot_pos_des - foot_pos_ini) / traj_duration_;

  // Construct Position trajectories
  pos_traj_init_to_mid_.initialize(foot_pos_ini, zero_vel_, 
                                  foot_pos_mid, foot_vel_mid, 0.5*traj_duration_);
  pos_traj_mid_to_end_.initialize(foot_pos_mid, foot_vel_mid,
                                  foot_pos_des, zero_vel_, 0.5*traj_duration_);

  my_utils::pretty_print(foot_pos_ini_, std::cout, "foot_pos_ini_");
  my_utils::pretty_print(foot_pos_mid, std::cout, "foot_pos_mid");
  my_utils::pretty_print(foot_pos_des_, std::cout, "foot_pos_des_");
  std::cout<<"swing_height_ = "<< swing_height_ << std::endl;
}


