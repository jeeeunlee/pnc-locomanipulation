#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/reference_generator/ee_trajectory_manager.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>

EETrajectoryManager::EETrajectoryManager(RobotSystem* _robot)
                        : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(3, "TrajectoryManager: ee");

  sp_ = ANYmalStateProvider::getStateProvider(robot_);

  // Initialize member variables
  ee_pos_des_.setZero();
  ee_vel_des_.setZero();
  ee_acc_des_.setZero();

  ee_ori_pos_des_ = Eigen::VectorXd::Zero(4); // quat
  ee_ori_vel_des_ = Eigen::VectorXd::Zero(3); // so(3)
  ee_ori_acc_des_ = Eigen::VectorXd::Zero(3); // so(3)

  zero_vel_ = Eigen::VectorXd::Zero(3);
}

EETrajectoryManager::~EETrajectoryManager() {}

void EETrajectoryManager::updateTask(const double& current_time, 
                                           Task* _ee_pos_task) {
  updateEETrajectory(current_time);
  _ee_pos_task->updateTask(ee_pos_des_, 
                            ee_vel_des_, 
                            ee_acc_des_);
}
void EETrajectoryManager::updateTask(const double& current_time,
                                           Task* _ee_pos_task,
                                           Task* _ee_ori_task) {
  updateEETrajectory(current_time);
  _ee_pos_task->updateTask(ee_pos_des_, 
                            ee_vel_des_, 
                            ee_acc_des_);                                
  _ee_ori_task->updateTask(ee_ori_pos_des_, 
                            ee_ori_vel_des_, 
                            ee_ori_acc_des_);

  // my_utils::pretty_print(ee_pos_des_, std::cout, "ee_pos_traj_");
}

void EETrajectoryManager::setEETrajectory(const double& _start_time,
                                          const double& _motion_period){
  ManipulationCommand motion_cmd(ANYmalBodyNode::ur3_ee_link, 
                                POSE_DATA(), 
                                _motion_period);
  setEETrajectory(_start_time, &motion_cmd);
}

// Initialize the swing ee trajectory
void EETrajectoryManager::setEETrajectory(const double& _start_time,
                                            ManipulationCommand* _motion_cmd) {
  POSE_DATA motion_cmd_data;
  Eigen::VectorXd pos_dev_b;
  int ee_idx_ = 0;
  link_idx_ = ANYmalBodyNode::ur3_ee_link; 

  if( _motion_cmd->get_ee_motion(motion_cmd_data)) {
    ee_idx_ = _motion_cmd->get_ee_idx(); 
    link_idx_ = ANYmalEE::EEarm;
    traj_duration_ = _motion_cmd->get_motion_period(); 
    pos_dev_b = motion_cmd_data.pos;
    is_base_frame_ = motion_cmd_data.is_baseframe;
  } else {
    traj_duration_ = 1.0;
    pos_dev_b = Eigen::VectorXd::Zero(3);
    is_base_frame_=true;
  }

  
  traj_duration_ = traj_duration_ > 0 ? traj_duration_ : 0.01;
  traj_start_time_ = _start_time;
  traj_end_time_ = traj_start_time_ + traj_duration_;

  //-----------------------------------------
  //            SET EE POS
  //-----------------------------------------
  // initialize pos_ini_ with current position
  ee_pos_ini_ = robot_->getBodyNodeIsometry(link_idx_).translation();
  ee_rot_ini_ = robot_->getBodyNodeIsometry(link_idx_).linear();
  if(is_base_frame_) {
    Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(ANYmalBodyNode::base).linear(); 
    ee_pos_des_ = ee_pos_ini_ + R_wb*pos_dev_b;
  }
  else // absolute coordinate
    ee_pos_des_ = ee_pos_ini_ + pos_dev_b;
  pos_hermite_curve_.initialize(ee_pos_ini_, zero_vel_, 
                  ee_pos_des_, zero_vel_, traj_duration_);

  my_utils::pretty_print(pos_dev_b,std::cout,"pos_dev_b_ee");
  // my_utils::pretty_print(ee_pos_ini_,std::cout,"ee_pos_ini_");
  // my_utils::pretty_print(ee_pos_des_,std::cout,"ee_pos_des_"); 

  //-----------------------------------------
  //            SET EE ORI
  //-----------------------------------------  
  ee_quat_ini_ = Eigen::Quaternion<double>( robot_->getBodyNodeIsometry(link_idx_).linear() );
  ee_quat_des_ = ee_quat_ini_;
  quat_hermite_curve_.initialize(ee_quat_ini_, zero_vel_,
                                 ee_quat_des_, zero_vel_,traj_duration_);

  sp_->ee_pos_init = ee_pos_ini_;
  sp_->ee_pos_target = ee_pos_des_;
  sp_->check_ee_planner_updated ++; // draw plot on the simulation

}

// Computes the swing ee trajectory
void EETrajectoryManager::updateEETrajectory(const double& current_time) {
  double t = (current_time - traj_start_time_) ;

  // Get ee position and its derivatives
  ee_pos_des_ = pos_hermite_curve_.evaluate(t);
  ee_vel_des_ = pos_hermite_curve_.evaluateFirstDerivative(t);
  ee_acc_des_ = pos_hermite_curve_.evaluateSecondDerivative(t);

  // Get ee orientation
  quat_hermite_curve_.evaluate(t, ee_quat_des_);
  quat_hermite_curve_.getAngularVelocity(t, ee_ori_vel_des_);
  quat_hermite_curve_.getAngularAcceleration(t, ee_ori_acc_des_);
  my_utils::convertQuatDesToOriDes(ee_quat_des_, ee_ori_pos_des_);

  // my_utils::saveVector(ee_pos_des_, "ee_pos_des_");
  // my_utils::saveVector(ee_vel_des_, "ee_vel_des_");
  // my_utils::saveVector(ee_acc_des_, "ee_acc_des_");
  // my_utils::saveVector(ee_quat_des_, "ee_quat_des_");
  // my_utils::saveVector(ee_ori_vel_des_, "ee_ori_vel_des_");
  // my_utils::saveVector(ee_ori_acc_des_, "ee_ori_acc_des_");
}

