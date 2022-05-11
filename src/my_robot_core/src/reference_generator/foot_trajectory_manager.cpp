#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/reference_generator/foot_trajectory_manager.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>

FootPosTrajectoryManager::FootPosTrajectoryManager(RobotSystem* _robot)
                        : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: FootPos");

  sp_ = MagnetoStateProvider::getStateProvider(robot_);

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

FootPosTrajectoryManager::~FootPosTrajectoryManager() {}

void FootPosTrajectoryManager::updateTask(const double& current_time, 
                                           Task* _foot_pos_task) {
  updateFootPosTrajectory(current_time);
  _foot_pos_task->updateTask(foot_pos_des_, 
                            foot_vel_des_, 
                            foot_acc_des_);
}
void FootPosTrajectoryManager::updateTask(const double& current_time,
                                           Task* _foot_pos_task,
                                           Task* _foot_ori_task) {
  updateFootPosTrajectory(current_time);
  _foot_pos_task->updateTask(foot_pos_des_, 
                            foot_vel_des_, 
                            foot_acc_des_);                                
  _foot_ori_task->updateTask(foot_ori_pos_des_, 
                            foot_ori_vel_des_, 
                            foot_ori_acc_des_);

  // my_utils::pretty_print(foot_pos_des_, std::cout, "foot_pos_traj_");
}

// Initialize the swing foot trajectory
void FootPosTrajectoryManager::setFootPosTrajectory(const double& _start_time,
                                            MotionCommand* _motion_cmd) {
  SWING_DATA motion_cmd_data;
  Eigen::VectorXd pos_dev_b;
  foot_idx_ = -1;
  link_idx_ = -1;
  if(_motion_cmd->get_foot_motion(motion_cmd_data)) {
      // if com motion command is given
      foot_idx_ = motion_cmd_data.foot_idx;
      link_idx_ = MagnetoFoot::LinkIdx[foot_idx_];
      traj_duration_ = _motion_cmd->get_swing_period();
      pos_dev_b = motion_cmd_data.dpose.pos;
      swing_height_ = motion_cmd_data.swing_height;
      is_base_frame_ = motion_cmd_data.dpose.is_baseframe;
      std::cout<<" setFootPosTrajectory " << foot_idx_<< ", " << link_idx_<< std::endl;
  } else {
    // heuristic computation
    std::cout<< "NOOOOO!! no foot motion cmd?" << std::endl;
    traj_duration_ = 1.0;
    pos_dev_b = Eigen::VectorXd::Zero(3);
    swing_height_ = 0.5;
    is_base_frame_=true;
  }
  traj_duration_ = traj_duration_ > 0 ? traj_duration_ : 0.01;
  traj_start_time_ = _start_time;
  traj_end_time_ = traj_start_time_ + traj_duration_;

  //-----------------------------------------
  //            SET FOOT POS
  //-----------------------------------------
  // initialize pos_ini_ with current position
  foot_pos_ini_ = robot_->getBodyNodeIsometry(link_idx_).translation();
  foot_rot_ini_ = robot_->getBodyNodeIsometry(link_idx_).linear();

  if(is_base_frame_) {
    // TODOJE : ? getBodyNodeIsometry(MagnetoBodyNode::base_link)
    Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(MagnetoBodyNode::base_link).linear(); 
    // Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(link_idx_).linear();
    Eigen::Vector3d pos_dev_tang = R_wb*pos_dev_b;
    Eigen::Vector3d pos_dev_normal = (pos_dev_tang.transpose()*sp_->surface_normal[foot_idx_])*sp_->surface_normal[foot_idx_];
    pos_dev_tang = pos_dev_tang - pos_dev_normal;
    foot_pos_des_ = foot_pos_ini_ + pos_dev_tang - 0.01*sp_->surface_normal[foot_idx_];
  }
  else // absolute coordinate
    foot_pos_des_ = foot_pos_ini_ + pos_dev_b;

  my_utils::pretty_print(pos_dev_b,std::cout,"pos_dev_b");
  // my_utils::pretty_print(foot_pos_ini_,std::cout,"foot_pos_ini_");
  // my_utils::pretty_print(foot_pos_des_,std::cout,"foot_pos_des_");
  
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

double FootPosTrajectoryManager::getTrajHeight() {
  // R bw *(p_curr - p_init)
  Eigen::VectorXd pos_dev_b 
    = foot_rot_ini_.transpose() * (foot_pos_des_ - foot_pos_ini_);
  return pos_dev_b[2];
}

// Computes the swing foot trajectory
void FootPosTrajectoryManager::updateFootPosTrajectory(const double& current_time) {
  double t = (current_time - traj_start_time_) ;
  quat_hermite_curve_.evaluate(t, foot_quat_des_);
  quat_hermite_curve_.getAngularVelocity(t, foot_ori_vel_des_);
  quat_hermite_curve_.getAngularAcceleration(t, foot_ori_acc_des_);
  convertQuatDesToOriDes(foot_quat_des_, foot_ori_pos_des_);
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
  //0112 my_utils::saveVector(foot_pos_des_, "foot_pos_des_");
  // //0112 my_utils::saveVector(foot_quat_des_, "foot_quat_des_");

  // my_utils::saveVector(foot_pos_des_, "foot_pos_des_");
  // my_utils::saveVector(foot_vel_des_, "foot_vel_des_");
  // my_utils::saveVector(foot_acc_des_, "foot_acc_des_");

  // my_utils::saveVector(foot_quat_des_, "foot_quat_des_");
  // my_utils::saveVector(foot_ori_vel_des_, "foot_ori_vel_des_");
  // my_utils::saveVector(foot_ori_acc_des_, "foot_ori_acc_des_");

}

void FootPosTrajectoryManager::setSwingPosCurve(const Eigen::VectorXd& foot_pos_ini, 
                                              const Eigen::VectorXd& foot_pos_des,
                                              const double& swing_height) {
  // Set Middle Swing Position/Velocity for Swing
  Eigen::Vector3d foot_pos_mid, foot_vel_mid;  
  // Eigen::Vector3d p_b(0, 0, swing_height);
  // Eigen::Matrix3d R_wb = robot_->getBodyNodeIsometry(link_idx_).linear();
  // foot_pos_mid = 0.5*(foot_pos_des+foot_pos_ini) + R_wb*p_b;

  // mid
  foot_pos_mid = 0.5*(foot_pos_des+foot_pos_ini) + swing_height*sp_->surface_normal[foot_idx_];
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


void FootPosTrajectoryManager::convertQuatDesToOriDes(
                                const Eigen::Quaterniond& quat_in,  
                                Eigen::VectorXd& ori_out) {
  ori_out = Eigen::VectorXd::Zero(4);
  ori_out[0] = quat_in.w();
  ori_out[1] = quat_in.x();
  ori_out[2] = quat_in.y();
  ori_out[3] = quat_in.z();
}