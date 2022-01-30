#include <my_robot_core/reference_generator/com_trajectory_manager.hpp>

CoMTrajectoryManager::CoMTrajectoryManager(RobotSystem* _robot)
                        : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: CoM");

  // Initialize member variables
  com_pos_des_.setZero();
  com_vel_des_.setZero();
  com_acc_des_.setZero();

  zero_vel_ = Eigen::VectorXd::Zero(3);
}

CoMTrajectoryManager::~CoMTrajectoryManager() {}

void CoMTrajectoryManager::updateTask(const double&  current_time, Task* _com_pos_task) {
  updateCoMTrajectory(current_time);
  _com_pos_task->updateTask(com_pos_des_, 
                            com_vel_des_, 
                            com_acc_des_);
}

// Initialize the swing com trajectory
void CoMTrajectoryManager::setCoMTrajectory(double _start_time,
                              MotionCommand* _motion_cmd) {
  
  MOTION_DATA motion_cmd_data = MOTION_DATA();
  Eigen::VectorXd pos_dev_b;
  if(_motion_cmd->get_com_motion(motion_cmd_data)){
      // if com motion command is given
      traj_duration_ = motion_cmd_data.motion_period;
      pos_dev_b = motion_cmd_data.pose.pos;
  } else if(_motion_cmd->get_foot_motion(motion_cmd_data)) {
    // heuristic computation
    traj_duration_ = motion_cmd_data.motion_period;
    pos_dev_b = 0.25 * motion_cmd_data.pose.pos;
  } else {
    // no motion given
    traj_duration_ = motion_cmd_data.motion_period; // zero
    pos_dev_b = motion_cmd_data.pose.pos; // zero
  }
  
  traj_duration_ = traj_duration_ > 0 ? traj_duration_ : 0.01;
  traj_start_time_ = _start_time;  
  traj_end_time_ = traj_start_time_ + traj_duration_;
  // initialize pos_ini_ with current position
  com_pos_ini_ = robot_ ->getCoMPosition(); 
  if(motion_cmd_data.pose.is_baseframe) {
    Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(MagnetoBodyNode::base_link).linear();    
    com_pos_des_ = com_pos_ini_ + R_wb*pos_dev_b;     
  }
  else // absolute coordinate
    com_pos_des_ = com_pos_ini_ + pos_dev_b;
  
  my_utils::pretty_print(com_pos_ini_, std::cout, "com_pos_ini_");
  my_utils::pretty_print(com_pos_des_, std::cout, "com_pos_des_");

  setPosCurve(com_pos_ini_, com_pos_des_);
}

void CoMTrajectoryManager::setCoMTrajectory(double _start_time,
                                            double _duration) {
  
  traj_start_time_ = _start_time;  
  traj_duration_ = _duration;
  traj_end_time_ = traj_start_time_ + traj_duration_;

  // initialize pos_ini_ with current position
  com_pos_ini_ = robot_ ->getCoMPosition();  
  com_pos_des_ = com_pos_ini_;
  
  my_utils::pretty_print(com_pos_ini_, std::cout, "com_pos_ini_");
  my_utils::pretty_print(com_pos_des_, std::cout, "com_pos_des_");

  setPosCurve(com_pos_ini_, com_pos_des_);
}


// Computes the swing com trajectory
void CoMTrajectoryManager::updateCoMTrajectory(double current_time) {
  double s = (current_time - traj_start_time_) / traj_duration_;
  // Get com position and its derivatives
  // std::cout<<"s = " << s << std::endl;
  com_pos_des_ = pos_traj.evaluate(s);
  com_vel_des_ = pos_traj.evaluateFirstDerivative(s);
  com_acc_des_ = pos_traj.evaluateSecondDerivative(s);

}

void CoMTrajectoryManager::setPosCurve(const Eigen::VectorXd& com_pos_ini, 
                      const Eigen::VectorXd& com_pos_des) {  

  // Construct Position trajectories
  // Eigen::VectorXd com_vel_mid = (com_pos_des -com_pos_ini)/traj_duration_;
  pos_traj.initialize(com_pos_ini, zero_vel_, 
                      com_pos_des, zero_vel_);
}

