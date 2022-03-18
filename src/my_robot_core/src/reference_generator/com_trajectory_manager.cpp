#include <my_robot_core/reference_generator/com_trajectory_manager.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>

CoMTrajectoryManager::CoMTrajectoryManager(RobotSystem* _robot)
                        : TrajectoryManagerBase(_robot) {
  my_utils::pretty_constructor(2, "TrajectoryManager: CoM");

  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // Initialize member variables
  com_pos_des_.setZero();
  com_vel_des_.setZero();
  com_acc_des_.setZero();

  zero_vel_ = Eigen::VectorXd::Zero(3);
}

CoMTrajectoryManager::~CoMTrajectoryManager() {}

void CoMTrajectoryManager::updateTask(const double&  current_time, 
                                      Task* _com_pos_task) {
  updateCoMTrajectory(current_time);
  _com_pos_task->updateTask(com_pos_des_, 
                            com_vel_des_, 
                            com_acc_des_); 

  sp_->com_pos_des = com_pos_des_;
  sp_->com_vel_des = com_vel_des_;
  // sp_->com_acc_des = com_acc_des_;
}

// Initialize the swing com trajectory
void CoMTrajectoryManager::setCoMTrajectory(double _start_time,
                                    const ComMotionCommand& _motion_cmd) {

  my_utils::pretty_print(_motion_cmd.pa, std::cout, "com_pos_ini_");
  my_utils::pretty_print(_motion_cmd.va, std::cout, "com_vel_ini_");
  
  traj_start_time_ = _start_time;  
  traj_duration_ = _motion_cmd.motion_period;
  traj_end_time_ = traj_start_time_ + traj_duration_;
  if(_motion_cmd.is_acc_constant){
    Eigen::Vector3d pb, vb;
    vb = _motion_cmd.va + _motion_cmd.acc*traj_duration_;
    pb = _motion_cmd.pa 
          + _motion_cmd.va*traj_duration_ 
          + 0.5*_motion_cmd.acc*traj_duration_*traj_duration_;
    pos_traj.initialize(_motion_cmd.pa, _motion_cmd.va, 
                      pb, vb, traj_duration_);
  } else{
    pos_traj.initialize(_motion_cmd.pa, _motion_cmd.va, 
                        _motion_cmd.pb, _motion_cmd.vb, 
                        traj_duration_);
    my_utils::pretty_print(_motion_cmd.pb, std::cout, "com_pos_des_");
    my_utils::pretty_print(_motion_cmd.vb, std::cout, "com_vel_des_");
  }
}

void CoMTrajectoryManager::setCoMTrajectory(double _start_time,
                                            double _duration) {
  
  traj_start_time_ = _start_time;  
  traj_duration_ = _duration;
  traj_end_time_ = traj_start_time_ + traj_duration_;

  // initialize pos_ini_ with current position
  com_pos_ini_ = robot_ ->getCoMPosition();  
  com_pos_des_ = com_pos_ini_;
  
  pos_traj.initialize(com_pos_ini_, zero_vel_, 
                      com_pos_des_, zero_vel_, traj_duration_);

}

// Computes the swing com trajectory
void CoMTrajectoryManager::updateCoMTrajectory(double current_time) {
  double t = (current_time - traj_start_time_ + MagnetoAux::servo_rate) ;
  // Get com position and its derivatives
  // std::cout<<"s = " << s << std::endl;
  com_pos_des_ = pos_traj.evaluate(t);
  com_vel_des_ = pos_traj.evaluateFirstDerivative(t);
  com_acc_des_ = pos_traj.evaluateSecondDerivative(t);

  my_utils::saveVector(com_pos_des_, "com_pos_des_");
  my_utils::saveVector(com_vel_des_, "com_vel_des_");
  my_utils::saveVector(com_acc_des_, "com_acc_des_");

}


