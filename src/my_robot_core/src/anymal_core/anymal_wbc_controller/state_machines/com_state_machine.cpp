#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/com_state_machine.hpp>

CoMStateMachine::CoMStateMachine(ANYmalReferenceGeneratorContainer* rg_container) 
    : StateMachine(rg_container->robot_) {
  my_utils::pretty_constructor(2, "StateMachine: CoMStateMachine (Balance)");

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

CoMStateMachine::~CoMStateMachine() {}

void CoMStateMachine::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[CoMStateMachine Balance] Start" << std::endl;

  // -- set current motion param
  MotionCommand mc_curr_ = sp_->com_motion_command;
  // mc_curr_.printMotionInfo();
  
  ctrl_start_time_ = sp_->curr_time;  
  ctrl_end_time_ = sp_->motion_start_time + mc_curr_.periods[1];
  ctrl_duration_ = ctrl_end_time_ - ctrl_start_time_;
  

  // ---------------------------------------
  //      Planning
  // ---------------------------------------
  // robot goal configuration
  Eigen::VectorXd q_init =  robot_->getQ();
  Eigen::Vector3d pcom = robot_->getCoMPosition();   
  Eigen::Vector3d zero_vel(0.0, 0.0, 0.0);
  Eigen::Vector3d com_dpos = mc_curr_.dpose.pos;
  if(mc_curr_.dpose.is_baseframe){
      Eigen::MatrixXd Rwb = robot_->getBodyNodeIsometry(ANYmalBodyNode::base).linear();
      com_dpos = Rwb*com_dpos;
  }
  Eigen::Vector3d pc_goal = pcom + com_dpos;  
  ComMotionCommand mc_com = 
      ComMotionCommand( pcom, zero_vel, pc_goal, zero_vel, ctrl_duration_ );

  sp_->com_pos_init = pcom;
  sp_->com_pos_target = pc_goal;

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------  
  // --set com traj
  rg_container_->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, mc_com);

  // -- set base ori traj
  rg_container_->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_, ctrl_duration_);

  // -- set joint traj
  rg_container_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_, ctrl_duration_, q_init);
 
  // -- set task_list in taf with hierachy
  ws_container_->add_task_list(ANYMAL_TASK::COM);
  ws_container_->add_task_list(ANYMAL_TASK::BASE_ORI);
  ws_container_->add_task_list(ANYMAL_TASK::JOINT_TASK);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  

}

void CoMStateMachine::_taskUpdate() {
  rg_container_->com_trajectory_manager_->updateTask(sp_->curr_time);
  rg_container_->base_ori_trajectory_manager_->updateTask(sp_->curr_time);
  rg_container_->joint_trajectory_manager_->updateTask(sp_->curr_time);
}

void CoMStateMachine::_weightUpdate() {
  // no change in weight
}

void CoMStateMachine::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void CoMStateMachine::lastVisit() {}

bool CoMStateMachine::endOfState() {
  // Also check if footstep list is non-zero
  // std::cout<<"state_machine_time_ = "<<state_machine_time_<<", ctrl_duration_ = " << ctrl_duration_;
  // std::cout<<", sp_->num_state = "<< sp_->num_state<< std::endl;
  if ( state_machine_time_ > ctrl_duration_ && sp_->num_com_state > 0) {
    std::cout << "[CoMStateMachine] End" << std:: endl;
    return true;
  }
  return false;
}

void CoMStateMachine::initialization(const YAML::Node& node) {}
