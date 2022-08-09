#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/manipulation.hpp>

Manipulation::Manipulation(const StateIdentifier state_identifier_in,
    ANYmalReferenceGeneratorContainer* rg_container) 
    : StateMachine(state_identifier_in, rg_container->robot_) {
  my_utils::pretty_constructor(2, "StateMachine: Manipulation");

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

Manipulation::~Manipulation() {}

void Manipulation::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[Full Support Balance] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // -- set current motion param
  ManipulationCommand mc_curr_ = sp_->curr_manipulation_command;
  mc_curr_.printMotionInfo();

  // ---------------------------------------
  //      Planning
  // ---------------------------------------
  // robot goal configuration
  Eigen::VectorXd q_init =  robot_->getQ();
  Eigen::VectorXd q_goal =  robot_->getQ();
  Eigen::Vector3d pcom = robot_->getCoMPosition();
  Eigen::Vector3d pc_goal = robot_->getCoMPosition();

  sp_->com_pos_init = pcom;
  sp_->com_pos_target = pc_goal;


  // CoM planner
  double period = mc_curr_.get_motion_period();
  Eigen::Vector3d zero3d(0.0, 0.0, 0.0); 
  ComMotionCommand mc_com = ComMotionCommand( pcom, zero3d, zero3d, period );      

  
  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->set_contact_list(-1);
  ws_container_->set_contact_maxfz();

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------
  
  // --set com traj
  rg_container_->com_trajectory_manager_
               ->setCoMTrajectory(ctrl_start_time_, mc_com);
  ctrl_duration_ = rg_container_->com_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = rg_container_->com_trajectory_manager_->getTrajEndTime();
  // -- set base ori traj
  rg_container_->base_ori_trajectory_manager_
               ->setBaseOriTrajectory(ctrl_start_time_, ctrl_duration_);
  // -- set EE pos/ori traj
  rg_container_->ee_trajectory_manager_
                ->setEETrajectory(ctrl_start_time_, &mc_curr_);
  // -- set joint traj
  rg_container_->joint_trajectory_manager_
                ->setJointTrajectory(ctrl_start_time_,ctrl_duration_,q_goal);
 
  // -- set task_list in taf with hierachy
  ws_container_->clear_task_list();
  ws_container_->add_task_list(ws_container_->com_task_);
  ws_container_->add_task_list(ws_container_->base_ori_task_);
  ws_container_->add_task_list(ws_container_->ee_pos_task_);
  ws_container_->add_task_list(ws_container_->ee_ori_task_);
  ws_container_->add_task_list(ws_container_->joint_task_);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  
  // ws_container_->W_qddot_ : will be always same  
  ws_container_->set_contact_weight_param();

}

void Manipulation::_taskUpdate() {
  // rg_container_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  rg_container_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->com_task_);

  // rg_container_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  rg_container_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->base_ori_task_);

  rg_container_->ee_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->ee_pos_task_, 
                                      ws_container_->ee_ori_task_);
  
  // rg_container_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  rg_container_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->joint_task_);
}

void Manipulation::_weightUpdate() {
  // no change in weight
  ws_container_->set_contact_weight_param();
  ws_container_->set_contact_maxfz();
}

void Manipulation::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void Manipulation::lastVisit() {}

bool Manipulation::endOfState() {
  // Also check if footstep list is non-zero
  // std::cout<<"state_machine_time_ = "<<state_machine_time_<<", ctrl_duration_ = " << ctrl_duration_;
  // std::cout<<", sp_->num_state = "<< sp_->num_state<< std::endl;
  if ( state_machine_time_ > ctrl_duration_ && sp_->num_state > 0) {
    std::cout << "[Full Support Balance] End" << std:: endl;
    return true;
  }
  return false;
}

void Manipulation::initialization(const YAML::Node& node) {}
